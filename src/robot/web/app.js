const storedClientId = sessionStorage.getItem('robot_client_id');
const clientId = storedClientId || crypto.randomUUID();
sessionStorage.setItem('robot_client_id', clientId);

const elements = Object.fromEntries([
  'connectionBadge', 'leaseBadge', 'missionMessage', 'taskValue',
  'controllerValue', 'clientValue', 'queueState', 'chatLog', 'chatForm',
  'chatInput', 'stopButton', 'cancelButton', 'releaseButton',
  'exploreButton', 'followButton', 'previewPanel', 'previewText',
  'candidateList', 'confirmButton', 'closePreview', 'cameraCanvas',
  'cameraEmpty', 'mapCanvas', 'mapEmpty', 'mapState', 'detectionCount',
  'detectionChips', 'healthGrid', 'captureButton'
].map(id => [id, document.getElementById(id)]));

elements.clientValue.textContent = clientId.slice(0, 8);
let websocket;
let heartbeatTimer;
let reconnectTimer;
let latestDetections = [];
let activePreview = null;
let latestCameraBitmap = null;

function requestId() { return crypto.randomUUID(); }

async function api(path, body = null) {
  const options = body ? {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body)
  } : {};
  const response = await fetch(path, options);
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(payload.message || `请求失败 ${response.status}`);
  return payload;
}

function connectWebSocket() {
  clearTimeout(reconnectTimer);
  const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
  websocket = new WebSocket(`${protocol}://${location.host}/ws/state?client_id=${encodeURIComponent(clientId)}`);
  websocket.onopen = () => {
    setConnection(true);
    clearInterval(heartbeatTimer);
    heartbeatTimer = setInterval(() => {
      if (websocket.readyState === WebSocket.OPEN) websocket.send(JSON.stringify({type: 'heartbeat'}));
    }, 5000);
  };
  websocket.onmessage = event => handleSocketMessage(JSON.parse(event.data));
  websocket.onclose = () => {
    setConnection(false);
    clearInterval(heartbeatTimer);
    reconnectTimer = setTimeout(connectWebSocket, 1500);
  };
  websocket.onerror = () => websocket.close();
}

function setConnection(connected) {
  elements.connectionBadge.textContent = connected ? '局域网已连接' : '连接已断开';
  elements.connectionBadge.className = `badge ${connected ? '' : 'error'}`;
}

function handleSocketMessage(message) {
  if (message.type === 'state') updateSharedState(message);
  if (message.type === 'chat_status') {
    elements.queueState.textContent = message.state === 'queued'
      ? `排队第 ${message.position} 位` : '模型正在回答';
  }
  if (message.type === 'chat_result') {
    appendMessage('robot', message.answer);
    elements.queueState.textContent = message.state === 'completed' ? '推理队列空闲' : '模型调用失败';
  }
}

function updateSharedState(state) {
  const lease = state.lease || {};
  const mission = state.mission || {};
  latestDetections = state.detections?.detections || [];
  elements.leaseBadge.textContent = lease.controller_short_id
    ? `控制者 ${lease.controller_short_id}` : '控制权空闲';
  elements.leaseBadge.className = `badge ${lease.controller_short_id ? 'warning' : ''}`;
  elements.missionMessage.textContent = mission.message || lease.message || '等待任务';
  elements.taskValue.textContent = translateTask(lease.task || mission.task) || '无';
  elements.controllerValue.textContent = lease.controller_short_id || '无';
  elements.detectionCount.textContent = latestDetections.length
    ? `${latestDetections.length} 个目标` : '未识别到目标';
  drawDetectionChips();
  drawHealth(state.health || {});
}

function translateTask(task) {
  return ({explore: '自主探索', follow_person: '跟随人员', goto_object: '前往物体'})[task] || task || '';
}

function drawDetectionChips() {
  elements.detectionChips.replaceChildren(...latestDetections.map(item => {
    const chip = document.createElement('span');
    chip.className = 'chip';
    const distance = item.distance == null ? '深度无效' : `${Number(item.distance).toFixed(2)}m`;
    chip.textContent = `${item.label_zh || item.class_name} · ${distance} · ${Math.round((item.confidence || 0) * 100)}%`;
    return chip;
  }));
}

function drawHealth(health) {
  const labels = {web: '网页', camera: '相机', semantic: '语义', slam: '建图', nav2: '规划'};
  elements.healthGrid.replaceChildren(...Object.entries(labels).map(([key, label]) => {
    const state = health[key]?.state || 'waiting';
    const item = document.createElement('div');
    item.className = `health-item ${state === 'ok' ? 'ok' : (state === 'error' ? 'error' : '')}`;
    item.innerHTML = `<strong>${label}</strong><span>${state}</span>`;
    return item;
  }));
}

function appendMessage(kind, text) {
  const message = document.createElement('div');
  message.className = `message ${kind}`;
  message.textContent = text;
  elements.chatLog.appendChild(message);
  elements.chatLog.scrollTop = elements.chatLog.scrollHeight;
}

elements.chatForm.addEventListener('submit', async event => {
  event.preventDefault();
  const text = elements.chatInput.value.trim();
  if (!text) return;
  elements.chatInput.value = '';
  appendMessage('user', text);
  try {
    const result = await api('/api/chat', {client_id: clientId, request_id: requestId(), text});
    if (result.kind === 'mission_preview') showPreview(result);
  } catch (error) { appendMessage('system', error.message); }
});

elements.chatInput.addEventListener('keydown', event => {
  if (event.key === 'Enter' && !event.shiftKey) {
    event.preventDefault();
    elements.chatForm.requestSubmit();
  }
});

elements.exploreButton.addEventListener('click', () => createPreview('explore'));
elements.followButton.addEventListener('click', () => createPreview('follow_person'));

async function createPreview(task, targetId = '') {
  try {
    const result = await api('/api/missions/preview', {
      client_id: clientId, request_id: requestId(), task, target_id: targetId
    });
    showPreview(result);
  } catch (error) { appendMessage('system', error.message); }
}

function showPreview(preview) {
  activePreview = preview;
  elements.previewPanel.classList.remove('hidden');
  const candidates = preview.candidates || [];
  elements.previewText.textContent = `${translateTask(preview.task)}：确认后将取得控制权。当前版本只生成目标和路径，不会运动。`;
  elements.candidateList.replaceChildren(...candidates.map((item, index) => {
    const label = document.createElement('label');
    label.className = 'candidate';
    const distance = item.distance == null ? '深度无效' : `${Number(item.distance).toFixed(2)}m`;
    label.innerHTML = `<input type="radio" name="candidate" value="${item.id}" ${index === 0 ? 'checked' : ''}>
      <span><strong>${item.label_zh || item.class_name}</strong><br><small>${item.id}</small></span>
      <small>${distance} · ${Math.round((item.confidence || 0) * 100)}%</small>`;
    return label;
  }));
  const needsTarget = preview.task !== 'explore';
  elements.confirmButton.disabled = needsTarget && candidates.length === 0;
}

elements.confirmButton.addEventListener('click', async () => {
  if (!activePreview) return;
  const selected = document.querySelector('input[name="candidate"]:checked');
  try {
    const result = await api(`/api/missions/${activePreview.mission_id}/confirm`, {
      client_id: clientId, request_id: requestId(),
      target_id: selected?.value || activePreview.selected_target_id || ''
    });
    appendMessage('system', result.message);
    elements.previewPanel.classList.add('hidden');
    activePreview = null;
  } catch (error) { appendMessage('system', error.message); }
});

elements.closePreview.addEventListener('click', () => {
  elements.previewPanel.classList.add('hidden');
  activePreview = null;
});

elements.cancelButton.addEventListener('click', () => controlRequest('/api/missions/cancel'));
elements.releaseButton.addEventListener('click', () => controlRequest('/api/control/release'));
elements.stopButton.addEventListener('click', () => controlRequest('/api/stop'));
elements.captureButton.addEventListener('click', async () => {
  const sceneId = window.prompt('请输入场景编号（例如 studyroom_near）', 'scene');
  if (sceneId === null) return;
  try {
    const result = await api('/api/dataset/capture', {
      client_id: clientId, request_id: requestId(), scene_id: sceneId
    });
    appendMessage('system', result.message);
  } catch (error) { appendMessage('system', error.message); }
});

async function controlRequest(path) {
  try {
    const result = await api(path, {client_id: clientId, request_id: requestId()});
    appendMessage('system', result.message);
  } catch (error) { appendMessage('system', error.message); }
}

async function updateCamera() {
  try {
    const response = await fetch(`/api/frame.jpg?t=${Date.now()}`, {cache: 'no-store'});
    if (response.status === 204) return;
    const bitmap = await createImageBitmap(await response.blob());
    if (latestCameraBitmap) latestCameraBitmap.close();
    latestCameraBitmap = bitmap;
    const canvas = elements.cameraCanvas;
    canvas.width = bitmap.width;
    canvas.height = bitmap.height;
    const context = canvas.getContext('2d');
    context.drawImage(bitmap, 0, 0);
    context.lineWidth = Math.max(2, bitmap.width / 320);
    context.font = `${Math.max(13, bitmap.width / 42)}px sans-serif`;
    latestDetections.forEach(item => {
      const [x1, y1, x2, y2] = item.bbox || [];
      if (![x1, y1, x2, y2].every(Number.isFinite)) return;
      context.strokeStyle = '#6df0a6';
      context.fillStyle = '#6df0a6';
      context.strokeRect(x1, y1, x2 - x1, y2 - y1);
      context.fillText(item.label_zh || item.class_name, x1 + 3, Math.max(18, y1 - 5));
    });
    elements.cameraEmpty.classList.add('hidden');
  } catch (_) { /* 相机尚未就绪时保留等待提示。 */ }
}

async function updateMap() {
  try {
    const map = await api('/api/map');
    const canvas = elements.mapCanvas;
    canvas.width = map.width;
    canvas.height = map.height;
    const context = canvas.getContext('2d');
    const image = context.createImageData(map.width, map.height);
    map.data.forEach((value, index) => {
      const color = value < 0 ? 55 : (value >= 65 ? 15 : 225);
      const row = Math.floor(index / map.width);
      const flipped = (map.height - 1 - row) * map.width + (index % map.width);
      const offset = flipped * 4;
      image.data[offset] = color;
      image.data[offset + 1] = value < 0 ? 65 : color;
      image.data[offset + 2] = value < 0 ? 60 : color;
      image.data[offset + 3] = 255;
    });
    context.putImageData(image, 0, 0);
    elements.mapEmpty.classList.add('hidden');
    elements.mapState.textContent = `${map.width}×${map.height} · ${map.resolution.toFixed(2)}m`;
  } catch (_) { /* SLAM 未发布地图时继续等待。 */ }
}

connectWebSocket();
setInterval(updateCamera, 250);
setInterval(updateMap, 1500);
updateCamera();
updateMap();
