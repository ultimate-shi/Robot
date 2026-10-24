/* 作用：控制台导航、人工控制及实时数据显示；使用方法：由 /robot/ 加载，地图支持滚轮缩放、拖动平移与点击选点。 */
(() => {
  const $ = id => document.getElementById(id);
  const clientId = `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
  let state = null;
  let map = null;
  let selectedGoal = null;
  let preview = null;
  let pressed = '';
  let driveSequence = 0;
  let camera = 'right';
  let frameBusy = false;
  let frameUrl = '';
  let lastMapFetch = 0;
  let pollBusy = false;
  let navPollBusy = false;
  let lastLog = 0;
  let logPaused = false;
  let logLines = [];
  let pendingModelTask = null;
  let statusConnected = false;
  let legacyBackend = false;
  const api = async (path, body) => {
    if (legacyBackend && body !== undefined && path !== 'stop')
      throw new Error('整车仍运行旧版控制台；请重启 robot.launch.py 后刷新页面。');
    const started = performance.now();
    const response = await fetch('/robot/api/' + path, {
      method: body === undefined ? 'GET' : 'POST',
      headers: {'Content-Type': 'application/json'},
      body: body === undefined ? undefined : JSON.stringify(body),
      cache: 'no-store',
    });
    const payload = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(payload.detail || `请求失败 ${response.status}`);
    $('connection').textContent = `已连接 · ${Math.round(performance.now() - started)} ms`;
    return payload;
  };
  const notice = (message, error = false) => {
    $('notice').textContent = message;
    $('notice').style.color = error ? '#ff8291' : '';
  };
  const guarded = async work => {
    try { return await work(); } catch (error) { notice(error.message, true); return null; }
  };
  const route = () => {
    const page = location.pathname.endsWith('/status') ? 'status' :
      location.pathname.endsWith('/console') ? 'logs' : 'home';
    for (const name of ['home','status','logs']) $(name).hidden = name !== page;
    $('sideStop').hidden = page === 'home';
    document.querySelectorAll('[data-page]').forEach(link =>
      link.classList.toggle('active', link.dataset.page === page));
  };
  document.querySelectorAll('[data-page]').forEach(link => link.addEventListener('click', event => {
    event.preventDefault(); history.pushState({}, '', link.href); route();
  }));
  window.addEventListener('popstate', route); route();
  const stop = async () => {
    pressed = '';
    const result = await guarded(() => api('stop', {}));
    if (result) notice('软件停止已锁定；请在首页明确解除。');
  };
  $('stop').addEventListener('click', stop);
  $('sideStop').addEventListener('click', stop);
  $('resetStop').addEventListener('click', () => guarded(async () => {
    await api('reset-stop', {}); notice('软件停止已解除。');
  }));
  const sendDrive = (linear, angular) =>
    api('drive', {linear, angular, client_id:clientId,
      sequence:++driveSequence});
  const velocity = () => ({linear: Number($('linearSpeed').value), angular: Number($('angularSpeed').value)});
  const driveValues = key => {
    const speed = velocity();
    return key === 'forward' ? [speed.linear, 0] :
      key === 'back' ? [-speed.linear, 0] :
      key === 'left' ? [0, speed.angular] :
      key === 'right' ? [0, -speed.angular] : [0, 0];
  };
  const hold = key => { pressed = key; guarded(() => sendDrive(...driveValues(key))); };
  const releaseDrive = () => {
    const wasPressed = Boolean(pressed);
    pressed = '';
    if (wasPressed) guarded(() => sendDrive(0, 0));
  };
  document.querySelectorAll('[data-drive]').forEach(button => {
    const key = button.dataset.drive;
    button.addEventListener('pointerdown', event => { event.preventDefault(); hold(key); });
    button.addEventListener('pointerup', releaseDrive);
    button.addEventListener('pointercancel', releaseDrive);
    button.addEventListener('pointerleave', releaseDrive);
  });
  const keys = {w:'forward',ArrowUp:'forward',s:'back',ArrowDown:'back',
    a:'left',ArrowLeft:'left',d:'right',ArrowRight:'right',' ':'stop'};
  const textFocus = element => element && (
    element.tagName === 'TEXTAREA' || element.tagName === 'SELECT' ||
    element.isContentEditable || (element.tagName === 'INPUT' &&
      ['text','number','search','password','email','url'].includes(element.type)));
  window.addEventListener('keydown', event => {
    if (!$('keyboardEnabled').checked || textFocus(document.activeElement) ||
        document.hidden || $('home').hidden) return;
    const key = keys[event.key] || keys[event.key.toLowerCase()];
    if (!key) return;
    event.preventDefault();
    if (!event.repeat && key !== pressed) hold(key);
  });
  window.addEventListener('keyup', event => {
    const key = keys[event.key] || keys[event.key.toLowerCase()];
    if (key && key === pressed) { event.preventDefault(); releaseDrive(); }
  });
  window.addEventListener('blur', releaseDrive);
  document.addEventListener('visibilitychange', () => { if (document.hidden) releaseDrive(); });
  window.addEventListener('pagehide', () => {
    if (pressed) navigator.sendBeacon('/robot/api/drive', new Blob([
      JSON.stringify({linear:0, angular:0, client_id:clientId,
        sequence:++driveSequence})], {type:'application/json'}));
  });
  setInterval(() => {
    if (pressed) guarded(() => sendDrive(...driveValues(pressed)));
  }, 100);
  for (const [id, output, suffix] of [['linearSpeed','linearValue',' m/s'],
                                       ['angularSpeed','angularValue',' rad/s']]) {
    $(id).addEventListener('input', () => { $(output).textContent = $(id).value + suffix; });
  }
  document.querySelectorAll('[data-speed]').forEach(button => button.addEventListener('click', () => {
    const profile = {low:[0.04,0.15],normal:[0.07,0.3],high:[0.1,0.45]}[button.dataset.speed];
    $('linearSpeed').value = profile[0]; $('angularSpeed').value = profile[1];
    $('linearSpeed').dispatchEvent(new Event('input'));
    $('angularSpeed').dispatchEvent(new Event('input'));
  }));
  document.querySelectorAll('[data-mode]').forEach(button => button.addEventListener('click', () => guarded(async () => {
    releaseDrive();
    const mode = button.dataset.mode;
    await api('mode', {mode});
    showMode(mode); notice('已切换到' + button.textContent.trim());
  })));
  function showMode(mode) {
    for (const name of ['manual','navigation','explore','follow'])
      $(name + 'Controls').hidden = name !== mode;
    document.querySelectorAll('[data-mode]').forEach(button =>
      button.classList.toggle('selected', button.dataset.mode === mode));
  }
  // 栅格缓存在离屏画布；视口按屏幕像素绘制，避免地图增长撑开布局或放大标记。
  const mapRaster = document.createElement('canvas');
  let initialPick = null;
  let initialFormEdited = false;
  let initialHint = null;
  let rasterMap = null;
  let centerOnRobot = false;
  let centeredMapScale = null;
  let mapView = null;
  // 手动视图以世界坐标和像素/米保存，建图原点、栅格尺寸改变时仍保持局部位置。
  let manualMapView = null;
  let mapDrag = null;
  let suppressMapClick = false;
  const mapWorldAt = (x, y, view = mapView) => {
    const gx = (x - view.offsetX) / view.scale;
    const gy = view.height - (y - view.offsetY) / view.scale;
    return {x: view.origin.x + view.resolution * (Math.cos(view.yaw)*gx - Math.sin(view.yaw)*gy),
      y: view.origin.y + view.resolution * (Math.sin(view.yaw)*gx + Math.cos(view.yaw)*gy)};
  };
  const renderMap = () => {
    if (!map) return;
    const canvas = $('mapCanvas');
    const bounds = canvas.getBoundingClientRect();
    if (!bounds.width || !bounds.height) return;
    const width = map.width, height = map.height;
    const pixelRatio = window.devicePixelRatio || 1;
    canvas.width = Math.round(bounds.width * pixelRatio);
    canvas.height = Math.round(bounds.height * pixelRatio);
    const context = canvas.getContext('2d');
    context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
    if (rasterMap !== map) {
      mapRaster.width = width; mapRaster.height = height;
      const rasterContext = mapRaster.getContext('2d');
      const image = rasterContext.createImageData(width, height);
      for (let y = 0; y < height; y++) for (let x = 0; x < width; x++) {
        const value = map.data[(height - y - 1) * width + x];
        const shade = value < 0 ? 58 : 235 - Math.round(value * 2.1);
        const index = (y * width + x) * 4;
        image.data[index] = shade; image.data[index+1] = shade;
        image.data[index+2] = shade; image.data[index+3] = 255;
      }
      rasterContext.putImageData(image, 0, 0);
      rasterMap = map;
    }
    const mapYaw = 2 * Math.atan2(map.origin.z, map.origin.w);
    const toGrid = (x, y) => {
      const dx = x - map.origin.x, dy = y - map.origin.y;
      return [(Math.cos(mapYaw)*dx + Math.sin(mapYaw)*dy) / map.resolution,
        height - (-Math.sin(mapYaw)*dx + Math.cos(mapYaw)*dy) / map.resolution];
    };
    const robot = state?.localization?.required && !state.localization.ready ? null : state?.pose;
    let center = centerOnRobot && robot?.fresh ?
      toGrid(robot.x, robot.y) : [width / 2, height / 2];
    // 全图适配只用于默认视图；机器人居中保留点击按钮时的像素/米比例。
    const fitScale = Math.min(Math.max(1, bounds.width - 16) / width,
      Math.max(1, bounds.height - 16) / height);
    if (manualMapView) center = toGrid(manualMapView.x, manualMapView.y);
    const scale = manualMapView ? manualMapView.scale * map.resolution :
      centerOnRobot && centeredMapScale !== null ? centeredMapScale * map.resolution : fitScale;
    const offsetX = bounds.width / 2 - center[0] * scale;
    const offsetY = bounds.height / 2 - center[1] * scale;
    mapView = {scale, fitScale, offsetX, offsetY, width: bounds.width,
      viewportHeight: bounds.height, height, origin: map.origin,
      resolution: map.resolution, yaw: mapYaw};
    context.imageSmoothingEnabled = false;
    context.drawImage(mapRaster, offsetX, offsetY, width * scale, height * scale);
    const toPixel = (x, y) => {
      const [gx, gy] = toGrid(x, y);
      return [offsetX + gx * scale, offsetY + gy * scale];
    };
    if (preview?.path?.length) {
      context.strokeStyle = '#468fee'; context.lineWidth = 1.5;
      context.beginPath();
      preview.path.forEach((point, index) => {
        const [x,y] = toPixel(point.x, point.y);
        if (index === 0) context.moveTo(x,y); else context.lineTo(x,y);
      });
      context.stroke();
    }
    if (state?.pose?.fresh && !(state.localization?.required && !state.localization.ready)) {
      const [x,y] = toPixel(state.pose.x,state.pose.y);
      context.save(); context.translate(x,y);
      context.fillStyle = '#20c790'; context.strokeStyle = '#0d1621';
      context.lineWidth = 1;
      context.beginPath(); context.arc(0,0,3,0,2*Math.PI);
      context.fill(); context.stroke();
      // 地图画布的 Y 轴向下，车头相对地图的朝向需要反向旋转。
      if (Number.isFinite(state.pose.yaw)) {
        context.rotate(mapYaw - state.pose.yaw);
        context.beginPath(); context.moveTo(2,-3);
        context.lineTo(8,0); context.lineTo(2,3);
        context.closePath(); context.fill(); context.stroke();
      }
      context.restore();
    }
    if (initialHint) {
      const [px,py] = toPixel(initialHint.x,initialHint.y);
      context.save(); context.translate(px,py);
      context.strokeStyle = '#ef9a24'; context.lineWidth = 2;
      context.beginPath(); context.arc(0,0,5,0,2*Math.PI); context.stroke();
      if (Number.isFinite(initialHint.yaw)) {
        context.rotate(mapYaw-initialHint.yaw);
        context.beginPath(); context.moveTo(0,0); context.lineTo(24,0);
        context.lineTo(17,-5); context.moveTo(24,0); context.lineTo(17,5); context.stroke();
      }
      context.restore();
    }
    if (selectedGoal) {
      const [cx,cy] = toPixel(selectedGoal.x,selectedGoal.y);
      context.fillStyle = '#e84158'; context.beginPath(); context.arc(cx,cy,3,0,2*Math.PI); context.fill();
    }
    $('mapEmpty').hidden = true;
  };
  $('mapCanvas').addEventListener('click', event => {
    if (suppressMapClick) { suppressMapClick = false; return; }
    if (!map || !mapView) return;
    const bounds = event.currentTarget.getBoundingClientRect();
    const cx = (event.clientX - bounds.left - mapView.offsetX) / mapView.scale;
    const gridY = (event.clientY - bounds.top - mapView.offsetY) / mapView.scale;
    // 留白区不是地图，不能在此生成导航目标。
    if (cx < 0 || cx >= map.width || gridY < 0 || gridY >= map.height) return;
    if (initialPick) {
      const point = mapWorldAt(event.clientX - bounds.left, event.clientY - bounds.top);
      if (initialPick === 'position') {
        initialHint = {...point, yaw: null}; initialPick = 'heading';
        $('initialX').value = point.x.toFixed(2); $('initialY').value = point.y.toFixed(2);
        notice('位置已选；再点击地图上的一点指定车头朝向。');
      } else {
        const dx = point.x - initialHint.x, dy = point.y - initialHint.y;
        if (Math.hypot(dx,dy) < map.resolution) { notice('朝向点请离位置点稍远一些。', true); return; }
        initialHint.yaw = Math.atan2(dy,dx);
        $('initialYaw').value = (initialHint.yaw * 180 / Math.PI).toFixed(1);
        initialPick = null; notice('位置和朝向已选，请核对箭头后点击“设置初始位姿”。');
      }
      renderMap(); return;
    }
    selectedGoal = {...mapWorldAt(event.clientX - bounds.left, event.clientY - bounds.top),
      yaw:state?.pose?.fresh ? state.pose.yaw : null};
    $('mapGoal').textContent = `目标：(${selectedGoal.x.toFixed(2)}, ${selectedGoal.y.toFixed(2)}) m`;
    preview = null; renderMap();
  });
  $('mapCanvas').addEventListener('wheel', event => {
    if (!mapView || !map || mapDrag) return;
    event.preventDefault();
    const bounds = event.currentTarget.getBoundingClientRect();
    // WheelEvent 的增量可能以像素、行或页为单位。
    const delta = event.deltaY * (event.deltaMode === 1 ? 16 :
      event.deltaMode === 2 ? bounds.height : 1);
    const scale = Math.max(mapView.fitScale / 4, Math.min(mapView.fitScale * 64,
      mapView.scale * Math.exp(Math.max(-2, Math.min(2, -delta * 0.0015)))));
    const ratio = scale / mapView.scale;
    const center = mapWorldAt(bounds.width / 2 +
      (event.clientX - bounds.left - bounds.width / 2) * (1 - 1 / ratio),
      bounds.height / 2 +
      (event.clientY - bounds.top - bounds.height / 2) * (1 - 1 / ratio));
    centerOnRobot = false;
    manualMapView = {...center, scale: scale / map.resolution};
    renderMap();
  }, {passive: false});
  $('mapCanvas').addEventListener('pointerdown', event => {
    if (!mapView || !map || event.button !== 0 || mapDrag) return;
    suppressMapClick = false;
    mapDrag = {id: event.pointerId, x: event.clientX, y: event.clientY,
      view: mapView, moved: false};
    event.currentTarget.setPointerCapture(event.pointerId);
  });
  $('mapCanvas').addEventListener('pointermove', event => {
    if (!mapDrag || mapDrag.id !== event.pointerId) return;
    const dx = event.clientX - mapDrag.x, dy = event.clientY - mapDrag.y;
    if (!mapDrag.moved && Math.hypot(dx, dy) < 4) return;
    mapDrag.moved = true;
    suppressMapClick = true;
    const view = mapDrag.view;
    manualMapView = {...mapWorldAt(view.width / 2 - dx,
      view.viewportHeight / 2 - dy, view), scale: view.scale / view.resolution};
    centerOnRobot = false;
    event.currentTarget.classList.add('dragging');
    renderMap();
  });
  const finishMapDrag = event => {
    if (!mapDrag || mapDrag.id !== event.pointerId) return;
    // 拖动或取消之后浏览器仍可能发出 click，必须阻止它生成导航目标。
    suppressMapClick = mapDrag.moved || event.type !== 'pointerup';
    mapDrag = null;
    event.currentTarget.classList.remove('dragging');
    if (event.currentTarget.hasPointerCapture(event.pointerId))
      event.currentTarget.releasePointerCapture(event.pointerId);
  };
  for (const name of ['pointerup', 'pointercancel', 'lostpointercapture'])
    $('mapCanvas').addEventListener(name, finishMapDrag);
  $('centerMap').addEventListener('click', () => {
    if (!map || !state?.pose?.fresh || (state.localization?.required && !state.localization.ready)) {
      notice('地图或机器人定位不可用，请等待恢复后再居中。', true); return;
    }
    if (!mapView) renderMap();
    if (!mapView) return;
    centeredMapScale = mapView.scale / mapView.resolution;
    manualMapView = null; centerOnRobot = true; renderMap();
  });
  $('fitMap').addEventListener('click', () => { manualMapView = null; centeredMapScale = null; centerOnRobot = false; renderMap(); });
  // 页面重新显示、屏幕旋转或容器大小变化时重新适配地图。
  new ResizeObserver(() => renderMap()).observe($('mapCanvas'));
  for (const id of ['initialX','initialY','initialYaw'])
    $(id).addEventListener('input', () => { initialFormEdited = true; });
  $('pickInitialPose').addEventListener('click', () => {
    initialFormEdited = true;
    initialPick = 'position'; initialHint = null; preview = null;
    notice('先点击地图上实车所在的位置，再点击车头指向的位置。');
  });
  $('cancelInitialPose').addEventListener('click', () => {
    initialPick = null; initialHint = null; renderMap(); notice('已取消初始位姿选取。');
  });
  $('submitInitialPose').addEventListener('click', () => guarded(async () => {
    const inputs = ['initialX','initialY','initialYaw'].map(id => $(id).value.trim());
    const [x,y,degrees] = inputs.map(Number);
    if (inputs.some(value => value === '') || ![x,y,degrees].every(Number.isFinite))
      throw new Error('请填写有效的位置和朝向');
    if (initialPick) throw new Error('请先完成位置及朝向选取，或取消选取后输入数值');
    preview = null;
    const result = await api('localization/initial-pose', {x,y,yaw:degrees*Math.PI/180});
    notice(result.message); initialHint = {x,y,yaw:degrees*Math.PI/180};
  }));
  $('retryLocalization').addEventListener('click', () => guarded(async () => {
    preview = null;
    const result = await api('localization/retry', {}); notice(result.message);
  }));
  $('planMap').addEventListener('click', () => guarded(async () => {
    if (!selectedGoal) throw new Error('请先在地图上选目标');
    if (!Number.isFinite(selectedGoal.yaw))
      throw new Error('机器人当前朝向不可用，请等待定位恢复后重新选择目标');
    preview = null;
    preview = await api('plan', selectedGoal);
    $('mapGoal').textContent = `规划完成：${preview.path.length} 个路径点，等待确认。`;
  }));
  $('confirmMap').addEventListener('click', () => guarded(async () => {
    if (!preview) throw new Error('请先规划路径');
    await api('navigate/' + preview.id, {}); preview = null;
    notice('Nav2 已接收导航目标。');
  }));
  $('cancelNav').addEventListener('click', () => guarded(async () => {
    await api('cancel', {}); preview = null; notice('导航已取消并停车。');
  }));
  $('saveMap').addEventListener('click', () => guarded(async () => {
    const button = $('saveMap');
    button.disabled = true;
    button.textContent = '正在保存…';
    try {
      notice('正在保存地图和定位数据库，可能需要一分钟，请保持停车并等待结果…');
      const result = await api('save-map', {});
      notice(result.success ? `地图已保存：${result.message}` : result.message, !result.success);
    } finally {
      button.disabled = false;
      button.textContent = '保存地图';
    }
  }));
  document.querySelectorAll('[data-camera]').forEach(button => button.addEventListener('click', () => {
    camera = button.dataset.camera;
    document.querySelectorAll('[data-camera]').forEach(item =>
      item.classList.toggle('selected', item === button));
  }));
  $('planObject').addEventListener('click', () => guarded(async () => {
    const detection = state?.detections.find(item => item.id === $('target').value);
    if (!detection?.map_position || !detection.distance)
      throw new Error('目标缺少可靠深度或地图坐标');
    const robot = state?.localization?.required && !state.localization.ready ? null : state?.pose;
    if (!robot?.fresh) throw new Error('机器人地图位姿不可用');
    const dx = detection.map_position[0] - robot.x, dy = detection.map_position[1] - robot.y;
    const range = Math.hypot(dx,dy);
    if (range <= 0.75) throw new Error('目标距离过近，无法生成安全停靠点');
    selectedGoal = {x:detection.map_position[0]-dx/range*0.75,
      y:detection.map_position[1]-dy/range*0.75,yaw:Math.atan2(dy,dx)};
    preview = null;
    preview = await api('plan', selectedGoal); renderMap();
    $('mapGoal').textContent = `物体目标已规划：${preview.path.length} 个路径点，请确认前往。`;
  }));
  document.querySelectorAll('[data-task]').forEach(button => button.addEventListener('click', () => guarded(async () => {
    const task = button.dataset.task;
    const payload = {task, target_id: $('followTarget').value,
      follow_distance: Number($('followDistance').value),
      max_speed: Number($('followSpeed').value)};
    const result = await api('task', payload);
    notice(`任务操作完成：${task}`);
    $('exploreState').textContent = result.paused ? '已暂停' : result.task || '已停止';
    $('followState').textContent = result.paused ? '已暂停' : result.task || '已停止';
  })));
  $('parseTask').addEventListener('click', () => guarded(async () => {
    pendingModelTask = null;
    $('confirmTask').disabled = true;
    $('taskResult').textContent = '模型正在解析…';
    const result = await api('model/parse', {text:$('taskText').value,
      provider:$('modelSource').value});
    $('taskResult').textContent = result.answer + (result.action ?
      ` · 待确认任务：${result.action}` : ' · 没有可执行任务');
    pendingModelTask = result.proposal_id;
    $('confirmTask').disabled = !pendingModelTask;
  }));
  $('confirmTask').addEventListener('click', () => guarded(async () => {
    if (!pendingModelTask) throw new Error('没有待确认任务');
    await api('model/confirm', {proposal_id:pendingModelTask});
    pendingModelTask = null; $('confirmTask').disabled = true;
    notice('模型任务已确认并交给机器人任务层。');
  }));
  $('discardTask').addEventListener('click', () => {
    pendingModelTask = null; $('confirmTask').disabled = true;
    $('taskResult').textContent = '模型任务已取消。';
  });
  const setRows = (element, entries) => {
    element.replaceChildren();
    for (const [key, value] of entries) {
      const label = document.createElement('dt'); label.textContent = key;
      const content = document.createElement('dd'); content.textContent = value;
      element.append(label, content);
    }
  };
  const renderStatus = data => {
    state = data;
    $('robotMode').textContent = data.mode_source === 'real' ? '当前：实机' : '当前：仿真';
    $('overall').textContent = data.control.stopped ? '软件停止已锁定' : '控制链运行中';
    const hardware = data.hardware;
    const hardwareAlert = $('hardwareAlert');
    let hardwareMessage = '';
    if (data.mode_source === 'real') {
      if (!hardware || !hardware.fresh)
        hardwareMessage = `底盘反馈已中断${hardware?.fault ? '；最近故障：' + hardware.fault : ''}。请检查硬件和控制器状态。`;
      else if (hardware.manual_recovery_required)
        hardwareMessage = `硬件已锁定：${hardware.fault || '请检查控制板故障'}。排除故障后再恢复。`;
      else if (!hardware.outputs_enabled || !hardware.connected)
        hardwareMessage = `底盘输出不可用：${hardware.fault || '控制板未全部连接或未使能'}。`;
      else if (!hardware.watchdog_ok)
        hardwareMessage = '底盘命令心跳异常，轮速已停车。';
      else if (hardware.fault)
        hardwareMessage = `底盘提示：${hardware.fault}`;
    }
    hardwareAlert.textContent = hardwareMessage;
    hardwareAlert.hidden = !hardwareMessage;
    const localization = data.localization || {required:data.mode_source === 'real',ready:false,
      state:'unavailable',message:'当前控制台尚未提供定位接口，请重启整车后刷新页面'};
    $('localizationPanel').hidden = !localization.required;
    const phases = {initializing:'视觉地图加载中',waiting:'等待定位',verifying:'正在确认定位',localized:'定位成功',
      failed:'定位超时',lost:'定位丢失',unavailable:'定位服务不可用'};
    $('localizationPhase').textContent = phases[localization.state] || localization.message;
    $('localizationMessage').textContent = localization.message +
      (localization.matches != null ? `（有效匹配 ${localization.matches}/${localization.required_matches}）` : '');
    $('localizationPanel').dataset.state = localization.state;
    if (!initialFormEdited && !initialHint && localization.initial_pose) {
      const hint = localization.initial_pose;
      if ([hint.x,hint.y,hint.yaw].every(Number.isFinite)) {
        $('initialX').value = hint.x.toFixed(3);
        $('initialY').value = hint.y.toFixed(3);
        $('initialYaw').value = (hint.yaw*180/Math.PI).toFixed(1);
        initialHint = {...hint};
      }
    }

    const blocked = localization.required && !localization.ready;
    if (blocked) preview = null;
    for (const id of ['planMap','confirmMap','planObject','confirmTask'])
      $(id).disabled = blocked || (id === 'confirmTask' && !pendingModelTask);
    document.querySelectorAll('[data-mode]').forEach(button => {
      button.disabled = blocked && button.dataset.mode !== 'manual';
    });
    document.querySelectorAll('[data-task]').forEach(button => {
      button.disabled = blocked && ['explore-start','follow-start','resume'].includes(button.dataset.task);
    });
    showMode(data.control.mode);
    renderNavigation(data);
    setRows($('robotStatus'), [
      ['模式',data.control.mode],['控制台接口',data.control.connected?'在线':'不可用'],
      ['软件停止',data.control.stopped?'已锁定':'已解除'],
      ['地图',data.map_ready?'可用':data.map_source || '等待地图'],
      ['定位',localization.message],
      ['位姿',data.pose?.fresh && !blocked?`${data.pose.x.toFixed(2)}, ${data.pose.y.toFixed(2)} m`:'不可用'],
      ['速度',data.odom?.fresh?`${data.odom.linear.toFixed(2)} m/s`:'不可用'],
      ['任务',data.task || '空闲']]);
    const controllerNames = [
      ['joint_state_broadcaster','关节状态广播器'],
      ['steering_controller','转向控制器'],
      ['wheel_controller','轮子控制器'],
      ['head_controller','头部控制器'],
      ['lap_fr_position_controller','右前腿控制器'],
      ['lap_fl_position_controller','左前腿控制器'],
      ['lap_rr_position_controller','右后腿控制器'],
      ['lap_rl_position_controller','左后腿控制器']];
    const controllers = data.controllers || {};
    const controllerState = name => !controllers.available ? '管理器未连接' :
      ({active:'运行中',inactive:'未激活',unconfigured:'未配置'}[controllers.states?.[name]] ||
      controllers.states?.[name] || '未加载');
    const boardState = name => !data.hardware?.fresh ? '无新反馈' :
      data.hardware[name] ? '已连接' : '断开';
    setRows($('controllerStatus'), [
      ...controllerNames.map(([name,label]) => [label,controllerState(name)]),
      ['DMC0 控制板',boardState('dmc0_connected')],
      ['DMC1 控制板',boardState('dmc1_connected')],
      ['SE2 控制板',boardState('se2_connected')],
      ['硬件输出',data.hardware?.fresh ?
        data.hardware.outputs_enabled?'已使能':'未使能' : '无新反馈']]);
    const sonarNames = [
      ['front_left','前左超声波'],['front_center','前中超声波'],
      ['front_right','前右超声波'],['rear_left','后左超声波'],
      ['rear_center','后中超声波'],['rear_right','后右超声波'],
      ['left','左侧超声波'],['right','右侧超声波']];
    const sonarState = name => {
      const item = data.ultrasonic?.[name];
      return !item?.received ? '无新数据' : item.fresh ?
        `${item.range_m.toFixed(2)} m` : '读数无效';
    };
    setRows($('sensorStatus'), [
      ['左目',!data.vision_enabled?'当前模式未接入':data.images.left?.fresh?'正常':'断流'],
      ['右目',!data.vision_enabled?'当前模式未接入':data.images.right?.fresh?'正常':'断流'],
      ['深度',!data.vision_enabled?'当前模式未接入':data.images.depth?.fresh?'正常':'断流'],
      ['YOLO',!data.vision_enabled?'当前模式未接入':data.detection_fresh?'正常':'数据过期'],
      ['IMU',data.imu_fresh?'正常':'数据过期'],
      ...sonarNames.map(([name,label]) => [label,sonarState(name)]),
      ['避障',data.safety?.fresh?'有新数据':'数据过期']]);
    setRows($('systemStatus'), [['节点在线',Math.round(data.uptime_seconds)+' 秒'],
      ['浏览器网络',$('connection').textContent],
      ['CPU 负载',String(data.system?.cpu_load ?? '未知')],
      ['内存',data.system?.memory_percent == null ? '未知' : data.system.memory_percent+'%'],
      ['磁盘',data.system?.disk_percent == null ? '未知' : data.system.disk_percent+'%'],
      ['温度',data.system?.temperature_c == null ? '未知' : data.system.temperature_c+'°C'],
      ['ROS 节点',data.ros_nodes?.length ? data.ros_nodes.join(' · ') : '未发现']]);
    setRows($('modelStatus'), [['本机 Qwen',data.local_model_ready?'在线':'不可用'],
      ['外部 API',data.external_model_configured?'已配置':'未配置']]);
    $('modelSource').querySelector('[value="external"]').disabled =
      !data.external_model_configured;
    const target = $('target'), follow = $('followTarget');
    const previous = target.value, previousFollow = follow.value;
    target.replaceChildren(new Option('请选择',''));
    follow.replaceChildren(new Option('请选择人员',''));
    const classFilter = $('classFilter'), oldClass = classFilter.value;
    classFilter.replaceChildren(new Option('全部',''));
    for (const name of new Set(data.detections.map(item => item.name)))
      classFilter.add(new Option(name,name));
    classFilter.value = oldClass;
    for (const item of data.detections.filter(item =>
      item.confidence >= Number($('confidence').value) &&
      (!classFilter.value || item.name === classFilter.value))) {
      const label = `${item.name} · ${Math.round(item.confidence*100)}%`;
      target.add(new Option(label,item.id));
      if (/person|人/i.test(item.name)) follow.add(new Option(label,item.id));
    }
    target.value = previous; follow.value = previousFollow;
    renderMap();
    drawDetections();
  };
  function renderNavigation(data) {
    const nav = data.navigation || {phase:'idle', message:'控制台尚未提供导航反馈'};
    const titles = {idle:'尚未开始导航',planning:'正在规划路径',ready:'路径已规划',
      starting:'正在启动导航',navigating:'正在前往目标',succeeded:'已到达目标',
      canceled:'导航已取消',failed:'导航失败'};
    const active = nav.phase === 'starting' || nav.phase === 'navigating';
    let title = titles[nav.phase] || '导航状态未知';
    const rows = [];
    if (nav.goal && Number.isFinite(nav.goal.x) && Number.isFinite(nav.goal.y))
      rows.push(['目标位置',`${nav.goal.x.toFixed(2)}, ${nav.goal.y.toFixed(2)} m`]);
    if (nav.distance_remaining_m != null)
      rows.push(['剩余距离',`${nav.distance_remaining_m.toFixed(2)} m`]);
    if (nav.heading_error_deg != null)
      rows.push(['剩余朝向',`${nav.heading_error_deg.toFixed(1)}°`]);
    if (nav.eta_seconds != null)
      rows.push(['预计还需',`${Math.ceil(nav.eta_seconds)} 秒`]);
    if (nav.elapsed_seconds != null)
      rows.push(['已导航',`${Math.floor(nav.elapsed_seconds)} 秒`]);
    if (nav.recoveries) rows.push(['恢复尝试',`${nav.recoveries} 次`]);
    if (nav.started_at) rows.push(['开始时间',new Date(nav.started_at*1000).toLocaleTimeString('zh-CN',{hour12:false})]);
    if (nav.finished_at) rows.push(['结束时间',new Date(nav.finished_at*1000).toLocaleTimeString('zh-CN',{hour12:false})]);
    if (nav.error_code) rows.push(['Nav2 错误码',String(nav.error_code)]);
    if (active && nav.feedback_age_seconds != null && nav.feedback_age_seconds > 3)
      rows.push(['反馈',`${nav.feedback_age_seconds.toFixed(1)} 秒未收到新反馈`]);
    else if (nav.phase === 'starting' && nav.start_age_seconds > 3)
      rows.push(['反馈',`目标已提交 ${nav.start_age_seconds.toFixed(1)} 秒，仍未收到 Nav2 导航反馈`]);
    if (active && data.task_paused) title = '导航已暂停';
    if (active && nav.distance_remaining_m != null && nav.distance_remaining_m <= 0.1 &&
        nav.heading_error_deg != null && nav.heading_error_deg > 14.3)
      title = '已接近目标，等待朝向调整';
    if (active && nav.appears_stalled) {
      title = '导航仍在执行，进度停滞';
      rows.push(['进度提醒',`约 ${Math.floor(nav.no_progress_seconds)} 秒内位置和朝向均未接近目标；可能正在等待转向、避障或重新规划`]);
    }
    if (active && data.safety?.fresh) {
      try {
        const safety = JSON.parse(data.safety.message);
        const labels = {
          FRONT_WALL:'前方有障碍',REAR_WALL:'后方有障碍',
          LEFT_WALL:'左侧有障碍',RIGHT_WALL:'右侧有障碍',
          LEFT_SIDE_WALL:'左侧通道有障碍',RIGHT_SIDE_WALL:'右侧通道有障碍',
          FRONT_APPROACH:'正在接近前方障碍',
          LEFT_SIDE_APPROACH:'正在接近左侧障碍',
          RIGHT_SIDE_APPROACH:'正在接近右侧障碍',
          FRONT_RANGE_STALE:'前方测距过期',REAR_RANGE_STALE:'后方测距过期',
          LEFT_RANGE_STALE:'左侧测距过期',RIGHT_RANGE_STALE:'右侧测距过期',
          LEFT_TURN_RANGE_STALE:'左侧测距过期',
          RIGHT_TURN_RANGE_STALE:'右侧测距过期',
          STEP_BLOCKED:'前方有台阶',DROPOFF_BLOCKED:'前方有落差',
          SLOPE_BLOCKED:'坡度过大',SLIPPING:'检测到打滑',
          ESCAPE_REVERSE:'正在执行脱困倒车',
          ESCAPE_REVERSE_START:'开始脱困倒车',
          ESCAPE_REAR_RANGE_STALE:'脱困时后方测距过期',
          REAR_NOT_CLEAR:'脱困时后方空间不足',
          MOTION_COMPONENT_BLOCKED_STOP:'安全层要求停车'};
        const warnings = (safety.warnings || []).filter(item =>
          !item.startsWith('MOTION_COMPONENT_BLOCKED_STOP')).map(item => {
          const [code, value] = item.split(':');
          const side = code.startsWith('RIGHT_') ? 'right' :
            code.startsWith('LEFT_') ? 'left' : null;
          if (code.endsWith('RANGE_STALE') && side &&
              data.ultrasonic?.[side]?.received &&
              !data.ultrasonic[side].fresh) {
            return `${side === 'right' ? '右侧' : '左侧'}测距无效`;
          }
          return (labels[code] || code) + (value ? `（${value}）` : '');
        });
        if (warnings.length) {
          rows.push(['安全限制',warnings.join('；')]);
          if (!nav.appears_stalled) title = '导航检测到安全限制';
        }
      } catch (_) { /* 状态仍显示 Nav2 原始反馈。 */ }
    }
    $('navigationPhase').textContent = title;
    $('navigationMessage').textContent = nav.message || '';
    $('navigationPhase').parentElement.dataset.phase = nav.phase;
    setRows($('navigationDetails'), rows);
  }
  function drawDetections() {
    const canvas = $('detectionsCanvas'), image = $('camera');
    const bounds = image.getBoundingClientRect(), parent = image.parentElement.getBoundingClientRect();
    canvas.width = parent.width; canvas.height = parent.height;
    if (!$('showYolo').checked || camera !== 'right' || !state?.image_size) return;
    const context = canvas.getContext('2d');
    context.lineWidth = 2; context.strokeStyle = '#57edaa'; context.fillStyle = '#57edaa';
    const scale = Math.min(bounds.width/state.image_size[0],bounds.height/state.image_size[1]);
    const offsetX = bounds.left-parent.left, offsetY = bounds.top-parent.top;
    for (const item of state.detections.filter(item =>
      item.confidence >= Number($('confidence').value) &&
      (!$('classFilter').value || item.name === $('classFilter').value))) {
      const [x1,y1,x2,y2] = item.bbox;
      context.strokeRect(offsetX+x1*scale,offsetY+y1*scale,(x2-x1)*scale,(y2-y1)*scale);
      context.fillText(item.name,offsetX+x1*scale,offsetY+y1*scale-3);
    }
  }
  $('camera').addEventListener('click', event => {
    if (camera !== 'right' || !state?.image_size) return;
    const bounds = event.currentTarget.getBoundingClientRect();
    const x = (event.clientX-bounds.left)/bounds.width*state.image_size[0];
    const y = (event.clientY-bounds.top)/bounds.height*state.image_size[1];
    const match = state.detections.find(item => x >= item.bbox[0] &&
      x <= item.bbox[2] && y >= item.bbox[1] && y <= item.bbox[3]);
    if (match) { $('target').value = match.id; notice(`已选择 ${match.name}`); }
  });
  $('classFilter').addEventListener('change', () => state && renderStatus(state));
  $('confidence').addEventListener('input', () => {
    $('confidenceValue').textContent = Math.round(Number($('confidence').value)*100)+'%';
    if (state) renderStatus(state);
  });
  async function poll() {
    if (pollBusy) return;
    pollBusy = true;
    try {
      try {
        renderStatus(await api('status'));
        legacyBackend = typeof state.control.connected !== 'boolean' || !state.localization;
        if (legacyBackend) {
          statusConnected = false;
          notice('整车仍运行旧版控制台；请重启 robot.launch.py 后刷新页面。', true);
        } else if (!statusConnected) {
          notice(state.control.stopped ?
            '已连接机器人；软件停止已锁定，请点击解除软件停止。' :
            '已连接机器人；可以直接操作。');
          statusConnected = true;
        }
      } catch (error) {
        statusConnected = false;
        $('connection').textContent = '连接断开'; notice(error.message,true);
        $('hardwareAlert').textContent = '控制台状态无法获取，无法确认底盘是否可用。';
        $('hardwareAlert').hidden = false;
        releaseDrive(); return;
      }
      if (state?.map_ready && (!map || Date.now() - lastMapFetch >= 2000)) {
        lastMapFetch = Date.now();
        try { map = await api('map'); renderMap(); }
        catch (error) {
          if (!map) $('mapEmpty').hidden = false;
          notice(error.message,true);
        }
      } else if (!map) {
        $('mapEmpty').textContent = state?.map_source || '等待地图';
        $('mapEmpty').hidden = false;
      }
    } finally {
      pollBusy = false;
    }
  }
  async function pollNavigation() {
    if (navPollBusy) return;
    navPollBusy = true;
    try {
      const response = await fetch('/robot/api/navigation-status', {cache:'no-store'});
      if (!response.ok) throw new Error('导航状态接口暂不可用');
      renderNavigation({...state, navigation: await response.json()});
    } catch (_) {
      if (!state?.navigation) {
        $('navigationPhase').textContent = '导航状态暂不可用';
        $('navigationMessage').textContent = '请检查控制台是否已更新并重新启动。';
      }
    } finally {
      navPollBusy = false;
    }
  }
  async function refreshFrame() {
    if ($('home').hidden) return;
    if (frameBusy) return;
    frameBusy = true;
    const image = $('camera');
    try {
      const response = await fetch(`/robot/api/frame/${camera}?t=${Date.now()}`,
        {cache:'no-store'});
      if (!response.ok) throw new Error('当前画面暂不可用');
      const nextUrl = URL.createObjectURL(await response.blob());
      await new Promise((resolve, reject) => {
        image.onload = resolve;
        image.onerror = reject;
        image.src = nextUrl;
      });
      if (frameUrl) URL.revokeObjectURL(frameUrl);
      frameUrl = nextUrl;
      $('cameraEmpty').hidden = true;
      drawDetections();
    } catch (_) {
      if (!image.naturalWidth) $('cameraEmpty').hidden = false;
    } finally {
      frameBusy = false;
    }
  }
  async function pollLogs() {
    try {
      const payload = await api(`logs?after=${lastLog}&level=${$('logLevel').value}`);
      for (const entry of payload.items) {
        lastLog = Math.max(lastLog,entry.id);
        const time = new Date(entry.time*1000).toLocaleTimeString('zh-CN',{hour12:false});
        const level = entry.level >= 40 ? 'ERROR' : entry.level >= 30 ? 'WARN' : 'INFO';
        logLines.push(`${time}  ${level.padEnd(5)}  ${entry.node}  ${entry.message}`);
      }
      logLines = logLines.slice(-600);
      if (!logPaused) { $('terminal').textContent = logLines.join('\n'); $('terminal').scrollTop = $('terminal').scrollHeight; }
    } catch (_) {}
  }
  $('pauseLogs').addEventListener('click', () => {
    logPaused = !logPaused; $('pauseLogs').textContent = logPaused?'继续滚动':'暂停滚动';
  });
  $('clearLogs').addEventListener('click', () => { logLines=[]; $('terminal').textContent=''; });
  poll();
  pollNavigation();
  pollLogs(); refreshFrame();
  setInterval(poll, 1000); setInterval(pollNavigation, 500);
  setInterval(pollLogs, 700); setInterval(refreshFrame, 200);
})();
