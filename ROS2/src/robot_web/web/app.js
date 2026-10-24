/* 作用：控制台页面导航、人工控制、状态和实时数据显示；使用方法：由 /robot/ 自动加载。 */
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
  const renderMap = () => {
    if (!map) return;
    const canvas = $('mapCanvas');
    const width = map.width, height = map.height;
    canvas.width = width; canvas.height = height;
    const context = canvas.getContext('2d');
    const image = context.createImageData(width, height);
    for (let y = 0; y < height; y++) for (let x = 0; x < width; x++) {
      const value = map.data[(height - y - 1) * width + x];
      const shade = value < 0 ? 58 : 235 - Math.round(value * 2.1);
      const index = (y * width + x) * 4;
      image.data[index] = shade; image.data[index+1] = shade;
      image.data[index+2] = shade; image.data[index+3] = 255;
    }
    context.putImageData(image, 0, 0);
    const toPixel = (x, y) => {
      const dx = x - map.origin.x, dy = y - map.origin.y;
      const yaw = 2 * Math.atan2(map.origin.z, map.origin.w);
      return [(Math.cos(yaw)*dx + Math.sin(yaw)*dy) / map.resolution,
        height - (-Math.sin(yaw)*dx + Math.cos(yaw)*dy) / map.resolution];
    };
    if (preview?.path?.length) {
      context.strokeStyle = '#468fee'; context.lineWidth = 3;
      context.beginPath();
      preview.path.forEach((point, index) => {
        const [x,y] = toPixel(point.x, point.y);
        if (index === 0) context.moveTo(x,y); else context.lineTo(x,y);
      });
      context.stroke();
    }
    if (state?.pose?.fresh) {
      const [x,y] = toPixel(state.pose.x,state.pose.y);
      context.fillStyle = '#20c790'; context.beginPath();
      context.arc(x,y,6,0,2*Math.PI); context.fill();
    }
    if (selectedGoal) {
      const [cx,cy] = toPixel(selectedGoal.x,selectedGoal.y);
      context.fillStyle = '#e84158'; context.beginPath(); context.arc(cx,cy,6,0,2*Math.PI); context.fill();
    }
    $('mapEmpty').hidden = true;
  };
  $('mapCanvas').addEventListener('click', event => {
    if (!map) return;
    const bounds = event.currentTarget.getBoundingClientRect();
    const cx = (event.clientX - bounds.left) / bounds.width * map.width;
    const cy = map.height - (event.clientY - bounds.top) / bounds.height * map.height;
    const yaw = 2 * Math.atan2(map.origin.z, map.origin.w);
    selectedGoal = {x: map.origin.x + map.resolution * (Math.cos(yaw)*cx - Math.sin(yaw)*cy),
      y: map.origin.y + map.resolution * (Math.sin(yaw)*cx + Math.cos(yaw)*cy),
      yaw:state?.pose?.fresh ? state.pose.yaw : null};
    $('mapGoal').textContent = `目标：(${selectedGoal.x.toFixed(2)}, ${selectedGoal.y.toFixed(2)}) m`;
    preview = null; renderMap();
  });
  $('centerMap').addEventListener('click', () => $('mapCanvas').scrollIntoView({block:'center'}));
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
    const result = await api('save-map', {}); notice(result.message);
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
    const robot = state?.pose;
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
    showMode(data.control.mode);
    renderNavigation(data);
    setRows($('robotStatus'), [
      ['模式',data.control.mode],['控制台接口',data.control.connected?'在线':'不可用'],
      ['软件停止',data.control.stopped?'已锁定':'已解除'],
      ['地图',data.map_ready?'可用':data.map_source || '等待地图'],
      ['位姿',data.pose?.fresh?`${data.pose.x.toFixed(2)}, ${data.pose.y.toFixed(2)} m`:'不可用'],
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
        legacyBackend = typeof state.control.connected !== 'boolean';
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
