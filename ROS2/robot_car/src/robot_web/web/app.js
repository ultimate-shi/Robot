/* 作用：控制台导航、支持多方向组合的四轮转向人工控制及中文状态显示，腿部角度/电流、轮速/电流同行展示；使用方法：由 /robot/ 加载，地图支持滚轮缩放、拖动平移与点击选点。 */
(() => {
  const $ = id => document.getElementById(id);
  const clientId = `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
  let state = null;
  let map = null;
  let selectedGoal = null;
  let preview = null;
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
      signal: ['status','drive','joint-motion','joint-step','joints','joints/zero','body-balance'].includes(path) ? AbortSignal.timeout(2000) : undefined,
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
    $('sideStop').hidden = false;
    window.dispatchEvent(new Event('robot-route'));
    document.querySelectorAll('[data-page]').forEach(link =>
      link.classList.toggle('active', link.dataset.page === page));
  };
  document.querySelectorAll('[data-page]').forEach(link => link.addEventListener('click', event => {
    event.preventDefault(); history.pushState({}, '', link.href); route();
  }));
  window.addEventListener('popstate', route); route();
  const stop = async () => {
    releaseAll();
    const result = await guarded(() => api('stop', {}));
    if (result) notice('软件停止已锁定；请在首页明确解除。');
  };
  $('sideStop').addEventListener('click', stop);
  $('resetStop').addEventListener('click', () => guarded(async () => {
    await api('reset-stop', {}); notice('软件停止已解除。');
  }));
  function createDriveSender() {
    let socket = null;
    let retryAt = 0;
    let lastAck = null;
    let timeoutMs = 0;
    const now = () => typeof performance === 'undefined' ? Date.now() : performance.now();
    const send = (linear, angular) => {
      const body = {linear, angular, client_id:clientId, sequence:++driveSequence};
      if (!socket && typeof WebSocket !== 'undefined' && Date.now() >= retryAt) {
        socket = new WebSocket(`${location.protocol === 'https:'?'wss:':'ws:'}//${location.host}/robot/api/drive-stream`);
        const current = socket;
        current.onmessage = event => {
          try {
            const result = JSON.parse(event.data);
            if (result.ok && result.accepted && Number.isFinite(result.timeout_seconds) && result.timeout_seconds > 0) {
              lastAck=now(); timeoutMs=result.timeout_seconds*1000;
            }
            if (result.ok === false) {notice(result.detail || '驾驶输入未接受',true); releaseAll();}
          } catch (_) { /* 非 JSON 帧不改变驾驶意图。 */ }
        };
        current.onclose = () => {if (socket === current) {socket=null;lastAck=null;retryAt=Date.now()+1000;}};
        current.onerror = () => current.close();
        // 建连时不缓存/重放非零命令；只有仍按住时的下一次刷新才发送。
      }
      if (socket?.readyState === 1) {
        if (!socket.bufferedAmount) {
          socket.send(JSON.stringify(body));
          return Promise.resolve({ok:true});
        }
        if (linear || angular) return Promise.resolve({ok:true});
        // 松键时不能排在旧帧后面：关流令服务端立即释放，并以 HTTP 补发零速。
        socket.close();
      }
      return api('drive',body);
    };
    send.isHealthy = () => socket?.readyState === 1 && lastAck !== null && now()-lastAck <= timeoutMs;
    return send;
  }
  const sendDrive = createDriveSender();
  const velocity = () => ({linear: Number($('linearSpeed').value), angular: Number($('angularSpeed').value)});
  // 每组独立记录按键，允许驾驶、姿态和头部同时操作。
  const heldKeys = new Set();
  // 每根手指独立保存方向，与键盘合并后只发布一条线/角速度命令。
  const drivePointers = new Map();
  document.querySelectorAll('[data-drive]').forEach(button => {
    const key = button.dataset.drive;
    button.addEventListener('pointerdown', event => {
      event.preventDefault();
      if (key === 'stop') { releaseAll(); return; }
      if (state?.control?.mode !== 'manual' || state?.control?.stopped) return;
      button.setPointerCapture?.(event.pointerId);
      drivePointers.set(event.pointerId, key);
      updateKeyboard();
    });
    for (const name of ['pointerup','pointercancel','pointerleave','lostpointercapture'])
      button.addEventListener(name, event => {
        if (drivePointers.delete(event.pointerId)) updateKeyboard();
      });
  });
  let motionSequence = 0;
  let keyboardDriving = false;
  let jointMoving = false;
  let motionBusy = false;
  let motionDirty = false;
  let forceJointHold = false;
  const legKeys=new Set(['w','a','s','d','q','e']);
  const oppositeLeg={w:'s',s:'w',a:'d',d:'a',q:'e',e:'q'};
  const nextLegRepeat=new Map();
  let legStepEpoch=0;
  let legStepQueue=Promise.resolve();
  const sendLegStep=key => {
    const epoch=legStepEpoch;
    legStepQueue=legStepQueue.then(async () => {
      if (epoch !== legStepEpoch) return;
      if (state?.actuators?.leg_reason) { notice(state.actuators.leg_reason,true); return; }
      const result=await guarded(() => api('joint-step',{action:key,client_id:clientId,
        sequence:++motionSequence,generation:state?.actuators?.generation ?? 0}));
      if (result === null) releaseAll();
    });
  };
  const pressLeg=key => {
    if (heldKeys.has(key)) return;
    heldKeys.add(key);
    nextLegRepeat.set(key,Date.now()+1000*(state?.actuators?.leg_step_repeat_seconds ?? 0.3));
    if (!heldKeys.has(oppositeLeg[key])) sendLegStep(key);
  };
  const knownKeys = new Set(['arrowup','arrowdown','arrowleft','arrowright',
    'w','a','s','d','q','e','j','l','i','k',' ']);
  const textFocus = element => element && (
    ['TEXTAREA','SELECT','INPUT'].includes(element.tagName) || element.isContentEditable);
  const axis = (positive, negative) => Number(heldKeys.has(positive))-Number(heldKeys.has(negative));
  const intentValues = () => {
    const data = {pitch:0,roll:0,lift:0,
      yaw:axis('j','l'),head_pitch:axis('i','k')};
    if (state?.actuators?.leg_reason) data.pitch=data.roll=data.lift=0;
    if (state?.actuators?.head_reason) data.yaw=data.head_pitch=0;
    return data;
  };
  const sendJointIntent = async (forceHold=false) => {
    if (forceHold) forceJointHold=true;
    if (motionBusy) { motionDirty = true; return; }
    const values = intentValues();
    const moving = Object.values(values).some(Boolean);
    if (!moving && !jointMoving && !forceJointHold) return;
    const holding=forceJointHold; forceJointHold=false;
    jointMoving = moving;
    motionBusy = true;
    try {
      await api('joint-motion', {client_id:clientId,sequence:++motionSequence,generation:state?.actuators?.generation ?? 0,hold:holding,...values});
    } catch (error) {
      heldKeys.clear(); notice(error.message, true);
    } finally {
      motionBusy = false;
      if (motionDirty) { motionDirty=false; sendJointIntent(); }
    }
  };
  const updateKeyboard = (forceHold=false) => {
    const directions = new Set(drivePointers.values());
    const active = (key, direction) => heldKeys.has(key) || directions.has(direction);
    const linear = (Number(active('arrowup','forward'))-Number(active('arrowdown','back')))*velocity().linear;
    const angular = (Number(active('arrowleft','left'))-Number(active('arrowright','right')))*velocity().angular;
    const moving = linear !== 0 || angular !== 0;
    if (moving || keyboardDriving) guarded(() => sendDrive(linear,angular));
    keyboardDriving = moving;
    sendJointIntent(forceHold);
  };
  function releaseAll() {
    legStepEpoch++; nextLegRepeat.clear();
    heldKeys.clear();
    drivePointers.clear();
    updateKeyboard(true);
  }
  window.addEventListener('keydown', event => {
    if (!$('keyboardEnabled').checked || textFocus(document.activeElement) ||
        document.hidden || $('home').hidden) return;
    const key = event.key.toLowerCase();
    if (!knownKeys.has(key)) return;
    // 首页驾驶键即使被停止锁拦截也不滚动页面，并明确说明当前不能驾驶的原因。
    event.preventDefault();
    if (state?.control?.stopped || state?.control?.mode !== 'manual') {
      if (!event.repeat) notice(state?.control?.stopped ?
        '软件停止仍锁定，请先点击“解除软件停止”。' : '请先切换到手动控制。', true);
      return;
    }
    if (key === ' ') { releaseAll(); return; }
    if (legKeys.has(key)) { if (!event.repeat) pressLeg(key); return; }
    if (!event.repeat) { heldKeys.add(key); updateKeyboard(); }
  });
  window.addEventListener('keyup', event => {
    const key = event.key.toLowerCase();
    nextLegRepeat.delete(key);
    if (legKeys.has(key)) legStepEpoch++;
    if (heldKeys.delete(key)) { event.preventDefault(); updateKeyboard(); }
  });
  window.addEventListener('blur', releaseAll);
  window.addEventListener('robot-route', releaseAll);
  document.addEventListener('focusin', event => { if (textFocus(event.target)) releaseAll(); });
  document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });
  $('keyboardEnabled').addEventListener('change', event => {
    releaseAll();
    // 勾选后交回驾驶焦点，避免复选框一直被当成表单输入而吞掉方向键。
    event?.currentTarget?.blur?.();
  });
  window.addEventListener('pagehide', () => {
    heldKeys.clear(); drivePointers.clear();
    for (const [path, data] of [
      ['drive',{linear:0,angular:0,client_id:clientId,sequence:++driveSequence}],
      ['joint-motion',{client_id:clientId,sequence:++motionSequence,hold:true}]
    ]) navigator.sendBeacon('/robot/api/'+path,new Blob([JSON.stringify(data)],{type:'application/json'}));
  });
  setInterval(() => {
    const now=Date.now();
    for (const key of heldKeys) if (legKeys.has(key) && !heldKeys.has(oppositeLeg[key]) && now >= (nextLegRepeat.get(key) ?? 0)) {
      nextLegRepeat.set(key,now+1000*(state?.actuators?.leg_step_repeat_seconds ?? 0.3));
      sendLegStep(key);
    }
    if (heldKeys.size || drivePointers.size || keyboardDriving || jointMoving) updateKeyboard();
  }, 100);
  document.querySelectorAll('[data-joint-key]').forEach(button => {
    const key = button.dataset.jointKey;
    button.addEventListener('pointerdown', event => {
      event.preventDefault(); button.setPointerCapture?.(event.pointerId);
      if (legKeys.has(key)) pressLeg(key);
      else { heldKeys.add(key); updateKeyboard(); }
    });
    for (const name of ['pointerup','pointercancel','lostpointercapture'])
      button.addEventListener(name, () => {
        if (heldKeys.delete(key) && legKeys.has(key)) legStepEpoch++;
        nextLegRepeat.delete(key); updateKeyboard();
      });
  });
  for (const [id,group] of [['legsZero','leg'],['headZero','head']])
    $(id).addEventListener('click', () => guarded(async () => {
      // 回零与清除本组输入由后端一次完成，不先发送异步保持命令。
      for (const key of group === 'leg'?legKeys:['j','l','i','k']) heldKeys.delete(key);
      if (group === 'leg') { legStepEpoch++; nextLegRepeat.clear(); }
      await api('joints/zero',{group,client_id:clientId,sequence:++motionSequence,
        generation:state?.actuators?.generation ?? 0});
      notice(group === 'leg'?'四条大腿已一次提交 0° 目标。':'头部两个关节已一次提交 0° 目标。');
    }));
  // 表单草稿持续保留到成功提交，不能仅依赖焦点（点击标签或触屏可不聚焦）。
  let balanceFormDirty = false;
  for (const id of ['balanceEnabled','balanceLevel','balanceRoll','balancePitch']) {
    for (const event of ['input','change'])
      $(id).addEventListener(event, () => { balanceFormDirty = true; });
  }
  function syncBalanceForm(b) {
    if (balanceFormDirty || document.activeElement?.closest('.balance-box') || !b.fresh || b.pending) return;
    $('balanceEnabled').checked=b.enabled;
    $('balanceLevel').value=b.level;
    $('balanceRoll').value=b.target_roll_deg;
    $('balancePitch').value=b.target_pitch_deg;
    for (const id of ['balanceRoll','balancePitch']) $(id).disabled=b.level !== 'mid';
  }
  const applyBalance = enabled => guarded(async () => {
    releaseAll();
    const level = $('balanceLevel').value;
    await api('body-balance',{enabled,level,
      target_roll_deg: level === 'mid' ? Number($('balanceRoll').value) : 0,
      target_pitch_deg: level === 'mid' ? Number($('balancePitch').value) : 0});
    balanceFormDirty = false;
    notice('平衡设置已提交，等待节点确认。');
  });
  $('applyBalance').addEventListener('click', () => applyBalance($('balanceEnabled').checked));
  $('resumeHeadLock').addEventListener('click', () => guarded(async () => {
    await api('head-lock/resume',{}); notice('已恢复建图/定位头部归中。');
  }));
  $('disableBalance').addEventListener('click', () => applyBalance(false));
  $('balanceLevel').addEventListener('change', () => {
    for (const id of ['balanceRoll','balancePitch']) $(id).disabled = $('balanceLevel').value !== 'mid';
  });
  const legLabels = ['左前腿','右前腿','左后腿','右后腿'];
  const headLabels = ['水平旋转','俯仰'];
  for (const [group,labels,container] of [['leg',legLabels,'legSliders'],['head',headLabels,'headSliders']]) {
    labels.forEach((label,index) => {
      const card=document.createElement('label'); card.className='joint-slider '+group;
      const title=document.createElement('strong'); title.textContent=label;
      const input=document.createElement('input'); input.type='range'; input.step='0.1';
      input.id=`joint-${group}-${index}`; input.disabled=true;
      const angleNumber=group === 'leg'?document.createElement('input'):null;
      if (angleNumber) {
        angleNumber.type='number'; angleNumber.step=input.step='0.01'; angleNumber.disabled=true;
        angleNumber.id=`joint-angle-${index}`; angleNumber.setAttribute('aria-label',label+'板端目标角度');
      }
      input.setAttribute('aria-label',label+(group === 'leg' ? '板端角度' : '角度'));
      const output=document.createElement('output'); output.id=`joint-value-${group}-${index}`;
      output.textContent='等待反馈'; card.append(title,input);
      if (angleNumber) card.append(angleNumber);
      card.append(output); $(container).append(card);
      input.addEventListener('input', () => {
        if (angleNumber) angleNumber.value=input.value;
        output.textContent=`目标 ${input.value}°`;
      });
      const submitAngle=() => guarded(async () => {
        const items=state?.actuators?.joints?.filter(item => item.group === group) || [];
        const values=items.map(item => group === 'leg' ? (item.hardware_target_deg ?? item.hardware_actual_deg) : (item.target_deg ?? item.actual_deg));
        if (values.length !== labels.length || values.some(v => !Number.isFinite(v)))
          throw new Error('关节反馈不可用');
        values[index]=Number(input.value);
        await api('joints',{group,values,unit:group === 'leg' ? 'hardware_deg' : 'deg'});
      });
      input.addEventListener('change',submitAngle);
      if (angleNumber) angleNumber.addEventListener('change',() => {
        const value=Number(angleNumber.value);
        if (!String(angleNumber.value).trim() || !Number.isFinite(value) || value < Number(input.min) || value > Number(input.max)) {
          notice('角度超出限位或数值无效',true); return;
        }
        input.value=angleNumber.value; submitAngle();
      });
    });
  }
  function renderActuators(data) {
    const a=data.actuators;
    $('jointManualControls').hidden=data.control.mode !== 'manual';
    if (!a) { $('legReason').textContent=$('headReason').textContent='关节接口不可用，请更新并重启控制台'; return; }
    $('legReason').textContent=a.leg_reason || '手动可用；允许同时驾驶';
    $('headReason').textContent=a.head_reason || '手动可用';
    $('resumeHeadLock').hidden=!['stopped','paused'].includes(a.head_lock?.state);
    $('resumeHeadLock').disabled=data.control.stopped;
    $('jointLimit').textContent=a.stop_warning || (a.limited ? '已到行程限位；可反向操作，其他有行程的动作仍可使用。' : '松键保持；整体升降保留各腿高度差。');
    const b=a.balance;
    for (const id of ['balanceRoll','balancePitch']) {
      $(id).min=-a.balance_max_setpoint_deg; $(id).max=a.balance_max_setpoint_deg;
    }
    const balanceText=!b.fresh ? '状态未知或过期' :
      `${b.enabled?'已开启':'已关闭'} · ${{low:'低档固定高度',mid:'中档自动调平',high:'高档固定高度'}[b.level] || b.level} · ${balanceState(b.state)}`;
    $('sideBalance').textContent='自动平衡：'+balanceText;
    $('balanceState').textContent=balanceText+(b.pending?' · 设置确认中':'')+
      (b.error?' · '+b.error:'')+(b.overload_active?' · 过流保护':'')+
      (b.fresh?`；实际横滚 ${fmt(b.actual_roll_deg)}°，俯仰 ${fmt(b.actual_pitch_deg)}°；目标 ${fmt(b.target_roll_deg)}° / ${fmt(b.target_pitch_deg)}°`:'');
    $('applyBalance').disabled=b.pending || data.control.stopped;
    $('disableBalance').disabled=b.pending;
    syncBalanceForm(b);
    document.querySelectorAll('[data-joint-key]').forEach(button => {
      button.disabled=Boolean('wasdqe'.includes(button.dataset.jointKey)?a.leg_reason:a.head_reason);
    });
    $('legsZero').disabled=Boolean(a.leg_reason); $('headZero').disabled=Boolean(a.head_reason);
    const rows=[];
    for (const item of a.joints) {
      const {group,index}=item;
      const label=(group === 'leg'?legLabels:headLabels)[index];
      const input=$(`joint-${group}-${index}`), output=$(`joint-value-${group}-${index}`);
      input.disabled=Boolean(item.reason);
      const angleNumber=group === 'leg'?$(`joint-angle-${index}`):null;
      if (angleNumber) angleNumber.disabled=input.disabled;
      input.min=group === 'leg'?item.hardware_min_deg:item.min_deg;
      input.max=group === 'leg'?item.hardware_max_deg:item.max_deg;
      if (group === 'leg') input.style.direction=item.extension_direction < 0?'ltr':'rtl';
      if (angleNumber) { angleNumber.min=input.min; angleNumber.max=input.max; }
      if (document.activeElement !== input && document.activeElement !== angleNumber) {
        input.value=(group === 'leg'?item.hardware_actual_deg:item.actual_deg) ?? 0;
        if (angleNumber) angleNumber.value=Number(input.value).toFixed(2);
        output.textContent=item.fresh ? `实际 ${fmt(group === 'leg'?item.hardware_actual_deg:item.actual_deg)}°；目标 ${fmt(group === 'leg'?item.hardware_target_deg:item.target_deg)}°` : '反馈缺失或过期';
      }
      const actual=group === 'leg'?item.hardware_actual_deg:item.actual_deg;
      const angle=item.fresh?`角度 ${fmt(actual)}°`:'角度反馈缺失或过期';
      rows.push([label,angle+(group === 'leg'?`；电流 ${data.hardware?.fresh?fmt(data.hardware.leg_current_ma?.[index])+' mA':'无有效反馈'}`:'')]);
    }
    rows.push(['自动平衡',balanceText],['平衡目标',`${fmt(b.target_roll_deg)}° / ${fmt(b.target_pitch_deg)}°`],
      ['实际姿态',b.fresh?`${fmt(b.actual_roll_deg)}° / ${fmt(b.actual_pitch_deg)}°`:'无有效反馈'],
      ['过流保护',b.fresh?(b.overload_active?'降档保护中':'未触发'):'未知']);
    // 电流数组沿用硬件 FL/FR/RL/RR 顺序，不能按关节消息到达顺序配对。
    for (const [index,position] of ['front_left','front_right','rear_left','rear_right'].entries()) {
      const label=legLabels[index].replace('腿','');
      const steer=a.other_joints?.[`${position}_steer_joint`];
      const wheel=a.other_joints?.[`${position}_wheel_joint`];
      rows.push([label+'转向',steer?.fresh?`角度 ${fmt(steer.actual_deg)}°`:'角度反馈缺失或过期']);
      rows.push([label+'轮',`${wheel?.fresh?'速度 '+fmt(wheel.velocity_rad_s)+' rad/s':'速度反馈缺失或过期'}；电流 ${data.hardware?.fresh?fmt(data.hardware.wheel_current_ma?.[index])+' mA':'无有效反馈'}`]);
    }
    setRows($('jointStatus'),rows);
  }
  function fmt(value) { return Number.isFinite(value)?value.toFixed(1):'未知'; }
  function balanceState(value) {
    return ({disabled:'未启用',stopped:'停止锁定',hardware_fault:'硬件失能，平衡已关闭',cycle_late:'控制周期延迟，已暂停追加调节',balanced:'已调平',balancing:'调平中',
      waiting_imu:'等待 IMU',waiting_joint_state:'等待腿反馈',joint_stale:'腿反馈过期',imu_stale:'IMU 过期',
      static_low:'低档固定高度',static_high:'高档固定高度',restoring_mid:'恢复中档',
      overload_low:'过流降至低档',overload_recovered_low:'过流恢复，保持低档'})[value] || value;
  }
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
    releaseAll();
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
    const result = await api('cancel', {}); preview = null;
    notice(result.canceled ? 'Nav2 已确认导航结束。' : '已切断导航速度，取消结果请查看地图下方导航状态。');
  }));
  $('saveMap').addEventListener('click', () => guarded(async () => {
    const button = $('saveMap');
    button.disabled = true;
    button.textContent = '正在保存…';
    try {
      notice('正在保存地图和定位数据库；建图退出后将尝试恢复，可能需要数分钟，请保持停车并等待结果…');
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
    renderActuators(data);
    if (data.control.stopped || data.control.mode !== 'manual') releaseAll();
    $('robotMode').textContent = data.mode_source === 'real' ? '当前：实机' : '当前：仿真';
    $('overall').textContent = data.control.stopped ? '紧急停止已锁定' :
      data.mode_source === 'real' && (!data.hardware?.fresh || !data.hardware.outputs_enabled ||
      !data.hardware.connected || !data.hardware.watchdog_ok || data.hardware.fault) ? '硬件异常或反馈不可用' :
      !data.controllers?.available ? '控制器状态不可用' : '状态监测中，详见各设备反馈';
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
    const astraCamera = data.camera_source === 'astra';
    document.querySelector('[data-camera="left"]').hidden = astraCamera;
    document.querySelector('[data-camera="right"]').textContent = astraCamera ? '彩色' : '右目';
    if (astraCamera && camera === 'left') camera = 'right';
    const localization = data.localization || {required:data.mode_source === 'real',ready:false,
      state:'unavailable',message:'当前控制台尚未提供定位接口，请重启整车后刷新页面'};
    $('localizationPanel').hidden = !localization.required;
    const phases = {initializing:'视觉地图加载中',waiting:'等待定位',verifying:'正在确认定位',localized:'定位成功',tracking:'里程计跟踪中',
      failed:'定位超时',lost:'定位丢失',unavailable:'定位服务不可用'};
    $('localizationPhase').textContent = phases[localization.state] || localization.message;
    $('localizationMessage').textContent = localization.message +
      (localization.matches != null ? `（有效匹配 ${localization.matches}/${localization.required_matches}）` : '') +
      (localization.state === 'tracking' ? `；距上次地图确认 ${localization.match_age_seconds} 秒，累计 ${localization.tracking_distance_m} 米 / ${localization.tracking_angle_deg}°` : '') +
      (localization.source ? `；来源：${({visual:'视觉',amcl:'深度轮廓',fused:'视觉＋轮廓',tracking:'里程计跟踪',waiting:'等待定位'})[localization.source] || localization.source}` : '') +
      (localization.scan_score != null ? `；轮廓吻合度 ${(localization.scan_score*100).toFixed(0)}%` : '') +
      (localization.sources ? Object.entries(localization.sources).filter(([,value]) => value.rejection)
        .map(([key,value]) => `；${key === 'visual' ? '视觉' : '轮廓'}：${value.rejection}`).join('') : '');
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
      ['模式',({manual:'手动',navigation:'导航',explore:'自主探索',follow:'跟随'})[data.control.mode] || '未知'],['控制台接口',data.control.connected?'在线':'不可用'],
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
      !data.hardware[name] ? '断开' : data.hardware.fault ? '已发现；通信/整车故障待检查' :
      !data.hardware.outputs_enabled ? '已发现；输出未就绪' : '已连接，有整车反馈';
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
      return !item ? '等待首个数据' : !item.received ? '数据过期' : item.fresh ?
        `${item.range_m.toFixed(2)} m` : '读数无效';
    };
    const imageState = image => !data.vision_enabled ? '当前模式未接入' : image?.state === 'unconfigured' ? '未配置' : image?.fresh ? '正常' :
      image?.state === 'delayed' ? `短时延迟（${image.age_seconds} 秒）` :
      !image || image.state === 'waiting' ? '等待首帧' : '持续断流';
    setRows($('cameraStatus'),[
      ['设备类型',astraCamera?'Astra RGB-D 深度相机':'双目相机（计算深度）'],
      ...(astraCamera?[
        ['原始彩色流',imageState(data.camera_raw?.color)],
        ['原始深度流',imageState(data.camera_raw?.depth)],
        ['处理后彩色流',imageState(data.images.right)],
        ['处理后深度流',imageState(data.images.depth)]
      ]:[['左目',imageState(data.images.left)],['右目',imageState(data.images.right)],
        ['计算深度',imageState(data.images.depth)]])]);
    setRows($('sensorStatus'), [
      ['YOLO',!data.vision_enabled?'当前模式未接入':data.detection_fresh?'正常':'数据过期'],
      ['IMU',!data.imu_fresh?'数据过期':data.imu_valid?'正常':'数据无效'],
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
      canceling:'正在取消导航',canceled:'导航已取消',failed:'导航失败'};
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
        // 驾驶流有及时确认时，页面查询超时不能伪造一次松键。
        if (!sendDrive.isHealthy?.()) releaseAll();
        else $('hardwareAlert').textContent='状态更新暂时延迟，驾驶连接正常。';
        return;
      }
      if (!keyboardDriving && !jointMoving && state?.map_ready && (!map || Date.now() - lastMapFetch >= 2000)) {
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
    if ($('logs').hidden) return;
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
