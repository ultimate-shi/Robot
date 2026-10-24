/* 作用：验证地图适配、缩放拖动、点击坐标及标记尺寸；使用方法：node test/test_map_viewport.cjs。 */
const {test} = require('node:test');
const assert = require('node:assert/strict');
const {readFileSync} = require('node:fs');
const vm = require('node:vm');
const source = readFileSync(`${__dirname}/../web/app.js`, 'utf8');

function fixture(yaw = 0) {
  const calls = [];
  const context = new Proxy({}, {get: (_, name) => (...args) => {
    calls.push([name, ...args]);
    if (name === 'createImageData') return {data: new Uint8ClampedArray(args[0] * args[1] * 4)};
  }});
  const bounds = {left: 20, top: 40, width: 600, height: 280};
  const elements = {};
  const $ = id => elements[id] ||= {handlers: {}, captured: null,
    classList: {add() {}, remove() {}},
    setPointerCapture(id) { this.captured = id; },
    hasPointerCapture(id) { return this.captured === id; },
    releasePointerCapture() { this.captured = null; },
    getBoundingClientRect: () => bounds, getContext: () => context,
    addEventListener(name, handler) { this.handlers[name] = handler; }};
  const sandbox = vm.createContext({$, document: {createElement: () => $('raster')},
    window: {devicePixelRatio: 2}, ResizeObserver: class {
      constructor(callback) { this.callback = callback; }
      observe() { sandbox.resize = this.callback; }
    }, notice: message => calls.push(['notice', message]),
    map: {width: 200, height: 100, resolution: 0.05,
      origin: {x: -2, y: -3, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2)},
      data: Array(200 * 100).fill(0)},
    state: {pose: {fresh: true, x: -2 + Math.cos(yaw) - Math.sin(yaw),
      y: -3 + Math.sin(yaw) + Math.cos(yaw), yaw: 0.4}},
    selectedGoal: null, preview: null});
  vm.runInContext(source.slice(source.indexOf('  const mapRaster ='),
    source.indexOf("  $('planMap').addEventListener")), sandbox);
  const run = code => vm.runInContext(code, sandbox);
  run('renderMap()');
  return {sandbox, calls, bounds, $, run};
}
const close = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-8,
  `${actual} 应接近 ${expected}`);
function assertFits(f) {
  const [, , x, y, width, height] = f.calls.filter(c => c[0] === 'drawImage').at(-1);
  assert.ok(x >= 7.99 && y >= 7.99);
  assert.ok(x + width <= f.bounds.width - 7.99);
  assert.ok(y + height <= f.bounds.height - 7.99);
}

test('地图逐步扩大及窗口改变后仍显示全图', () => {
  const f = fixture();
  assertFits(f);
  const previousScale = f.run('mapView.scale');
  f.run('map = {...map, width: 1400, height: 900, data: Array(1400*900).fill(-1)}; renderMap()');
  assertFits(f);
  assert.ok(f.run('mapView.scale') < previousScale);
  f.bounds.width = 320;
  f.sandbox.resize();
  assertFits(f);
  assert.equal(f.$('mapCanvas').width, 640);
});

test('机器人居中后保持当前比例跟随，显示全图可复位', () => {
  const f = fixture();
  const scale = f.run('mapView.scale');
  f.$('centerMap').handlers.click();
  let translation = f.calls.filter(c => c[0] === 'translate').at(-1);
  close(translation[1], 300); close(translation[2], 140);
  close(f.run('mapView.scale'), scale);
  f.run('state.pose.x += 3; renderMap()');
  translation = f.calls.filter(c => c[0] === 'translate').at(-1);
  close(translation[1], 300); close(translation[2], 140);
  close(f.run('mapView.scale'), scale);
  f.$('fitMap').handlers.click();
  assert.equal(f.run('centerOnRobot'), false);
  assertFits(f);
});

test('居中和旋转地图下点击位置仍正确，留白不生成目标', () => {
  for (const yaw of [0, Math.PI / 3]) {
    const f = fixture(yaw);
    f.$('centerMap').handlers.click();
    const click = (x, y) => f.$('mapCanvas').handlers.click({currentTarget: f.$('mapCanvas'),
      clientX: f.bounds.left + x, clientY: f.bounds.top + y});
    click(300, 140);
    close(f.sandbox.selectedGoal.x, f.sandbox.state.pose.x);
    close(f.sandbox.selectedGoal.y, f.sandbox.state.pose.y);
    const goal = f.sandbox.selectedGoal;
    click(0, 0);
    assert.equal(f.sandbox.selectedGoal, goal);
  }
});

test('标记使用固定屏幕半径，地图缩放及高分屏不改变尺寸', () => {
  const f = fixture();
  f.run('selectedGoal = {x: 1, y: 1}; renderMap()');
  f.run('map = {...map, width: 1200, data: Array(1200*100).fill(0)}; renderMap()');
  assert.ok(f.calls.filter(c => c[0] === 'arc').every(c => c[3] === 3));
  assert.ok(f.calls.some(c => c[0] === 'rotate'));
});

test('定位过期时提示且不切换居中模式', () => {
  const f = fixture();
  f.run('state.pose.fresh = false');
  f.$('centerMap').handlers.click();
  assert.equal(f.run('centerOnRobot'), false);
  assert.ok(f.calls.some(c => c[0] === 'notice'));
});

test('只更新位姿时复用栅格缓存，隐藏后恢复可重绘', () => {
  const f = fixture();
  const count = () => f.calls.filter(c => c[0] === 'putImageData').length;
  const before = count();
  f.run('state.pose.x += 1; renderMap()');
  assert.equal(count(), before);
  f.bounds.width = 0;
  f.sandbox.resize();
  f.bounds.width = 600;
  f.sandbox.resize();
  assertFits(f);
});

function emit(f, type, x, y, extra = {}) {
  const event = {type, currentTarget: f.$('mapCanvas'),
    clientX: f.bounds.left + x, clientY: f.bounds.top + y,
    pointerId: 1, button: 0, deltaMode: 0,
    preventDefault() { this.prevented = true; }, ...extra};
  f.$('mapCanvas').handlers[type](event);
  return event;
}

test('滚轮缩放以鼠标位置为锚点，旋转地图中选点仍对应原世界坐标', () => {
  for (const yaw of [0, Math.PI / 3]) {
    const f = fixture(yaw);
    const before = f.run('mapWorldAt(250, 120)');
    const scale = f.run('mapView.scale');
    const event = emit(f, 'wheel', 250, 120, {deltaY: -300});
    assert.equal(event.prevented, true);
    assert.ok(f.run('mapView.scale') > scale);
    const after = f.run('mapWorldAt(250, 120)');
    close(after.x, before.x); close(after.y, before.y);
    emit(f, 'click', 250, 120);
    close(f.sandbox.selectedGoal.x, before.x);
    close(f.sandbox.selectedGoal.y, before.y);
    emit(f, 'wheel', 250, 120, {deltaY: 300});
    close(f.run('mapView.scale'), scale);
  }
});

test('拖动平移后不误选目标，下次普通点击可正常选点', () => {
  const f = fixture(Math.PI / 3);
  const before = f.run('mapWorldAt(250, 120)');
  const scale = f.run('mapView.scale');
  emit(f, 'pointerdown', 250, 120);
  assert.equal(f.$('mapCanvas').captured, 1);
  emit(f, 'pointermove', 280, 140);
  const after = f.run('mapWorldAt(280, 140)');
  close(after.x, before.x); close(after.y, before.y);
  close(f.run('mapView.scale'), scale);
  emit(f, 'pointerup', 280, 140);
  assert.equal(f.$('mapCanvas').captured, null);
  emit(f, 'click', 280, 140);
  assert.equal(f.sandbox.selectedGoal, null);
  emit(f, 'pointerdown', 280, 140);
  emit(f, 'pointermove', 281, 141);
  emit(f, 'pointerup', 280, 140);
  emit(f, 'click', 280, 140);
  close(f.sandbox.selectedGoal.x, before.x);
  close(f.sandbox.selectedGoal.y, before.y);
});

test('取消拖动和捕获丢失时清理状态，忽略其他指针和右键', () => {
  for (const end of ['pointercancel', 'lostpointercapture']) {
    const f = fixture();
    emit(f, 'pointerdown', 200, 120, {button: 2});
    assert.equal(f.run('mapDrag'), null);
    emit(f, 'pointerdown', 200, 120);
    emit(f, 'pointermove', 240, 140, {pointerId: 2});
    assert.equal(f.run('manualMapView'), null);
    emit(f, 'pointermove', 240, 140);
    emit(f, end, 240, 140);
    assert.equal(f.run('mapDrag'), null);
    emit(f, 'click', 240, 140);
    assert.equal(f.sandbox.selectedGoal, null);
  }
});

test('手动视图在地图扩大、原点旋转及窗口变化后保留世界中心与比例', () => {
  const f = fixture();
  emit(f, 'wheel', 250, 120, {deltaY: -300});
  const center = f.run('mapWorldAt(300, 140)');
  const scale = f.run('mapView.scale / mapView.resolution');
  f.run(`map = {...map, width: 1000, height: 400, resolution: 0.1,
    origin: {x: -5, y: -6, z: Math.sin(0.3), w: Math.cos(0.3)},
    data: Array(1000 * 400).fill(0)}; renderMap()`);
  f.bounds.width = 320;
  f.sandbox.resize();
  const after = f.run('mapWorldAt(160, 140)');
  close(after.x, center.x); close(after.y, center.y);
  close(f.run('mapView.scale / mapView.resolution'), scale);
});

test('缩放设有上下限，居中和显示全图均可退出手动视图', () => {
  const f = fixture();
  for (let i = 0; i < 6; i++) emit(f, 'wheel', 300, 140, {deltaY: -10000});
  close(f.run('mapView.scale / mapView.fitScale'), 64);
  for (let i = 0; i < 6; i++) emit(f, 'wheel', 300, 140, {deltaY: 10000, deltaMode: 1});
  close(f.run('mapView.scale / mapView.fitScale'), 0.25);
  const scale = f.run('mapView.scale');
  f.$('centerMap').handlers.click();
  assert.equal(f.run('manualMapView'), null);
  close(f.run('mapView.scale'), scale);
  emit(f, 'wheel', 300, 140, {deltaY: -100, deltaMode: 2});
  assert.equal(f.run('centerOnRobot'), false);
  f.$('fitMap').handlers.click();
  assert.equal(f.run('manualMapView'), null);
  assertFits(f);
});


test('放大拖动后居中保持比例，重复居中及地图增长不改变显示大小', () => {
  const f = fixture();
  emit(f, 'wheel', 250, 120, {deltaY: -600});
  emit(f, 'pointerdown', 250, 120);
  emit(f, 'pointermove', 280, 140);
  emit(f, 'pointerup', 280, 140);
  const scale = f.run('mapView.scale / mapView.resolution');
  const assertCentered = () => {
    const translation = f.calls.filter(c => c[0] === 'translate').at(-1);
    close(translation[1], f.bounds.width / 2);
    close(translation[2], f.bounds.height / 2);
    close(f.run('mapView.scale / mapView.resolution'), scale);
  };
  f.$('centerMap').handlers.click();
  assertCentered();
  f.run('state.pose.x += 1; renderMap()');
  assertCentered();
  f.$('centerMap').handlers.click();
  assertCentered();
  f.run(`map = {...map, width: 1000, height: 400, resolution: 0.1,
    origin: {x: -5, y: -6, z: Math.sin(0.3), w: Math.cos(0.3)},
    data: Array(1000 * 400).fill(0)}; renderMap()`);
  f.bounds.width = 320;
  f.sandbox.resize();
  assertCentered();
  f.$('fitMap').handlers.click();
  assertFits(f);
});

test('初始位姿分两次选取，旋转地图中朝向正确且不生成导航目标', () => {
  const f = fixture(Math.PI / 3);
  f.$('pickInitialPose').handlers.click();
  const click = (x,y) => f.$('mapCanvas').handlers.click({currentTarget:f.$('mapCanvas'),
    clientX:f.bounds.left+x,clientY:f.bounds.top+y});
  click(300,140);
  const point = f.run('({...initialHint})');
  assert.equal(f.run('initialPick'), 'heading');
  assert.equal(f.sandbox.selectedGoal, null);
  click(350,140);
  close(Number(f.$('initialYaw').value), 60);
  assert.equal(f.run('initialPick'), null);
  close(f.run('initialHint.x'),point.x);
  assert.equal(f.sandbox.selectedGoal, null);
  assert.ok(f.calls.some(call => call[0] === 'lineTo' && call[1] === 24));
  f.$('cancelInitialPose').handlers.click();
  assert.equal(f.run('initialHint'), null);
});

test('位姿提交将角度换算成弧度，未完成选取时不提交', async () => {
  const f = fixture();
  const sent = [];
  f.sandbox.guarded = async action => action();
  f.sandbox.api = async (path,body) => { sent.push({path,body}); return {message:'waiting'}; };
  f.$('initialX').value = '1.2'; f.$('initialY').value = '-0.4'; f.$('initialYaw').value = '90';
  await f.$('submitInitialPose').handlers.click();
  assert.equal(sent[0].path,'localization/initial-pose');
  close(sent[0].body.yaw,Math.PI/2);
  assert.equal(sent[0].body.x,1.2);
  f.$('pickInitialPose').handlers.click();
  await assert.rejects(f.$('submitInitialPose').handlers.click(), /完成位置/);
  assert.equal(sent.length,1);
});

test('未确认定位时不绘制可信机器人标记', () => {
  const f = fixture();
  f.sandbox.state.localization = {required:true,ready:false};
  f.calls.length = 0;
  f.run('renderMap()');
  assert.ok(!f.calls.some(call => call[0] === 'arc' && call[3] === 3));
});
