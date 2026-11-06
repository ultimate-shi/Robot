/* 作用：验证四轮转向组合驾驶、默认键位、多组并行、失焦与平衡草稿保护；用法：node --test 本文件。 */
const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(`${__dirname}/../web/app.js`, 'utf8');
function fixture() {
  let now=1000;
  const events = {}, docEvents = {}, elements = {}, calls = [], intervals = [];
  const buttons = ['forward','back','left','right','stop'].map(key=>({dataset:{drive:key},handlers:{},
    addEventListener(name,fn){this.handlers[name]=fn;},setPointerCapture(){}}));
  const element = () => ({value:0, checked:true, hidden:false, handlers:{},style:{},
    set id(value){this._id=value;elements[value]=this;},get id(){return this._id;},
    setAttribute(){}, append(){}, addEventListener(k, fn){this.handlers[k]=fn;}});
  const $ = id => elements[id] ||= element();
  const state = {control:{mode:'manual',stopped:false},actuators:{leg_reason:'',head_reason:''}};
  const sandbox = vm.createContext({$, state, clientId:'test', driveSequence:0,
    document:{activeElement:null,hidden:false,querySelectorAll:selector=>selector==='[data-drive]'?buttons:[],createElement:element,
      addEventListener:(k,fn)=>docEvents[k]=fn},
    window:{addEventListener:(k,fn)=>events[k]=fn},
    Date:{now:()=>now},navigator:{sendBeacon(){}},Blob:class{},setInterval:fn=>intervals.push(fn),
    api:async (path,body)=>{calls.push({path,body});return {};},
    sendDrive:async (linear,angular)=>{calls.push({path:'drive',body:{linear,angular}});},
    guarded:async fn=>fn(),notice:()=>{},
  });
  $('linearSpeed').value=0.06; $('angularSpeed').value=0.1;
  vm.runInContext(source.slice(source.indexOf('  const velocity ='),
    source.indexOf('  for (const [id, output, suffix]')),sandbox);
  const key = (kind,value,repeat=false) => {
    let prevented=false;
    events[kind]({key:value,repeat,preventDefault(){prevented=true;}});
    return prevented;
  };
  return {sandbox,events,docEvents,calls,$,state,key,intervals,
    pointer(kind,key,id){buttons.find(b=>b.dataset.drive===key).handlers[kind]({pointerId:id,preventDefault(){}});},advance(ms){now+=ms;intervals.forEach(fn=>fn());}};
}
const flush = () => new Promise(resolve=>setImmediate(resolve));

test('停止锁和非手动模式拦截方向键滚屏，表单输入仍使用原生方向键',async()=>{
  const f=fixture();
  f.state.control.stopped=true;
  assert.equal(f.key('keydown','ArrowUp'),true);
  assert.equal(f.key('keydown','ArrowDown'),true);
  f.state.control.stopped=false;f.state.control.mode='navigation';
  assert.equal(f.key('keydown','ArrowUp'),true);
  f.sandbox.document.activeElement={tagName:'INPUT'};
  assert.equal(f.key('keydown','ArrowUp'),false);
  await flush();assert.equal(f.calls.length,0);
});

test('平衡勾选在失焦和连续状态刷新后保留，应用时提交启用',async()=>{
  const f=fixture();
  const disabled={fresh:true,pending:false,enabled:false,level:'mid',target_roll_deg:0,target_pitch_deg:0};
  f.sandbox.syncBalanceForm(disabled);
  f.$('balanceEnabled').checked=true;
  f.$('balanceEnabled').handlers.change();
  for(let i=0;i<5;i++) f.sandbox.syncBalanceForm(disabled);
  assert.equal(f.$('balanceEnabled').checked,true);
  f.$('applyBalance').handlers.click(); await flush();
  assert.equal(f.calls.find(c=>c.path==='body-balance').body.enabled,true);
  f.sandbox.syncBalanceForm({...disabled,enabled:true});
  f.sandbox.syncBalanceForm(disabled);
  assert.equal(f.$('balanceEnabled').checked,false);
});

test('平衡姿态草稿失焦后保留，未编辑时仍同步真实状态',()=>{
  const f=fixture();
  const b={fresh:true,pending:false,enabled:false,level:'mid',target_roll_deg:1,target_pitch_deg:2};
  f.sandbox.syncBalanceForm(b);
  assert.equal(f.$('balanceRoll').value,1);
  f.$('balanceRoll').value='3.5'; f.$('balanceRoll').handlers.input();
  f.sandbox.syncBalanceForm(b);
  assert.equal(f.$('balanceRoll').value,'3.5');
});

test('arrows drive while WASD and IJKL control independent joints',async()=>{
  const f=fixture();
  f.key('keydown','ArrowUp'); f.key('keydown','W'); f.key('keydown','I');
  await flush();
  assert.ok(f.calls.some(c=>c.path==='drive' && c.body.linear===0.06));
  const motion=f.calls.filter(c=>c.path==='joint-motion').at(-1).body;
  assert.equal(motion.pitch,0); assert.equal(motion.head_pitch,1);
  assert.equal(f.calls.find(c=>c.path==='joint-step').body.action,'w');
  f.key('keydown','S'); await flush();
  assert.equal(f.calls.filter(c=>c.path==='joint-step').length,1);
  f.key('keyup','W'); f.advance(300); await flush();
  assert.equal(f.calls.filter(c=>c.path==='joint-step').at(-1).body.action,'s');
});

test('key repeat does not create extra motion; blur and space release all groups',async()=>{
  const f=fixture(); f.key('keydown','e'); await flush();
  const n=f.calls.length; f.key('keydown','e',true); await flush(); assert.equal(f.calls.length,n);
  f.events.blur(); await flush();
  assert.equal(f.calls.filter(c=>c.path==='joint-motion').at(-1).body.lift,0);
  f.key('keydown','l'); await flush(); f.key('keydown',' '); await flush();
  assert.equal(f.calls.filter(c=>c.path==='joint-motion').at(-1).body.yaw,0);
});

test('form focus, nonmanual mode and keyboard disable suppress input',async()=>{
  const f=fixture();
  f.sandbox.document.activeElement={tagName:'INPUT'}; f.key('keydown','w');
  f.sandbox.document.activeElement=null; f.state.control.mode='navigation'; f.key('keydown','w');
  f.state.control.mode='manual'; f.$('keyboardEnabled').checked=false; f.key('keydown','w');
  await flush(); assert.equal(f.calls.length,0);
});

test('locked head does not prevent body and driving; route sends release',async()=>{
  const f=fixture(); f.state.actuators.head_reason='建图锁定';
  f.key('keydown','j'); f.key('keydown','a'); f.key('keydown','ArrowLeft'); await flush();
  assert.equal(f.calls.find(c=>c.path==='joint-step').body.action,'a');
  assert.equal(f.calls.filter(c=>c.path==='joint-motion').length,0);
  f.events['robot-route'](); await flush();
  assert.equal(f.calls.filter(c=>c.path==='joint-motion').at(-1).body.roll,0);
  assert.equal(f.calls.filter(c=>c.path==='drive').at(-1).body.angular,0);
});

test('HTML defaults enable keyboard and keep only the sidebar emergency stop',()=>{
  const html=fs.readFileSync(`${__dirname}/../web/index.html`,'utf8');
  assert.match(html,/id="keyboardEnabled"[^>]+checked/);
  assert.doesNotMatch(html,/id="stop"/);
  assert.match(html,/id="sideStop"[^>]*>紧急停止/);
  assert.match(html,/id="legSliders"/);
  const css=fs.readFileSync(`${__dirname}/../web/style.css`,'utf8');
  assert.match(css,/writing-mode:vertical-lr;direction:rtl/);
});


test('leg slider sends board degrees directly and retains untouched targets',async()=>{
  const f=fixture();
  f.state.actuators.joints=[15,5,-10,8].map((v,index)=>({group:'leg',index,
    hardware_target_deg:v,hardware_actual_deg:0}));
  const slider=f.$('joint-leg-1'); slider.value='15';
  slider.handlers.change(); await flush();
  const command=f.calls.find(c=>c.path==='joints').body;
  assert.equal(command.unit,'hardware_deg');
  assert.deepEqual(Array.from(command.values),[15,15,-10,8]);
  f.calls.length=0;
  slider.min=-30;slider.max=30;
  const number=f.$('joint-angle-1');number.value='-15';number.handlers.change();await flush();
  assert.deepEqual(Array.from(f.calls.find(c=>c.path==='joints').body.values),[15,-15,-10,8]);
});


test('held leg key repeats discrete steps at configured intervals and stops on release',async()=>{
  const f=fixture();
  f.state.actuators.leg_step_repeat_seconds=0.3;
  f.key('keydown','E'); await flush();
  assert.equal(f.calls.filter(c=>c.path==='joint-step').length,1);
  f.advance(100); await flush();
  assert.equal(f.calls.filter(c=>c.path==='joint-step').length,1);
  f.advance(200); await flush();
  f.advance(300); await flush();
  f.advance(300); await flush();
  assert.equal(f.calls.filter(c=>c.path==='joint-step').length,4);
  f.key('keyup','E'); f.advance(900); await flush();
  assert.equal(f.calls.filter(c=>c.path==='joint-step').length,4);
});

test('leg and head zero buttons each send one scoped request without a racing hold',async()=>{
  const f=fixture();
  f.$('legsZero').handlers.click(); await flush();
  assert.equal(f.calls.length,1);
  assert.equal(f.calls[0].path,'joints/zero');
  assert.equal(f.calls[0].body.group,'leg');
  f.calls.length=0;
  f.$('headZero').handlers.click(); await flush();
  assert.equal(f.calls.length,1);
  assert.equal(f.calls[0].path,'joints/zero');
  assert.equal(f.calls[0].body.group,'head');
  const html=fs.readFileSync(`${__dirname}/../web/index.html`,'utf8');
  assert.match(html,/id="headZero"[^>]*>头部回零/);
});

// 对外驾驶行为：只前进、边行驶边转弯、单独原地旋转，以及组合输入的释放。
for (const [direction,key,linear] of [['forward','ArrowUp',0.06],['back','ArrowDown',-0.06]]) {
  for (const [turn,turnKey,angular] of [['left','ArrowLeft',0.1],['right','ArrowRight',-0.1]]) {
    test(`${direction}+${turn} uses both configured speeds for keyboard and multitouch`,async()=>{
      for (const pointer of [false,true]) {
        const f=fixture();
        const drive=()=>f.calls.filter(c=>c.path==='drive').at(-1).body;
        const down=(d,k,id)=>pointer?f.pointer('pointerdown',d,id):f.key('keydown',k);
        const up=(d,k,id)=>pointer?f.pointer('pointerup',d,id):f.key('keyup',k);
        down(direction,key,1); await flush();
        assert.deepEqual(drive(),{linear,angular:0});
        down(turn,turnKey,2); await flush();
        assert.deepEqual(drive(),{linear,angular});
        f.advance(100); await flush();
        assert.deepEqual(drive(),{linear,angular});
        up(direction,key,1); await flush();
        assert.deepEqual(drive(),{linear:0,angular});
        up(turn,turnKey,2); await flush();
        assert.deepEqual(drive(),{linear:0,angular:0});
      }
    });
  }
}

test('mixed pointer and keyboard combine once per refresh; opposite directions cancel',async()=>{
  const f=fixture();
  const drive=()=>f.calls.filter(c=>c.path==='drive').at(-1).body;
  f.pointer('pointerdown','forward',1); f.key('keydown','ArrowLeft'); await flush();
  assert.deepEqual(drive(),{linear:0.06,angular:0.1});
  f.calls.length=0; f.advance(100); await flush();
  assert.equal(f.calls.filter(c=>c.path==='drive').length,1);
  f.key('keydown','ArrowRight'); await flush();
  assert.deepEqual(drive(),{linear:0.06,angular:0});
  f.pointer('pointercancel','forward',1); await flush();
  assert.deepEqual(drive(),{linear:0,angular:0});
});

test('pointer stop and blur clear all driving sources without resuming on refresh',async()=>{
  for (const end of ['stop','blur','pointerleave','lostpointercapture']) {
    const f=fixture(); f.pointer('pointerdown','forward',1);
    if (end==='stop') { f.key('keydown','ArrowLeft'); f.pointer('pointerdown','stop',3); }
    else if (end==='blur') { f.key('keydown','ArrowLeft'); f.events.blur(); }
    else f.pointer(end,'forward',1);
    await flush(); f.advance(100); await flush();
    assert.deepEqual(f.calls.filter(c=>c.path==='drive').at(-1).body,{linear:0,angular:0});
  }
});
