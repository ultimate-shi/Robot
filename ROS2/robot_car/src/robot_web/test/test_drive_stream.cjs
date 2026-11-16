/* 作用：验证持续驾驶连接不排队重放旧按键且松键优先；用法：node --test 本文件。 */
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const source=fs.readFileSync(`${__dirname}/../web/app.js`,'utf8');
function fixture(){
  const sockets=[],http=[];let now=1000;
  class Socket{
    constructor(){this.readyState=0;this.bufferedAmount=0;this.sent=[];sockets.push(this);}
    send(data){this.sent.push(JSON.parse(data));}
    close(){this.readyState=3;this.onclose?.();}
  }
  const context=vm.createContext({WebSocket:Socket,Date:{now:()=>now},Promise,JSON,
    location:{protocol:'http:',host:'robot:9100'},clientId:'page',driveSequence:0,
    api:async(path,body)=>{http.push({path,body});return {ok:true};},notice(){},releaseAll(){}});
  vm.runInContext(source.slice(source.indexOf('  function createDriveSender()'),source.indexOf('  const velocity =')),context);
  return {sockets,http,context,advance:ms=>{now+=ms;},
    healthy:()=>vm.runInContext('sendDrive.isHealthy()',context),
    send:(v,w=0)=>{context.linear=v;context.angular=w;return vm.runInContext('sendDrive(linear,angular)',context);}};
}
test('松键发生在建连前时，不重放早先的前进意图',async()=>{
  const f=fixture();await f.send(.07);await f.send(0);
  const socket=f.sockets[0];socket.readyState=1;socket.onopen?.();
  assert.equal(socket.sent.length,0);
  await f.send(0);
  assert.equal(socket.sent.at(-1).linear,0);
});
test('长按持续复用同一连接，图像 HTTP 不参与续期',async()=>{
  const f=fixture();await f.send(.07);f.sockets[0].readyState=1;
  for(let i=0;i<30;i++)await f.send(.07,.1);
  assert.equal(f.http.length,1);assert.equal(f.sockets.length,1);
  assert.equal(f.sockets[0].sent.length,30);
  await f.send(0);assert.equal(f.sockets[0].sent.at(-1).linear,0);
});
test('发送缓冲忙时丢弃旧移动意图，松键关闭流并补零速',async()=>{
  const f=fixture();await f.send(.07);const socket=f.sockets[0];socket.readyState=1;socket.bufferedAmount=100;
  await f.send(.07);assert.equal(socket.sent.length,0);
  await f.send(0);
  assert.equal(socket.readyState,3);assert.equal(f.http.at(-1).body.linear,0);
});

test('驾驶健康状态使用服务端超时和有效确认，过期后失效',async()=>{
  const f=fixture();await f.send(.07);const socket=f.sockets[0];socket.readyState=1;
  assert.equal(f.healthy(),false);
  socket.onmessage({data:JSON.stringify({ok:true,accepted:true,timeout_seconds:.35})});
  assert.equal(f.healthy(),true);f.advance(351);assert.equal(f.healthy(),false);
});

for(const healthy of [true,false])test(`页面状态超时${healthy?'保留健康驾驶流':'释放失效驾驶流'}`,async()=>{
  const elements={};let releases=0;
  const context=vm.createContext({pollBusy:false,statusConnected:true,legacyBackend:false,
    $:id=>elements[id]??={textContent:'',hidden:false},api:async()=>{throw Error('timeout');},
    sendDrive:{isHealthy:()=>healthy},releaseAll:()=>releases++,notice(){}});
  vm.runInContext(source.slice(source.indexOf('  async function poll()'),source.indexOf('  async function pollNavigation()')),context);
  await vm.runInContext('poll()',context);
  assert.equal(releases,healthy?0:1);
});
