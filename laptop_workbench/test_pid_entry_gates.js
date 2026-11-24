// Execute the actual UI handlers in a VM. Every API is an in-memory fake.
// No browser, HTTP, SSH, timers, hardware or old test suites are invoked.
const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path'),crypto=require('crypto');
const originalSource=fs.readFileSync(path.join(__dirname,'pid-live.js'),'utf8');
const source=originalSource.replace(/\r\n/g,'\n');
const startBody=source.split("$('livePIDStart').onclick=async()=>{")[1].split("\n  $('livePIDStop').onclick=")[0];
assert(startBody.endsWith('};'));
const exported=`globalThis.testUI={recover,tick,start:async(host)=>{${startBody.slice(0,-2)}},
 seed(s,d){last=s;preparedDoc=d;tick.lastPoll=0;},
 state(){return {owned,preparedDoc,runMode,lastRTT};}};`;
const script=source.replace('new MutationObserver(mount).observe($(\'main\'),{childList:true});mount();',exported);
assert(script.includes('globalThis.testUI'));
const clone=x=>JSON.parse(JSON.stringify(x));

function fixture(options={}){
 let now=10000;
 const doc={rev:7,data:{parameters:{row_ratio:.6,center_us:1360,max_gap:.4,pwm:5}}};
 const status={path_algorithm:'calibrated-v7',path_algorithms:[{id:'calibrated-v7',preview:true,control:true}],
  autonomous_pid_version:1,state:'ready',ready_frames:3,
  latest:{age_s:4,inference_ms:200,path_diagnostic:{calibration_sha256:'v7'}},
  profile:{center_us:1360,min_us:500,max_us:2500},model_id:'v4-default',row_ratio:.6,run:null,...options.status};
 const elements=new Map(),calls=[],messages=[];
 for(const id of ['livePID','livePIDPath','livePIDModel','livePIDPose','livePIDClear','livePIDStart',
  'livePIDSeconds','livePIDRecovery','livePIDState','livePIDHeartbeat','livePIDMode','livePIDPathStatus',
  'livePIDProfile','livePIDImage','livePIDParam'])
  elements.set(id,{value:'',checked:true,disabled:true,isConnected:true,options:[{}],textContent:''});
 elements.get('livePIDPath').value='calibrated-v7';
 elements.get('livePIDModel').value='v4-default';
 elements.get('livePIDSeconds').value='3';
 const context={route:'overview',console,crypto:{randomUUID:()=> 'a'.repeat(32)},performance:{now:()=>now},
  Date,AbortController,setTimeout:()=>0,clearTimeout(){},setInterval:()=>0,clearInterval(){},
  $:id=>elements.get(id),notify:m=>messages.push(m),
  window:{addEventListener(){},PIDField:{isDirty:()=>false,observe(){}}},
  document:{hidden:false,hasFocus:()=>true,addEventListener(){}},
  fetch:async()=>({ok:true,json:async()=>({})}),
  api:async(url,body)=>{
   calls.push({url,body});now+=900; // Successful slow communication (>500ms).
   if(url==='/api/pid/heartbeat-probe')throw Error('Unexpected heartbeat gate');
   if(url==='/api/pid/live/status'){
    if(options.statusError)throw Error('STATUS_UNAVAILABLE');
    return clone(status);
   }
   if(url==='/api/document/visual-pid')return clone(doc);
   if(url==='/api/integration/status')return {vehicle:{vehicle:{state:'READY',
    motor_a:{pwm_command:options.moving?1:0},motor_b:{pwm_command:0}}}};
   if(url==='/api/pid/live/prepare')return {ok:true};
   if(url==='/api/pid/live/start'){
    if(options.startError)throw Error(options.startError);
    status.run={id:'fake-run',state:'running',execution_mode:'orin_timed'};
    status.remaining_s=3;
    return {run_id:'fake-run',execution_mode:'orin_timed'};
   }
   throw Error('Unexpected API '+url);
  }};
 vm.createContext(context);vm.runInContext(script,context);
 context.route='drive';context.testUI.seed(status,doc);
 return {ui:context.testUI,elements,calls,messages,context,status,doc};
}

const results=[];
async function test(name,fn){
 try{await fn();results.push({name,passed:true});}
 catch(error){results.push({name,passed:false,error:String(error.stack)});}
}
(async()=>{
 await test('查詢時影格年齡4秒不再禁用開始；由車端裁決',async()=>{
  const f=fixture();await f.ui.tick();assert.equal(f.elements.get('livePIDStart').disabled,false);
  assert.match(f.elements.get('livePIDState').textContent,/非PID消費延遲/);
 });
 await test('車端無有效路徑時仍不啟用開始',async()=>{
  const f=fixture({status:{state:'no_reliable_path',ready_frames:0}});await f.ui.tick();
  assert.equal(f.elements.get('livePIDStart').disabled,true);
 });
 await test('限時開始不要求500ms探測；成功只派送一次',async()=>{
  const f=fixture();await f.ui.start({isConnected:true});
  assert.equal(f.calls.filter(c=>c.url.endsWith('/start')).length,1);
  assert(!f.calls.some(c=>c.url.includes('heartbeat-probe')));
  assert.equal(f.ui.state().owned,'fake-run');assert(f.ui.state().lastRTT>=900);
 });
 for(const reason of ['Need 3 consecutive fresh valid model results','ACK_UNKNOWN']){
  await test('車端拒絕或回覆未知不重試：'+reason,async()=>{
   const f=fixture({startError:reason});await f.ui.start({isConnected:true});
   assert.equal(f.calls.filter(c=>c.url.endsWith('/start')).length,1);
   assert.equal(f.ui.state().owned,null);assert.equal(f.ui.state().preparedDoc,null);
   assert(f.messages.some(m=>m.includes(reason)));
  });
 }
 await test('慢網路恢復進行中限時任務只讀觀測',async()=>{
  const f=fixture({status:{run:{id:'existing',state:'running',execution_mode:'orin_timed'},remaining_s:2}});
  await f.ui.recover();
  assert(f.calls.every(c=>c.url==='/api/pid/live/status'));
  assert.match(f.elements.get('livePIDRecovery').textContent,/只恢復觀測/);
 });
 await test('已有匹配模型可恢復觀測、不要求LOCKED或重新載入',async()=>{
  const f=fixture();await f.ui.recover();
  assert(!f.calls.some(c=>c.url.includes('/prepare')||c.url.includes('/integration/')));
  assert.equal(f.ui.state().preparedDoc.rev,7);
 });
 await test('READY且兩輪0允許恢復時載入閒置預覽',async()=>{
  const f=fixture({status:{state:'idle'}});await f.ui.recover();
  assert.equal(f.calls.filter(c=>c.url.endsWith('/prepare')).length,1);
  assert(!f.calls.some(c=>c.url.endsWith('/start')));
 });
 await test('兩輪非0時恢復不載入新預覽',async()=>{
  const f=fixture({status:{state:'idle'},moving:true});await f.ui.recover();
  assert(!f.calls.some(c=>c.url.endsWith('/prepare')||c.url.endsWith('/start')));
  assert.match(f.elements.get('livePIDRecovery').textContent,/兩輪PWM 0/);
 });
 await test('狀態通道失敗不載入、不開始',async()=>{
  const f=fixture({statusError:true});await f.ui.recover();
  assert.equal(f.calls.length,1);
  assert.match(f.elements.get('livePIDRecovery').textContent,/STATUS_UNAVAILABLE/);
 });
 await test('舊心跳模式執行中仍不接管',async()=>{
  const f=fixture({status:{run:{id:'old',state:'running',execution_mode:'browser_lease'}}});
  await f.ui.recover();assert.equal(f.calls.length,1);
  assert.match(f.elements.get('livePIDRecovery').textContent,/不接管/);
 });
 const report={tests:results.length,passed:results.every(r=>r.passed),results,
  source_sha256:crypto.createHash('sha256').update(originalSource).digest('hex'),
  test_sha256:crypto.createHash('sha256').update(fs.readFileSync(__filename)).digest('hex'),
  scope:'僅本次前端門檻差異測試；API全為記憶體假端點，未連Orin或HTTP，未重跑95項既有測試。'};
 fs.writeFileSync(path.join(__dirname,'data/experiments/v7-pid-integration/entry-gates-tests.json'),JSON.stringify(report,null,2));
 console.log(JSON.stringify(report,null,2));process.exitCode=report.passed?0:1;
})();
