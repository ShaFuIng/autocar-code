const vm=require('node:vm'),fs=require('node:fs'),assert=require('node:assert/strict');
const source=fs.readFileSync(__dirname+'/pid-live.js','utf8').replace('})();','globalThis.test={beat,recover,stopOwned,tick,own:x=>{owned=x;lastRunId=x;},mode:x=>runMode=x,state:()=>({owned,preparedDoc,beatError,runMode})};})();');
function setup(state='ready'){
 const nodes={},calls=[],events=[],handlers={};let fail=false;
 const node=id=>nodes[id]??=( {textContent:'',value:id==='livePIDModel'?'v4-default':'',checked:true,disabled:false,options:[1],querySelector:()=>null});
 const current={state,run:{state:'stopped'},profile:{center_us:1360},model_id:'v4-default',row_ratio:.75,ready_frames:5,latest:{age_s:.05},models:[]};
 const ctx={console,crypto:{randomUUID:()=> 'a'.repeat(32)},performance,Date,setTimeout,clearTimeout,setInterval:()=>1,clearInterval:()=>{},AbortController,route:'other',$:node,notify:()=>{},MutationObserver:class{observe(){}},document:{hidden:false,hasFocus:()=>true,addEventListener:(k,v)=>handlers[k]=v},window:{addEventListener:(k,v)=>handlers[k]=v,PIDField:{isDirty:()=>false,observe:()=>{}}}};
 ctx.api=async(path,data)=>{calls.push(path);if(path.endsWith('heartbeat-probe'))return {rtt_ms:15,ready:true};if(path.endsWith('/status')&&path.includes('/live/'))return current;if(path==='/api/integration/status')return {vehicle:{vehicle:{state:'LOCKED',motor_a:{pwm_command:0},motor_b:{pwm_command:0}}}};if(path.includes('/document/'))return {rev:27,data:{parameters:{center_us:1360,row_ratio:.75,max_gap:.4,pwm:20}}};return {ok:true};};
 ctx.fetch=async(path,opts)=>{if(path.endsWith('connection-event')){events.push(JSON.parse(opts.body));return {};};calls.push(path);if(fail)throw Error('timeout');return {ok:true,json:async()=>({ok:true})};};
 vm.runInNewContext(source,ctx);return {ctx,calls,events,handlers,nodes,current,fail:()=>fail=true};
}
(async()=>{
 let x=setup();await x.ctx.test.recover();assert(x.ctx.test.state().preparedDoc);assert(!x.calls.some(p=>p.endsWith('/start')||p.endsWith('/prepare')));assert.equal(x.nodes.livePIDClear.checked,false);
 x=setup('idle');await x.ctx.test.recover();assert(x.calls.includes('/api/pid/live/prepare'));assert(!x.calls.includes('/api/pid/live/start'));
 x=setup();x.current.run.state='running';await x.ctx.test.recover();assert.equal(x.ctx.test.state().preparedDoc,null);assert(!x.calls.includes('/api/pid/live/prepare'));
 x=setup();x.ctx.test.own('r');await x.ctx.test.beat();assert.equal(x.events.at(-1).event,'heartbeat_ack');x.fail();await x.ctx.test.beat();assert.equal(x.ctx.test.state().owned,null);const n=x.calls.length;await x.ctx.test.beat();assert.equal(x.calls.length,n);
 x=setup();x.ctx.test.own('r');x.handlers.blur();assert.equal(x.ctx.test.state().owned,null);assert(x.events.some(e=>e.event==='blur'));assert(x.calls.includes('/api/pid/live/stop'));
 x=setup();x.current.run={id:'r',state:'stopped',reason:'PID_INVALID_OR_STALE_PATH'};x.ctx.test.own('r');x.fail();await x.ctx.test.beat();assert(x.ctx.test.state().beatError.includes('不是心跳斷線'));
 x=setup();x.ctx.test.own('r');x.ctx.test.mode('orin_timed');await x.ctx.test.beat();x.handlers.blur();x.ctx.document.hidden=true;x.handlers.visibilitychange();x.handlers.pagehide();x.handlers.hashchange();assert.equal(x.ctx.test.state().owned,'r');assert(!x.calls.some(p=>p.includes('/stop')||p.includes('/heartbeat')));
 x=setup();x.current.run={id:'r',state:'running',execution_mode:'orin_timed'};x.current.remaining_s=5;await x.ctx.test.recover();assert(x.nodes.livePIDRecovery.textContent.includes('只恢復觀測'));assert(!x.calls.some(p=>p.endsWith('/start')||p.endsWith('/prepare')||p.endsWith('/stop')));assert.equal(x.ctx.test.state().runMode,'orin_timed');
 x=setup();let release;const oldApi=x.ctx.api;x.ctx.api=(path,data)=>path==='/api/pid/live/status'?new Promise(r=>release=r):oldApi(path,data);const poll=x.ctx.test.tick();x.ctx.test.own('new-run');release(x.current);await poll;assert.equal(x.ctx.test.state().owned,'new-run');
 console.log('PASS: recovery reuse/reload/no auto-start, active-run rejection, heartbeat ACK/failure/no retry, blur stop+log');
})().catch(e=>{console.error(e);process.exitCode=1});
