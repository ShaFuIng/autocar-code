const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const html=fs.readFileSync(path.join(__dirname,'../web/manual.html'),'utf8');
const elements=new Map([...html.matchAll(/id="([^"]+)"/g)].map(m=>[m[1],{value:'',disabled:false,textContent:'',style:{},checked:true,tagName:'BUTTON',classList:{toggle(){}},setAttribute(){},listeners:{},addEventListener(n,f){this.listeners[n]=f},contains(t){return t===this||t===elements.get('camera')||t?.inside===true},focus(){},setPointerCapture(){},append(){},replaceChildren(){}}]));
for(const [id,value] of [['center','1460'],['right','10'],['left','15'],['steerRate','300'],['accelTime','2'],['decelTime','1']]){elements.get(id).value=value;elements.get(id).tagName='INPUT'}
const requests=[],listeners={},windowEvents={},timers=[];let fakeNow=1000;
class FakeWebSocket{
 static OPEN=1;
 constructor(){this.readyState=1;this.bufferedAmount=0;FakeWebSocket.current=this;queueMicrotask(()=>this.onmessage?.({data:JSON.stringify({type:'hello',freshness:'test'})}))}
 send(raw){const p=JSON.parse(raw);if(p.action!=='probe')requests.push({url:'/api/manual/'+p.action,body:p.payload});
  const result=p.action==='start'?{run_id:'c'.repeat(32)}:p.action==='settings'?{config:p.payload}:{};
  queueMicrotask(()=>this.onmessage?.({data:JSON.stringify({id:p.id,ok:true,freshness:'test',server_processing_ms:0,...result})}));
 }
 close(){this.readyState=3;this.onclose?.()}
}
const context={WebSocket:FakeWebSocket,location:{protocol:'http:',host:'test'},console,Math,Number,JSON,Error,AbortController,URL,Set,Map,TextDecoder,performance:{now(){return fakeNow}},
 crypto:{getRandomValues(x){return x.fill(1)}},setTimeout(fn,ms){timers.push({fn,ms})},clearTimeout(){},
 document:{hidden:false,getElementById(id){return elements.get(id)},addEventListener(name,fn){listeners[name]=fn}},
 window:{addEventListener(name,fn){windowEvents[name]=fn}},
 fetch(url,opts){if(!opts||opts.method!=='POST')return new Promise(()=>{});const body=JSON.parse(opts.body);requests.push({url,body});return Promise.resolve({ok:true,json:async()=>url.endsWith('/start')?{ok:true,run_id:'c'.repeat(32)}:url.endsWith('/settings')?{ok:true,config:body}:{ok:true}})}
};
vm.createContext(context);vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);
const flush=async()=>{for(let i=0;i<24;i++)await Promise.resolve()};
function key(name,k,target={tagName:'BODY'},repeat=false,code){let prevented=false;listeners[name]({key:k,code,target,repeat,preventDefault(){prevented=true}});return prevented}
const input=()=>requests.findLast(r=>r.url.endsWith('/input')).body;
async function main(){
 await flush();assert.equal(requests.length,0,'Loading the page never arms motors');
 const testConnection=context.runLinkTest();await flush();
 for(let i=0;i<160&&vm.runInContext('linkTesting',context);i++){
  const index=timers.findIndex(t=>t.ms===200&&t.fn.name==='');assert(index>=0);fakeNow+=200;timers.splice(index,1)[0].fn();await flush();
 }
 await testConnection;await flush();assert.equal(requests.filter(r=>r.url.endsWith('/start')).length,0,'Connection test never arms the vehicle');fakeNow=1000;

 vm.runInContext('streamError=null;lastRenderedAt=1000',context);
 await elements.get('enable').onclick();await flush();assert.equal(input().direction,0);assert.equal(input().steer,0);
 vm.runInContext("lastRun={id:runId,status:'running'};lastStatus={vehicle:{state:'MANUAL'}}",context);
 const heartbeatBefore=requests.length;
 for(let i=0;i<6;i++){const index=timers.findIndex(t=>t.fn.name==='inputTick');assert(index>=0,'Heartbeat must remain scheduled independently of rendering');timers.splice(index,1)[0].fn();await flush()}
 assert.equal(requests.length-heartbeatBefore,6,'Idle keyboard must still send periodic control inputs');
 const neutral=requests.length;key('keydown','w');await flush();assert.equal(requests.length,neutral,'Outside game focus must not drive');
 context.enterKeyboard();await flush();
 assert(key('keydown','ArrowUp',{tagName:'BODY'},false,'ArrowUp'),'Captured arrows suppress scroll');await flush();assert.equal(input().direction,1);
 key('keyup','ArrowUp',{tagName:'BODY'},false,'ArrowUp');await flush();
 key('keydown','w');await flush();assert.equal(input().direction,1);
 key('keydown','a');await flush();assert.equal(input().steer,-1);
 key('keyup','w');await flush();assert.equal(input().direction,0);assert.equal(input().hard_zero,false,'Normal throttle release uses smooth deceleration');
 key('keyup','a');await flush();assert.equal(input().steer,0,'Released steering requests hold rather than automatic center');assert.equal(input().center,false);
 key('keydown','c',{tagName:'BODY'},false,'KeyC');await flush();assert.equal(input().center,true);
 key('keydown','a');await flush();key('keydown','d');await flush();assert.equal(input().steer,1,'Last steering key wins');assert.equal(input().center,false);
 key('keyup','d');await flush();assert.equal(input().steer,-1,'Releasing last steering key restores still-held first key');key('keyup','a');await flush();
 listeners.pointerdown({target:elements.get('center')});await flush();assert.equal(input().direction,0);assert.equal(input().steer,0);assert.equal(requests.filter(r=>r.url.endsWith('/stop')).length,0,'Leaving game focus neutralizes without locking');
 const before=requests.length;key('keydown','w',{tagName:'INPUT'});await flush();assert.equal(requests.length,before,'Editing must not drive');
 context.enterKeyboard();await flush();key('keydown','w');await flush();key('keydown','s');await flush();assert.equal(input().direction,0,'Conflicting throttle keys neutralize');
 key('keydown',' ');await flush();assert.equal(input().direction,0);assert.equal(input().hard_zero,true,'Space bypasses deceleration');
 key('keydown','w',{tagName:'BODY'},true);await flush();assert.equal(input().direction,0,'Held key repeats cannot restart after Space');
 await elements.get('centerPlus').onclick();await flush();assert.equal(requests.findLast(r=>r.url.endsWith('/settings')).body.center_us,1465);
 await elements.get('save').onclick();assert.equal(requests.at(-1).body.save,true);assert.equal(requests.at(-1).body.accel_seconds,2);assert.equal(requests.at(-1).body.decel_seconds,1);
 windowEvents.blur();await flush();assert.equal(requests.at(-1).url,'/api/manual/stop');assert.equal(requests.at(-1).body.reason,'WINDOW_BLUR');
 const stopped=requests.length;await context.sendInput();assert.equal(requests.length,stopped,'No automatic renewal after focus loss');
 assert.equal(requests.filter(r=>r.url.endsWith('/start')).length,1,'No automatic rearm');
 const passive=requests.length;windowEvents.blur();await flush();assert.equal(requests.length,passive,'Passive page blur does not stop another owner');
 // Snapshot preview: off must stop downloads while preserving health gates.
 let rendered=0;
 elements.get('camera').getContext=()=>({drawImage(){rendered++}});
 context.Blob=Blob;context.createImageBitmap=async()=>({width:2,height:2,close(){}});
 const originalFetch=context.fetch;let imageRequests=0;
 context.fetch=(url,opts)=>url.startsWith('/camera.jpg')?(imageRequests++,Promise.resolve({ok:true,arrayBuffer:async()=>new Uint8Array([255,216,255,217]).buffer})):originalFetch(url,opts);
 await context.camera();await flush();assert.equal(rendered,1);assert.equal(vm.runInContext('previewFresh()',context),true);
 fakeNow=2001;assert.equal(vm.runInContext('previewFresh()',context),false);fakeNow=1000;
 elements.get('previewRate').value='0';elements.get('previewRate').onchange();
 vm.runInContext('lastStatus={camera_age_s:.1,uart_age_s:.1}',context);
 const count=imageRequests;await context.camera();assert.equal(imageRequests,count);assert.equal(vm.runInContext('previewFresh()',context),true);
 vm.runInContext('lastStatus.camera_age_s=2',context);assert.equal(vm.runInContext('previewFresh()',context),false);
 elements.get('previewRate').value='5';elements.get('previewRate').onchange();
 // New intent does not wait for old ACKs; backpressure closes instead of queueing.
 vm.runInContext("runId='cccccccccccccccccccccccccccccccc';lastRun={id:runId,status:'running'};lastStatus={vehicle:{state:'MANUAL'}};keyboardCaptured=true;streamError=null;lastRenderedAt=1000",context);
 const socket=FakeWebSocket.current,send=socket.send.bind(socket);let held;
 socket.send=raw=>{const p=JSON.parse(raw);if(p.action==='input'&&!held){held=p;requests.push({url:'/api/manual/input',body:p.payload})}else send(raw)};
 key('keydown','w');await flush();const oldSequence=input().seq;
 context.exitKeyboard();await flush();assert.equal(input().direction,0);assert(input().seq>oldSequence);
 socket.onmessage({data:JSON.stringify({id:held.id,ok:true,freshness:'test',server_processing_ms:0})});await flush();
 socket.send=send;socket.bufferedAmount=8193;
 await context.sendInput();await flush();assert.equal(vm.runInContext('runId',context),null,'Backpressure cannot preserve driving');
 assert.equal(socket.readyState,3);socket.bufferedAmount=0;
 context.connectControl();await flush();assert.equal(vm.runInContext('runId',context),null,'Reconnect never resumes driving');
 // Normal queued bytes during overlapping probe/input sends are not congestion.
 FakeWebSocket.current.bufferedAmount=300;
 const parallelA=context.controlRequest('probe'),parallelB=context.controlRequest('probe');
 await Promise.all([parallelA,parallelB]);assert.equal(FakeWebSocket.current.readyState,1);
 FakeWebSocket.current.bufferedAmount=0;
 // A missing ACK closes the socket without extending the original lease.
 const unanswered=FakeWebSocket.current;unanswered.send=()=>{};
 const timedOut=context.controlRequest('probe').catch(e=>e.message);await flush();
 timers.filter(t=>t.ms===800).at(-1).fn();
 assert.match(await timedOut,/800 ms/);assert.equal(unanswered.readyState,3);
 context.connectControl();await flush();
 // Diagnostics are rate-limited rather than creating a request storm.
 fakeNow=Math.max(fakeNow+1100,vm.runInContext('lastDiagnosticAt',context)+1100);await flush();const beforeDiag=requests.length;context.diagnostic('preview_error','test');context.diagnostic('preview_error','test');await flush();assert.equal(requests.length,beforeDiag+1);
 fakeNow+=1100;
 // Failed start is visible in Chinese and logs its real server reason.
 context.fetch=(url,opts)=>url==='/api/manual/start'?Promise.resolve({ok:false,status:409,json:async()=>({ok:false,reason:'Confirm clear operating area'})}):originalFetch(url,opts);
 await assert.rejects(context.httpPost('start',{}),/已確認周圍無人及障礙/);await flush();
 assert(requests.some(r=>r.url.endsWith('/diagnostic')&&r.body.detail?.includes('start HTTP 409: Confirm clear operating area')));
 console.log('Manual UI: explicit arm, deadman W/S, A/D hold and C return, input-field exclusion, conflict/Space/focus stop, trim saving passed');
}
main().catch(e=>{console.error(e);process.exitCode=1});
