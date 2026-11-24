const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');
const html=fs.readFileSync(path.join(__dirname,'../web/index.html'),'utf8');
const elements=new Map([...html.matchAll(/id="([^"]+)"/g)].map(m=>[m[1],{
 value:'',disabled:false,textContent:'',style:{},classList:{toggle(){}},setAttribute(){}
}]));
for(const [id,value] of [['motorWheel','both'],['motorDirection','forward'],['motorRight','10'],['motorLeft','10'],['motorSeconds','5'],['motorWarmup','1'],['motorLabel','']])elements.get(id).value=value;
elements.get('motorEqual').checked=elements.get('wheelsRaised').checked=true;
const requests=[],timers=[];
let reply={ok:true,active:true};
const context={console,Math,Number,JSON,Error,AbortController,
 crypto:{getRandomValues(x){return x.fill(1)}},location:{hostname:'test'},
 setTimeout(fn,ms){timers.push({fn,ms});return timers.length},clearTimeout(){},
 document:{hidden:false,getElementById(id){return elements.get(id)},addEventListener(){}},
 fetch(url,options){
  if(!options)return new Promise(()=>{}); // Status/camera requests deliberately never finish.
  requests.push({url,body:JSON.parse(options.body),signal:options.signal});
  return Promise.resolve({ok:true,json:async()=>url==='/api/motor/start'?{ok:true,run_id:'c'.repeat(32)}:reply});
 }
};
vm.createContext(context);vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);
const flush=async()=>{for(let i=0;i<8;i++)await Promise.resolve()};
const count=()=>requests.filter(r=>r.url==='/api/motor/heartbeat').length;
async function main(){
 assert.equal(count(),0,'Page load must never take ownership or renew a test');
 await elements.get('motorStart').onclick();await flush();
 assert.equal(count(),1,'First heartbeat is immediate after explicit start');
 assert(requests.at(-1).signal,'Heartbeat has its own bounded request');
 assert.equal(requests.at(-1).body.run_id,'c'.repeat(32));
 // The status poll started at page load is still pending. Its delay must not gate heartbeat.
 const tick=timers.find(t=>t.fn.name==='heartbeatTick'&&t.ms===200);
 assert(tick);tick.fn();await flush();assert.equal(count(),2);
 context.document.hidden=true;await context.motorHeartbeat();assert.equal(count(),2);
 context.document.hidden=false;elements.get('motorPanel').style.display='none';
 await context.motorHeartbeat();assert.equal(count(),2);
 elements.get('motorPanel').style.display='block';
 reply={ok:true,active:false,status:'completed',reason:'DURATION_COMPLETE'};
 await context.motorHeartbeat();assert.equal(count(),3);
 assert.match(elements.get('motorMessage').textContent,/正常完成/);
 await context.motorHeartbeat();assert.equal(count(),3,'Completion must stop lease renewal');
 assert.equal(requests.filter(r=>r.url==='/api/motor/start').length,1,'No automatic restart');
 console.log('Heartbeat UI: stalled status/camera do not gate renewal; hidden pages do not renew; completion does not restart');
}
main().catch(e=>{console.error(e);process.exitCode=1});
