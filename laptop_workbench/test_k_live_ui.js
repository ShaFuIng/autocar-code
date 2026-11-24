const fs=require('fs'),vm=require('vm'),path=require('path');
const helper=fs.readFileSync(path.join(__dirname,'test_pid_entry_gates.js'),'utf8').split('(async()=>{')[0].replace("const context={route:","const context={pathName:x=>x,route:");
const cases=`
(async()=>{
 for(const algorithm of ['junction-k-lane-v3']){
  const f=fixture({status:{path_algorithm:algorithm,path_algorithms:[{id:algorithm,preview:true,control:true}]}});
  f.elements.get('livePIDPath').value=algorithm;f.elements.get('livePIDPose').checked=false;
  f.ui.seed(f.status,f.doc);await f.ui.tick();assert.equal(f.elements.get('livePIDStart').disabled,false);
  await f.ui.start({isConnected:true});assert.equal(f.calls.filter(c=>c.url==='/api/pid/live/start').length,1);
  assert.equal(f.calls.find(c=>c.url==='/api/pid/live/start').body.path_algorithm,algorithm);
 }
 const old=fixture({status:{path_algorithms:[]}});old.elements.get('livePIDPath').value='road-image-v3';old.ui.seed(old.status,old.doc);await old.ui.tick();assert.equal(old.elements.get('livePIDStart').disabled,true);
 console.log('K menu/start protocol and missing capability cases passed; fake API only, no historical cases executed.');
})().catch(e=>{console.error(e);process.exitCode=1;});`;
vm.runInNewContext(helper+cases,{require,__dirname,__filename,console,process,AbortController,setTimeout,clearTimeout});
