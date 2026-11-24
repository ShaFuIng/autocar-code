
const el=id=>document.getElementById(id),owner=Array.from(crypto.getRandomValues(new Uint8Array(16)),x=>x.toString(16).padStart(2,'0')).join('');
let runId=null,lastRun=null,lastStatus=null,inputBusy=false,sequence=0,settingsBusy=false,dirty=false,initialConfig=null,blocked=false,keyboardCaptured=false;
const keys=new Set(),pointers=new Map(),heldCodes=new Map();
const keyMap={KeyW:"w",ArrowUp:"w",KeyS:"s",ArrowDown:"s",KeyA:"a",ArrowLeft:"a",KeyD:"d",ArrowRight:"d"};
const viewport=el("driveViewport");
let pendingInputEvent="HEARTBEAT",inputFlight=0,inputController=null;
const active=r=>r&&['preparing','running','stopping'].includes(r.status);
const editable=e=>!!e&&(e.isContentEditable||['INPUT','TEXTAREA','SELECT'].includes(e.tagName));
async function post(action,payload,signal){const r=await fetch('/api/manual/'+action,{method:'POST',cache:'no-store',signal,headers:{'Content-Type':'application/json','X-Orin-Control':'1'},body:JSON.stringify(payload)});const d=await r.json();if(!r.ok||!d.ok)throw Error(d.reason||'操作失敗');return d}
function config(){return {center_us:Number(el('center').value),right_pct:Number(el('right').value),left_pct:Number(el('left').value)}}
function fill(c){el('center').value=c.center_us;el('right').value=c.right_pct;el('left').value=c.left_pct}
async function apply(save=false){if(settingsBusy)return;settingsBusy=true;try{const d=await post('settings',{...config(),save,owner,run_id:runId});dirty=true;fill(d.config);el('message').textContent=save?'設定已保存；下次啟用也會使用。':'已套用本次設定。'}catch(e){el('message').textContent=e.message}finally{settingsBusy=false}}
el('apply').onclick=()=>apply();el('save').onclick=()=>apply(true);el('restore').onclick=()=>{if(initialConfig){fill(initialConfig);apply()}};
for(const [id,field,delta] of [['centerMinus','center',-5],['centerPlus','center',5],['rightMinus','right',-1],['rightPlus','right',1],['leftMinus','left',-1],['leftPlus','left',1]])el(id).onclick=()=>{if(settingsBusy)return;el(field).value=Number(el(field).value)+delta;dirty=true;apply()};
for(const id of ['center','right','left'])el(id).oninput=()=>{dirty=true};
function axes(){return {direction:blocked?0:Number(keys.has('w'))-Number(keys.has('s')),steer:Number(keys.has('d'))-Number(keys.has('a'))}}
async function sendInput(event="HEARTBEAT"){if(event!=="HEARTBEAT")pendingInputEvent=event;if(!runId||document.hidden)return;const urgent=['KEY_UP','CONTROL_AREA_EXIT','SPACE_ZERO','INPUT_FIELD'].includes(event);if(inputBusy){if(!urgent)return;inputController?.abort()}const flight=++inputFlight;inputBusy=true;const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),600);inputController=controller;const id=runId,eventToSend=pendingInputEvent;pendingInputEvent='HEARTBEAT';try{await post('input',{owner,run_id:id,seq:++sequence,...axes(),event:eventToSend},controller.signal)}catch(e){if(runId===id&&flight===inputFlight)el('message').textContent='控制意圖未確認；持续失聯會停止：'+e.message}finally{clearTimeout(timer);if(flight===inputFlight){inputBusy=false;inputController=null}}}
function inputTick(){sendInput();setTimeout(inputTick,200)}inputTick();
async function stopDriving(reason='BUTTON_STOP',explicit=false){keyboardCaptured=false;clearKeys();blocked=true;paintControls();const id=runId;runId=null;if(!id&&!explicit)return;try{await post('stop',{reason,owner,run_id:id});el('message').textContent='已要求停止並鎖定；重新按放實體啟動鍵才可再啟用。'}catch(e){el('message').textContent='停止未確認；請按實體急停。'}}
function clearKeys(){keys.clear();pointers.clear();heldCodes.clear()}
function zero(){clearKeys();blocked=true;paintControls();sendInput('SPACE_ZERO')}
el('stop').onclick=()=>stopDriving('BUTTON_STOP',true);el('hudStop').onclick=()=>stopDriving('BUTTON_STOP',true);el('zero').onclick=zero;
el('enable').onclick=async()=>{try{if(!previewFresh())throw Error('等待即時影像恢復後再啟用');clearKeys();keyboardCaptured=false;blocked=false;sequence=0;initialConfig=config();const d=await post('start',{owner,...config(),record:el('recordOnStart').checked,area_clear:el('areaClear').checked});runId=d.run_id;dirty=true;sendInput();el('message').textContent='手動模式準備中；沒有按住行車鍵不會走。'}catch(e){el('message').textContent=e.message}};
function canDrive(){return runId&&lastRun?.id===runId&&lastRun.status==='running'&&lastStatus?.vehicle?.state==='MANUAL'}
function syncKeys(){keys.clear();for(const key of heldCodes.values())keys.add(key);for(const key of pointers.values())keys.add(key);paintControls()}
function paintControls(){
 viewport.classList.toggle('captured',keyboardCaptured);viewport.setAttribute('aria-label',keyboardCaptured?'鍵盤駕駛中；點擊區外退出':'點擊進入鍵盤駕駛');
 el('keyboardState').textContent=keyboardCaptured?'鍵盤已接管 · WASD / ↑↓←→':canDrive()?'點擊畫面進入鍵盤駕駛':'先啟用手動模式，再點擊畫面';
 for(const [key,id] of [['w','forward'],['s','reverse'],['a','leftSteer'],['d','rightSteer']]){el(id).classList.toggle('pressed',keys.has(key));el('key'+key.toUpperCase()).classList.toggle('pressed',keys.has(key))}
}
function enterKeyboard(){
 if(!canDrive()||!previewFresh()){el('message').textContent='先啟用手動模式，確認即時畫面正常。';return}
 clearKeys();blocked=true;keyboardCaptured=true;viewport.focus({preventScroll:true});paintControls();sendInput('CONTROL_AREA_ENTER');
}
function exitKeyboard(event='CONTROL_AREA_EXIT'){
 if(!keyboardCaptured&&!keys.size&&!pointers.size)return;
 keyboardCaptured=false;clearKeys();blocked=true;paintControls();sendInput(event);
 el('message').textContent='已退出鍵盤駕駛，動力歸零、轉向逐步回中心；點回畫面可繼續。';
}
viewport.onpointerdown=e=>{if(!e.target.closest?.('button')){e.preventDefault();enterKeyboard()}};
viewport.addEventListener('focusout',e=>{if(!viewport.contains(e.relatedTarget))exitKeyboard()});
document.addEventListener('pointerdown',e=>{if(!viewport.contains(e.target))exitKeyboard()});
document.addEventListener('keydown',e=>{
 const code=e.code||({w:'KeyW',s:'KeyS',a:'KeyA',d:'KeyD',' ':'Space',Escape:'Escape'}[e.key]||e.key);
 if(code==='Escape'){e.preventDefault();stopDriving('ESC_STOP',true);return}
 if(e.ctrlKey||e.metaKey||e.altKey){exitKeyboard();return}
 const key=keyMap[code],shortcut=['Space','Equal','Minus','NumpadAdd','NumpadSubtract'].includes(code);
 if(!keyboardCaptured||editable(e.target)||(!key&&!shortcut))return;
 e.preventDefault(); // Suppress scrolling even for repeats or while state updates arrive.
 if(e.repeat||!canDrive())return;
 if(code==='Space'){zero();return}
 if(key){if(['w','s'].includes(key))blocked=false;heldCodes.set(code,key);syncKeys();sendInput('KEY_DOWN');return}
 const delta=['Minus','NumpadSubtract'].includes(code)?-1:1;el('right').value=Math.max(0,Math.min(25,Number(el('right').value)+delta));el('left').value=Math.max(0,Math.min(25,Number(el('left').value)+delta));apply();
});
document.addEventListener('keyup',e=>{const code=e.code||({w:'KeyW',s:'KeyS',a:'KeyA',d:'KeyD'}[e.key]||e.key);if(keyMap[code]){if(keyboardCaptured)e.preventDefault();heldCodes.delete(code);syncKeys();sendInput('KEY_UP')}});
document.addEventListener('focusin',e=>{if(editable(e.target))exitKeyboard('INPUT_FIELD')});
document.addEventListener('visibilitychange',()=>{if(document.hidden){stopDriving('PAGE_HIDDEN');streamController?.abort()}});
window.addEventListener('blur',()=>stopDriving('WINDOW_BLUR'));window.addEventListener('pagehide',()=>{stopDriving('PAGE_EXIT');streamController?.abort()});
for(const [id,key] of [['forward','w'],['reverse','s'],['leftSteer','a'],['rightSteer','d']]){
 const button=el(id);button.onpointerdown=e=>{if(!canDrive()||!previewFresh())return;e.preventDefault();clearKeys();keyboardCaptured=true;blocked=false;viewport.focus({preventScroll:true});pointers.set(e.pointerId,key);syncKeys();button.setPointerCapture(e.pointerId);sendInput('POINTER')};
 const release=e=>{pointers.delete(e.pointerId);syncKeys();sendInput('POINTER')};button.onpointerup=release;button.onpointercancel=release;button.onlostpointercapture=release;
}
el('recordToggle').onclick=async()=>{try{await post('record',{owner,run_id:runId,record:!lastRun?.recording})}catch(e){el('message').textContent=e.message}};
el('files').onclick=async()=>{const session=el('sessionSelect').value||lastRun?.id;if(!session)return;try{const r=await fetch('/api/manual/files/'+session);if(!r.ok)throw Error('讀取檔案失敗');const files=await r.json();el('downloads').replaceChildren();for(const f of files){const a=document.createElement('a');a.href='/api/manual/file/'+session+'/'+f.name;a.textContent=f.name+' ('+Math.round(f.size/1024)+' KB)';a.download=f.name;el('downloads').append(a,document.createElement('br'))}}catch(e){el('message').textContent=e.message}};
async function poll(){try{
 const r=await fetch('/api/manual/status',{cache:'no-store'});if(!r.ok)throw Error('資料不可用');const d=await r.json();lastRun=d.current;lastStatus=d.vehicle;const v=lastStatus.vehicle;
 el('raw').textContent=JSON.stringify(d,null,2);el('state').textContent=v?.state||'—';el('firmware').textContent='韌體：'+(v?.fw||'未收到')+'；需要 '+d.firmware_required;
 el('connection').textContent=lastStatus.uart_error||'UART 更新距今 '+lastStatus.uart_age_s.toFixed(2)+' 秒';
 el('output').textContent='BMduino PWM（0～255）：'+(v?v.motor_a.pwm_command+' / '+v.motor_b.pwm_command:'—')+(lastRun?'；動力指令 '+lastRun.command.right+' / '+lastRun.command.left+'%':'');
 el('steering').textContent='舵機：'+(v?.steer_output_enabled?v.steer_us_command+' µs（指令）':'輸出關閉');el('encoder').textContent='編碼器右／左：'+(v?v.enc_a.delta+' / '+v.enc_b.delta:'—');
 const fresh=lastStatus.uart_age_s<1&&!lastStatus.uart_error&&lastStatus.camera_age_s<1&&!lastStatus.camera_error;
 el('enable').disabled=active(lastRun)||v?.state!=='READY'||v?.fw!==d.firmware_required||!fresh||!previewFresh()||!lastStatus.servo_settings_synced;
 el('stop').disabled=!active(lastRun);el('recordToggle').disabled=!runId||lastRun?.status!=='running';el('recordToggle').textContent=lastRun?.recording?'停止錄影':'開始錄影';el('recordState').textContent=lastRun?(lastRun.recording?'錄影中 · ':'')+lastRun.id+' · '+lastRun.status+' '+lastRun.reason:'尚未開始';
 const limits=v?.servo_profile;el('limits').textContent=limits?'行程 '+limits.min_us+'～'+limits.max_us+' µs':'等待限位';el('saved').textContent='保存：中心 '+d.defaults.center_us+' µs；右／左 '+d.defaults.right_pct+' / '+d.defaults.left_pct+'%';
 const selected=el('sessionSelect').value;el('sessionSelect').replaceChildren();for(const item of [d.current,...d.history].filter((x,i,a)=>x&&a.findIndex(v=>v?.id===x.id)===i)){const option=document.createElement('option');option.value=item.id;option.textContent=item.created_at+' · '+item.status+' · '+item.id.slice(0,8);el('sessionSelect').append(option)}if(selected)el('sessionSelect').value=selected;if(!dirty)fill(d.defaults);if(runId&&lastRun?.id===runId&&!active(lastRun)){runId=null;keyboardCaptured=false;clearKeys();el('message').textContent='已停止：'+lastRun.reason}
 if(runId&&!fresh)stopDriving('CAMERA_OR_UART_STALE');
 el('driveHud').textContent=lastRun?'動力 '+lastRun.command.right+' / '+lastRun.command.left+'% · 轉向 '+(v?.steer_us_command||'—')+' µs':'動力 0 / 0% · 轉向 —';paintControls();
 el('cameraState').textContent=lastStatus.camera_error||'相機正常，更新距今 '+lastStatus.camera_age_s.toFixed(2)+' 秒';
}catch(e){el('connection').textContent='操作台連線中斷';clearKeys();stopDriving('STATUS_CONNECTION_LOST')}finally{setTimeout(poll,300)}}poll();
let streamController=null,streamEpoch=0,decodeBusy=false,pendingJpeg=null,lastRenderedAt=null,streamStartedAt=0,renderTimes=[],streamError='等待串流';
function previewFresh(){return !streamError&&lastRenderedAt!==null&&performance.now()-lastRenderedAt<1000}
function appendBytes(a,b){const out=new Uint8Array(a.length+b.length);out.set(a);out.set(b,a.length);return out}
function headerEnd(bytes){for(let i=0;i+3<bytes.length;i++)if(bytes[i]===13&&bytes[i+1]===10&&bytes[i+2]===13&&bytes[i+3]===10)return i;return -1}
async function renderLatest(){
 if(decodeBusy)return;decodeBusy=true;
 try{while(pendingJpeg){const packet=pendingJpeg;pendingJpeg=null;const bitmap=await createImageBitmap(new Blob([packet.bytes],{type:'image/jpeg'}));try{if(packet.epoch!==streamEpoch)continue;const canvas=el('camera');if(canvas.width!==bitmap.width||canvas.height!==bitmap.height){canvas.width=bitmap.width;canvas.height=bitmap.height}canvas.getContext('2d').drawImage(bitmap,0,0);lastRenderedAt=performance.now();renderTimes=renderTimes.filter(t=>lastRenderedAt-t<2000);renderTimes.push(lastRenderedAt);streamError=null;}finally{bitmap.close()}}}
 catch(e){streamError='影像解碼失敗';if(runId)stopDriving('PREVIEW_LOST')}
 finally{decodeBusy=false}
}
async function camera(){
 if(document.hidden){setTimeout(camera,500);return}
 const epoch=++streamEpoch,controller=new AbortController();streamController=controller;streamStartedAt=performance.now();
 try{
  const r=await fetch('/camera.mjpg',{cache:'no-store',signal:controller.signal});if(!r.ok||!r.body)throw Error('串流不可用');
  const reader=r.body.getReader(),decoder=new TextDecoder();let buffer=new Uint8Array(0),needed=null;
  while(true){const part=await reader.read();if(part.done)throw Error('串流中斷');buffer=appendBytes(buffer,part.value);
   while(true){if(needed===null){const end=headerEnd(buffer);if(end<0){if(buffer.length>4096)throw Error('串流標頭異常');break}const header=decoder.decode(buffer.slice(0,end)),match=header.match(/Content-Length: ([0-9]+)/i);if(!match)throw Error('串流格式異常');needed=Number(match[1]);if(needed<1||needed>1000000)throw Error('影格大小異常');buffer=buffer.slice(end+4)}
    if(buffer.length<needed)break;pendingJpeg={bytes:buffer.slice(0,needed),epoch};buffer=buffer.slice(needed);needed=null;renderLatest();
   }
  }
 }catch(e){if(epoch===streamEpoch){++streamEpoch;streamError=document.hidden?'頁面已隱藏':'影像串流中斷，重新連線中';pendingJpeg=null;if(runId)stopDriving('PREVIEW_LOST')}}
 finally{controller.abort();setTimeout(camera,1000)}
}
function previewMetrics(){
 const now=performance.now(),age=lastRenderedAt===null?null:(now-lastRenderedAt)/1000;
 const times=renderTimes.filter(t=>now-t<2000),fps=times.length>1&&age<1?(times.length-1)*1000/(times.at(-1)-times[0]):0;
 el('previewMetrics').textContent=(streamError||'串流正常')+' · 顯示 '+fps.toFixed(1)+' FPS · 最新畫面 '+(age===null?'尚未收到':age.toFixed(1)+' 秒前')+' · Orin '+(lastStatus?.preview_fps??'—')+' FPS';
 if(runId&&!previewFresh())stopDriving('PREVIEW_LOST');
 if(streamController&&now-streamStartedAt>1500&&(age===null||age>1.5))streamController.abort();
 setTimeout(previewMetrics,200)
}
camera();previewMetrics();
