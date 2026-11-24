/* Only decides whether an existing preview can be reused; never sends commands. */
(function(root){
 const api={
  reusable(s,d,model,path){return !!s && ['loading','ready','no_reliable_path','camera_unavailable'].includes(s.state) && s.model_id===model && s.path_algorithm===path && (s.curve_outward_px_640??0)===(d?.data?.parameters?.curve_outward_px_640??0) && s.row_ratio===d?.data?.parameters?.row_ratio && (!['straight-connector-v1','straight-bearing-v1','camera-pair-v2','camera-pair-v2-junction','camera-pair-v1','preview-error-v1'].includes(path)||s.connector_center_offset===(d?.data?.parameters?.visual_center_offset_px_640??0));},   // auto-centre-r1 (2026-11-10): the shared centre moves after each run on the car; Orin's PID follows it, so the saved PID centre no longer has to match
  label(s){return s.state==='idle' ? (s.preview_unload_reason==='IDLE_TIMEOUT'?'模型已因閒置卸載；連線正常':s.preview_unload_reason==='USER_STOP'?'模型已手動停止；連線正常':'模型未載入／已卸載；連線正常') : '連線正常';}
 };root.PIDSession=api;if(typeof module!=='undefined')module.exports=api;
})(globalThis);

'use strict';
(()=>{
 let layerSnapshot=null,layerToken=0,hostSessionReady=false,lastDocCheck=0;
 async function drawLiveLayers(){if(!layerSnapshot||!$('livePIDCanvas'))return;const mine=++layerToken;const d=layerSnapshot;await VisionLayers.draw($('livePIDCanvas'),'data:image/jpeg;base64,'+d.raw,d.mask?'data:image/png;base64,'+d.mask:null,d.vision,VisionLayers.options('live'),()=>mine===layerToken&&!!$('livePIDCanvas'));}
 let timer=null,beatTimer=null,busy=false,beatBusy=false,owned=null,seq=0,last=null,savedVisualOffset=null,preparedDoc=null,recovering=false,lastSend=0,lastAck=0,lastRTT=null,maxRTT=0,beatError="",lastRunId=null,runMode='browser_lease';
 const stateLabel={idle:'未載入',loading:'模型載入中',ready:'路徑有效',no_reliable_path:'目前沒有可用路徑',camera_unavailable:'相機影像不可用',failed:'辨識失敗'};
 const stopLabel=r=>({PID_INVALID_OR_STALE_PATH:'路徑無效或影像過期（不是心跳斷線）',PID_BROWSER_LOST:'Orin未及時收到網頁心跳',PID_DURATION_COMPLETE:'本輪時間完成',PID_VISION_STALE:'影像過期',USER_STOP:'已要求停止',PREVIEW_STOPPED:'已停止預覽與試跑',PHYSICAL_ESTOP:'實體急停（預覽保留）',MANUAL_SESSION_ENDED:'車控手動工作階段已結束（預覽保留）',PID_MODEL_FAILURE:'辨識／控制程式例外'})[r]||r||'原因未確認';
 const pathChoices=['camera-pair-v2-junction-flag','camera-pair-v2-junction','camera-pair-v2-junction-crest-full','camera-pair-v2-junction-uphill','camera-pair-v2-junction-crest','camera-pair-v2','camera-pair-v1','preview-error-v1','straight-bearing-v1','standalone-line-v3','straight-connector-v1','standalone-line-v2','standalone-line-v1','junction-k-integrated-r2','junction-k-lane-v3','road-image-v3','road-memory-v3','legacy9','temporal-v2','curve-v1','single-left','single-right','boundary-priority','calibrated-v7'].map(id=>[id,id==='junction-k-lane-v3'?'K 完整路口融合＋A接回 · 共用車道 r1':pathName(id)]);
 const controlCapable=s=>(s?.path_algorithms||[]).some(a=>a.id===chosenPath()&&a.control);
 const chosenPath=()=> $('livePIDPath')?.value||'legacy9';
 function showPathStatus(s){
  const id=chosenPath(),cap=s?.path_algorithms?.find(a=>a.id===id);
  const ability=!s?'尚未取得車端能力':cap?.control?'車端支援即時PID':cap?.preview?'車端僅支援預覽':'車端未回報此算法支援；可至06查看離線功能';
  if($('livePIDPathStatus'))$('livePIDPathStatus').textContent=`${cap?.label||pathName(id)} · ${ability} · 代號 ${id}${s?.latest?.path_diagnostic?.road_state?.predicted ? ' · 橘色虛線＝短暫預測，來源年齡'+Math.round(1000*s.latest.path_diagnostic.road_state.anchor_age_s)+'ms' : ''}${id==='straight-connector-v1' ? (s?.road_state_version===1 ? ' · 道路狀態r1已生效' : s?.exit_release_version===1 ? ' · 出彎收回r1已生效' : ' · 待確認車端版本') : ''}`+(s?.path_algorithm?`；車端目前：${s.path_algorithms?.find(a=>a.id===s.path_algorithm)?.label||pathName(s.path_algorithm)}`:'')+(()=>{const j=s?.latest?.path_diagnostic?.junction;if(s?.path_algorithm!=='camera-pair-v2-junction')return '';if(!j)return '；路口切換版本：預覽載入後顯示';const v=x=>String(x||'?').replace(/^.*-(r[\d.]+)$/,'$1');return `；車端路口模組：切換 ${v(j.switch_revision)}（${j.switch_revision}）· 通道 ${v(j.corridor_revision)} · 整合 ${v(j.revision)}`;})();
 }
 const client=crypto.randomUUID().replaceAll('-','');
 async function command(action,data={}){return api('/api/pid/live/'+action,data);}
 function event(name,extra={}){
  const data={event:name,client_time:Date.now(),run_id:owned||lastRunId||'',...extra};
  fetch('/api/pid/connection-event',{method:'POST',headers:{'Content-Type':'application/json','X-Workbench':'1'},body:JSON.stringify(data),keepalive:true}).catch(()=>{});
 }
 function showBeat(){if($('livePIDHeartbeat'))$('livePIDHeartbeat').textContent=(runMode==='orin_timed'?'Orin限時自主PID · 網頁僅觀測，斷線／失焦不停止本輪':'通道觀測 · '+(owned?'舊版網頁心跳模式':'未啟動本輪'))+` · 通道檢查／回應 ${lastRTT===null?'—':Math.round(lastRTT)+' ms'}${beatError?' · '+beatError:''}`;}
 function leave(reason){event(reason);if(runMode!=='orin_timed')return stopOwned(reason);}
 async function beat(){
  showBeat();if(!owned||beatBusy||runMode==='orin_timed')return;
  const run=owned,n=++seq,start=performance.now(),gap=lastSend?start-lastSend:0;lastSend=start;beatBusy=true;
  const abort=new AbortController(),timeout=setTimeout(()=>abort.abort(),900);
  try{
   const response=await fetch('/api/pid/live/heartbeat',{method:'POST',headers:{'Content-Type':'application/json','X-Workbench':'1'},body:JSON.stringify({owner:client,run_id:run,seq:n}),signal:abort.signal});
   const result=await response.json();if(!response.ok)throw Error(result.error||response.status);
   const rtt=performance.now()-start;
   event('heartbeat_ack',{run_id:run,seq:n,rtt_ms:rtt,send_gap_ms:gap});
   if(owned===run){lastAck=performance.now();lastRTT=rtt;maxRTT=Math.max(maxRTT,rtt);beatError='';}
  }catch(e){
   event('heartbeat_error',{run_id:run,seq:n,send_gap_ms:gap,detail:String(e.message).slice(0,300)});
   if(owned===run){
    owned=null;preparedDoc=null;beatError='心跳未確認，正在核對停止原因；不續跑';if($('livePIDClear'))$('livePIDClear').checked=false;showBeat();
    try{
     const status=await api('/api/pid/live/status');
     if(lastRunId===run&&!owned){
      if(status.run?.id===run&&status.run.state!=='running')beatError='本輪已停止：'+stopLabel(status.run.reason);
      else beatError='心跳回覆未確認；尚未確認實車停止，請確認車況／必要時急停';
      event('heartbeat_error',{run_id:run,seq:n,detail:beatError});notify(beatError);
     }
    }catch{if(!owned){beatError='心跳及狀態均無法確認，請確認車已停止／必要時急停';notify(beatError);}}
   }
  }finally{clearTimeout(timeout);beatBusy=false;showBeat();}
 }
 async function stopOwned(reason='user_stop'){if(owned){event(reason);owned=null;if($('livePIDClear'))$('livePIDClear').checked=false;try{await command('stop');}catch(e){notify('停止未確認：請用實體急停。'+e.message);}}}
 // 影像顯示與狀態輪詢分開：同時只有一個影像請求，依選擇的幀率取下一張。
 let imageWanted=false,layerCapable=false,imageTimer=null,imageBusy=false,imageShown=[],imageUrl=null;
 function imageInterval(){const v=Number($('livePIDFps')?.value??5);return v>0?1000/v:0;}
 function showImageRate(){const n=imageShown.length,el=$('livePIDFpsActual');if(!el)return;el.textContent=n>1?`實收 ${((n-1)*1000/(imageShown[n-1]-imageShown[0])).toFixed(1)} FPS`:imageWanted?'實收 — FPS':'無新影像（未載入模型或影像過舊）';}
 async function pullImage(){
  imageTimer=null;if(!$('livePID')||document.hidden||!imageWanted||imageBusy)return;
  imageBusy=true;const start=performance.now();let ok=false;
  try{
   if(layerCapable){const r=await fetch('/vehicle/api/pid/layers');if(r.ok){ok=true;layerSnapshot=await r.json();if($('livePIDCanvas')&&layerSnapshot.raw){$('livePIDImage').hidden=true;$('livePIDCanvas').hidden=false;await drawLiveLayers();}}}
   else{const r=await fetch('/vehicle/api/pid/frame?t='+Date.now());if(r.ok&&$('livePIDImage')){ok=true;const url=URL.createObjectURL(await r.blob());$('livePIDImage').src=url;if(imageUrl)URL.revokeObjectURL(imageUrl);imageUrl=url;$('livePIDImage').hidden=false;$('livePIDCanvas').hidden=true;}}
   if(ok){const now=performance.now();imageShown.push(now);imageShown=imageShown.filter(t=>t>now-3000);}
  }catch{}finally{imageBusy=false;showImageRate();}
  // 失敗時退後0.5秒，避免斷線時空轉；成功時「不限」＝回覆到就取下一張。
  if($('livePID')&&imageWanted)imageTimer=setTimeout(pullImage,ok?Math.max(0,imageInterval()-(performance.now()-start)):500);
 }
 window.addEventListener('hashchange',()=>{clearInterval(timer);clearInterval(beatTimer);clearTimeout(imageTimer);imageTimer=null;imageWanted=false;leave('route_change');});
 document.addEventListener('visibilitychange',()=>{if(document.hidden){leave('hidden');}});
 window.addEventListener('blur',()=>{leave('blur');});
 window.addEventListener('pagehide',()=>{event('pagehide');if(owned&&runMode!=='orin_timed'){fetch('/api/pid/live/stop',{method:'POST',headers:{'Content-Type':'application/json','X-Workbench':'1'},body:'{}',keepalive:true}).catch(()=>{});owned=null;}});
 async function recover(){
  if(recovering)return;recovering=true;preparedDoc=null;$('livePIDClear').checked=false;$('livePIDStart').disabled=true;
  event('recover');$('livePIDRecovery').textContent='讀取Orin任務與既有模型…（不啟動車輛）';
  try{
   if(owned&&runMode!=='orin_timed')throw Error('仍持有舊版心跳試跑，請先停止');
   const queryStarted=performance.now();
   let current=await api('/api/pid/live/status');
   const probe={rtt_ms:performance.now()-queryStarted};
   if(current.run?.state==='running'){
    if(current.run.execution_mode!=='orin_timed')throw Error('Orin仍在執行舊版心跳試跑，不接管或續跑');
    owned=null;runMode='orin_timed';lastRunId=current.run.id;lastRTT=probe.rtt_ms;beatError='';
    $('livePIDRecovery').textContent='已接回正在執行的Orin任務，只恢復觀測；沒有重新啟動、延長時限或改參數。剩餘約 '+Math.ceil(current.remaining_s??0)+' 秒。';
    event('recovered',{rtt_ms:probe.rtt_ms});await tick();return;
   }
   owned=null;
   if(window.PIDField?.isDirty())throw Error('請先保存或恢復參數草稿');
   const d=await api('/api/document/visual-pid');if(!d.rev)throw Error('請先保存PID參數');
   const centreNote=d.data.parameters.center_us!==current.profile?.center_us?`（車上自動中心 ${current.profile?.center_us} µs，本輪PID依車上中心；保存參數為 ${d.data.parameters.center_us}）`:'';   // auto-centre-r1
   if(chosenPath()!=='legacy9'&&!(current.path_algorithms||[]).some(a=>a.id===chosenPath()&&a.preview))throw Error('此算法尚未部署即時預覽');
   const model=$('livePIDModel').value||'v4-default';
   const reloaded=['idle','failed'].includes(current.state);
   if(reloaded){
    $('livePIDRecovery').textContent=PIDSession.label(current)+'；按「載入模型」才重新載入。';return;
   }
   if(!PIDSession.reusable(current,d,model,chosenPath()))throw Error('模型、算法、取樣位置或中心與保存設定不同，請核對；沒有重新載入');
   preparedDoc=d;lastRTT=probe.rtt_ms;beatError='';lastAck=0;
   $('livePIDRecovery').textContent=(reloaded?'連線已恢復，已要求重新載入停止的模型。':'連線已恢復，沿用原模型，沒有重新載入。')+'等待新鮮有效路徑後，重新確認現場並按開始本輪；不自動續跑。';
   $('livePIDParam').textContent=`已確認保存參數v${d.rev} · PWM ${d.data.parameters.pwm}%`+centreNote;
   event('recovered',{rtt_ms:probe.rtt_ms});await tick();
  }catch(e){$('livePIDRecovery').textContent='恢復未完成：'+e.message;event('recovery_failed',{detail:String(e.message).slice(0,300)});}
  finally{recovering=false;showBeat();}
 }
 async function tick(){
  if(busy||document.hidden||!$('livePID')||Date.now()-(tick.lastPoll||0)<(owned?500:1000))return;busy=true;tick.lastPoll=Date.now();
  try{
   const pollingRun=owned;const s=await api('/api/pid/live/status');if(!$('livePID')||(owned&&pollingRun!==owned))return;last=s;if(s.run?.execution_mode)runMode=s.run.execution_mode;window.PIDField?.observe(s);showBeat();
   if($('livePIDMode'))$('livePIDMode').textContent=s.autonomous_pid_version>=1?'已支援Orin限時自主PID：網頁斷線／失焦不會結束任務，依指定秒數結束；實體急停與Orin故障保護仍有效。':'Orin尚未更新限時自主PID；目前禁止用此新模式開始，待車控更新後使用。';
   $('livePIDState').textContent=`辨識：${stateLabel[s.state]||s.state} · ${s.error||''} · 連續有效${s.ready_frames}張 · 查詢時影格年齡${s.latest?.age_s?.toFixed(2)??'—'}秒（非PID消費延遲） · Orin控制消費年齡${s.latest?.control_age_s?.toFixed(3)??'—'}秒 · 實際取樣${s.latest?.sample_row_ratio===undefined?'—':(s.latest.sample_row_ratio*100).toFixed(1)+'%'} · 推論${s.latest?.inference_ms?.toFixed(0)??'—'}ms · ${s.run?'本輪 '+(s.run.state==='running'?'執行中，剩餘約 '+Math.ceil(s.remaining_s??0)+' 秒':'已停止')+' '+stopLabel(s.run.reason): '尚未試跑'}`;
   if(s.latest?.turn_preview){const h=s.latest.turn_preview;const xy=p=>p?p.map(v=>v.toFixed(1)).join(', '):'—';$('livePIDState').textContent+=` · 提前轉向 ${h.correction_us.toFixed(1)}µs · 方向近點(${xy(h.near_point)})／遠點(${xy(h.far_point)})（640座標） · ${h.reason==='SUPPORTED'?'方向支持':h.reason==='SHORT_CONTINUOUS_PATH'?'遠點不足，僅位置PID':'無位置路徑'}${s.run?.state==='running'?'':' · 靜態預覽非本輪保存PID命令'}`;}
   const straight=s.latest?.path_diagnostic?.line_estimate?.straight_selection;if(straight){$('livePIDState').textContent+=` · 岔口直行：${straight.active?'當張近側直線延伸（'+straight.side+'側）':straight.reason==='NO_CURRENT_STRAIGHT_SUPPORT'?'當張支持不足，回一般循線':straight.route_state==='finished'?'本輪直行已用完，回一般循線':'等待下一入口'} · ${straight.revision}`;}
   if(s.camera_pipeline_version>=1)$('livePIDState').textContent+=' · 控制影像獨立／錄影背景佇列 · 本次辨識最大結果間隔 '+((s.stream_diagnostics?.max_result_gap_s??0)*1000).toFixed(0)+'ms';
   if(s.control_publish_version>=2)$('livePIDState').textContent+=' · 路徑先發布／預覽背景繪圖';
   if(s.latest?.path_diagnostic?.path_revision==='k-parallel-live-r1')$('livePIDState').textContent+=' · 同影格A／K平行版';
   if(owned&&(s.run?.id!==owned||s.run?.state!=='running')){owned=null;$('livePIDClear').checked=false;}

   showPathStatus(s);
   $('livePIDPath').disabled=s.run?.state==='running';
   $('livePIDModel').disabled=['loading','ready','no_reliable_path','camera_unavailable'].includes(s.state)||s.run?.state==='running';
   if(s.models?.length&&!$('livePIDModel').options.length)$('livePIDModel').innerHTML=options(s.models.map(m=>[m.id,m.label]),'v4-default');
   if(!hostSessionReady){
    hostSessionReady=true;
    if(s.model_id && s.state!=='idle' && s.state!=='failed'){
     $('livePIDModel').value=s.model_id;$('livePIDPath').value=s.path_algorithm;
    }
   }
   if(!recovering && !window.PIDField?.isDirty() && Date.now()-lastDocCheck>2000){
    lastDocCheck=Date.now();const d=await api('/api/document/visual-pid');savedVisualOffset=d.data.parameters.visual_center_offset_px_640??0;
    preparedDoc=PIDSession.reusable(s,d,$('livePIDModel').value||'v4-default',chosenPath())?d:null;
    if(preparedDoc)$('livePIDParam').textContent=`已接回既有模型 · 保存參數v${d.rev} · 不重新載入、不自動試跑`;
   }
   $('livePIDStart').disabled=!controlCapable(s)||(chosenPath()==='calibrated-v7'&&!$('livePIDPose')?.checked)||(s.path_algorithm||'legacy9')!==chosenPath()||s.autonomous_pid_version!==1||!!owned||recovering||s.state!=='ready'||s.ready_frames<3||!s.latest||s.run?.state==='running'||!preparedDoc||window.PIDField?.isDirty();
   $('livePIDConnection').textContent=PIDSession.label(s);
   if(document.activeElement!==$('livePIDRetention'))$('livePIDRetention').value=s.preview_retention_s??30;
   $('livePIDRetentionSave').disabled=s.preview_session_version!==1;
   $('livePIDRetentionNote').textContent=s.preview_session_version===1?'只影響無試跑且無觀測時的模型保留；不改變行車停止條件。':'車端仍使用30秒自動卸載；保留時間配套待部署，尚未生效。';
   $('livePIDProfile').textContent=`共用實車中心 ${s.profile.center_us} µs；限位 ${s.profile.min_us}～${s.profile.max_us} µs。每輪結束後車上依中心估計自動微調（最多20 µs），本輪PID依車上中心。視覺中心偏移支援：${s.visual_center_version===1?'已部署':'待部署'}；目前保存偏移 ${savedVisualOffset??'尚未讀取'} px／640寬。此偏移僅在本輪PID控制使用，靜態預覽未套用。彎道外移支援：${s.curve_outward_version===1?'已部署':'待部署'}，預覽外移設定 ${s.curve_outward_px_640??0}px。IMU不參與控制。`;
   imageWanted=!!(s.latest&&s.latest.age_s<2&&s.state!=='idle');layerCapable=!!s.vision_layers_version;if(imageWanted&&!imageTimer&&!imageBusy)pullImage();
  }catch(e){preparedDoc=null;if($('livePIDStart'))$('livePIDStart').disabled=true;if($('livePIDConnection'))$('livePIDConnection').textContent='連線／狀態查詢中斷；模型是否仍載入尚未確認';showPathStatus(null);if($('livePIDState'))$('livePIDState').textContent=(runMode==='orin_timed'?'觀測中斷；Orin可能仍在執行限時任務，請勿重複啟動。網頁停止按鈕在斷線時無法保證送達；必要時按實體急停。':'實車PID介面尚未可用：')+e.message;}finally{busy=false;}
 }
 function mount(){
  if(route!=='drive'||$('livePID')||!$('main').querySelector('.heading'))return;
  hostSessionReady=false;preparedDoc=null;lastDocCheck=0;
  const host=document.createElement('section');host.id='livePID';host.className='card';
  host.innerHTML=`<span class="pill warn">實車PID · 明確啟動才會動 · 不自動續跑</span><h2>即時視覺 PID 單次試跑</h2><p>完整K（共用車道r1）已接入Orin即時PID入口，原A／B與9列算法保留。模型選V4，算法選「K 完整路口融合＋A接回」。K在一般道路使用入口約束A，路口確認且參考吻合才接管，再接回A。紫色為白線遮罩、藍線為K完整算法輸出、黃線為PID取樣列；使用目前保存參數，不自動改PWM、中心或時長。本次尚未啟動馬達，實車試跑由你按下開始。</p><p>先載入模型並查看紫色白線、綠色中央路徑和黃色前視列。只有連續新鮮有效路徑才可開始。辨識模型產生白線遮罩，路徑算法再產生PID追蹤目標，兩者分開選擇。原9列需要雙側白線；失線就停止。</p><div class="formgrid"><label>辨識模型<select id="livePIDModel"></select></label><label>路徑算法<select id="livePIDPath">${options(pathChoices,'legacy9')}</select></label>${field('livePIDSeconds','單次時長 秒（至少1秒）',3,'number','min="1" step="any"')}</div><p id="livePIDPathStatus" role="status">尚未取得車端能力</p><label><input type="checkbox" id="livePIDPose"> v7試跑前確認：相機安裝姿態與v7量測一致，影像為16:9且未裁切</label><p>v7候選使用50～120cm量測範圍；PID近列可先設0.60、遠列0.50，中心維持1360µs。需先保存參數；不自動更改現有設定。</p><details><summary>最新校準結果與路徑限制</summary><div id="livePIDGeometry">讀取校準報告…</div><p>v7筆電候選已接入路徑與視覺PID，是否已部署以車端能力回報為準。中心1360 µs仍以共用保存設定為準。Pure Pursuit是追蹤控制器，不是白線辨識模型；06的歷史A/B/C是既有離線結果，不能直接套到即時相機。</p><a href="#config">02校準與幾何診斷</a> · <a href="#models">06算法比較</a></details><div class="actions"><button id="livePIDPrepare">載入模型／預覽（不動車）</button><button id="livePIDPreviewStop" class="ghost">停止辨識與試跑</button></div><button id="livePIDRecover" class="ghost">嘗試恢復連線（不動車）</button><p id="livePIDRecovery" role="status"></p><p id="livePIDConnection">查詢連線中…</p><label>預覽閒置保留秒數（0＝不自動卸載）<input id="livePIDRetention" type="number" min="0" step="1" value="300"></label><button id="livePIDRetentionSave" disabled>套用保留時間</button><p id="livePIDRetentionNote"></p><p id="livePIDMode" class="ribbon">確認Orin限時自主PID支援…</p><p id="livePIDHeartbeat" role="status"></p><details><summary>心跳與失焦紀錄（筆電保存）</summary><p>顯示瀏覽器往返延遲、傳輸排隊與失焦事件；不是Orin接收間隔的直接量測。恢復連線不續跑。</p><button id="livePIDConnectionLog">讀取最近紀錄</button><div id="livePIDConnectionRows"></div></details><p id="livePIDProfile"></p><p id="livePIDParam"></p><label><input type="checkbox" id="livePIDClear"> 我在車旁、場地清空，可立即按實體急停；了解網頁斷線後本輪仍會繼續至限時或車端保護停止</label><div class="actions"><button id="livePIDStart" disabled>開始本輪實車PID（會動）</button><button id="livePIDStop" class="danger">立即停止PID</button></div><p id="livePIDState" role="status">查詢中…</p>${VisionLayers.controls("live")}<p>新版逐影格遮罩保存需部署後才生效；舊車端僅顯示已合成預覽，圖層不能拆開。</p><p><label>畫面更新<select id="livePIDFps"><option value="1">1 FPS</option><option value="2">2 FPS</option><option value="5" selected>5 FPS</option><option value="10">10 FPS</option><option value="15">15 FPS</option><option value="0">不限（上一張到就取下一張）</option></select></label> <span id="livePIDFpsActual" class="note">實收 — FPS</span><br><small>只影響筆電顯示；同時只取一張，不佔心跳／停止通道。Orin辨識與PID頻率不變，實收受網路限制。</small></p><canvas id="livePIDCanvas" hidden width="640" height="360" style="width:100%;max-width:640px"></canvas><img id="livePIDImage" alt="尚未載入即時模型疊圖" style="max-width:100%;width:640px;border-radius:8px"><p class="note">每輪凍結保存的PID參數。PWM直接輸出到設定值，不做秒級加速。本輪由Orin執行，網頁離開／失焦／斷線不停止。影像過期、失線、急停、車控故障與限時仍停止；Orin內部PID輸入及MCU通訊逾時保護不變。重連只接回觀測，不重啟或延長任務。斷線時網頁停止可能無法送達，現場使用實體急停。圖像預覽依上方選擇的畫面更新（受網路限制），不代表推論或PID頻率。原始錄影、車控與PID輸出保存在同一manual試跑ID，可從05同步回看。</p>`;
  $('main').prepend(host);$('liveLayers').oninput=()=>drawLiveLayers().catch(notify);window.PIDField.mount(host).catch(notify);
  $('livePIDPath').onchange=()=>{$('livePIDPose').checked=false;preparedDoc=null;$('livePIDStart').disabled=true;tick.lastPoll=0;tick();};
  fetch('/experiment/ground-calibration/latest.json').then(r=>{if(!r.ok)throw Error('索引未找到');return r.json();}).then(i=>fetch('/experiment/ground-calibration/'+encodeURIComponent(i.folder)+'/paths-result.json')).then(r=>{if(!r.ok)throw Error('報告未找到');return r.json();}).then(r=>{if(!$('livePIDGeometry'))return;const m=r.metrics;$('livePIDGeometry').textContent=`校準v${r.calibration_revision}：距離以前輪輪胎最前緣為原點；50／80／120cm量測，跑道內寬46±0.5cm。單線左右路徑差平均${m.single_path_separation_mean_cm.toFixed(1)}cm、最大${m.single_path_separation_max_cm.toFixed(1)}cm；投影內寬${m.projected_inner_width_range_cm.map(x=>x.toFixed(1)).join('～')}cm。100cm獨立檢查約+3.6cm偏差。單張直線診斷，未驗證彎道與車輪掃掠。`;}).catch(e=>{if($('livePIDGeometry'))$('livePIDGeometry').textContent='校準報告無法讀取：'+e.message;});
  $('livePIDRecover').onclick=recover;
  $('livePIDRetentionSave').onclick=async()=>{try{const v=Number($('livePIDRetention').value);if(!Number.isInteger(v)||v<0)throw Error('請輸入非負整數');await command('preview-config',{preview_retention_s:v});$('livePIDRetentionNote').textContent='已套用；本次車控服務期間有效，重啟後預設300秒。';}catch(e){notify(e.message);}};
  $('livePIDConnectionLog').onclick=async()=>{try{const rows=await api('/api/pid/connection-log');pagedList('livePIDConnectionRows',rows,r=>`<article class="record-row">${new Date(r.server_time*1000).toLocaleTimeString()} · ${esc(({blur:'視窗失焦',hidden:'分頁隱藏',pagehide:'離開網頁',route_change:'切換分頁',heartbeat_ack:'心跳收到回覆',heartbeat_error:'心跳未確認',recover:'開始檢查連線',recovered:'恢復檢查完成',recovery_failed:'恢復檢查失敗',start_ack:'試跑啟動回覆',user_stop:'使用者停止'})[r.event]||(r.path==='/api/pid/heartbeat'?'心跳傳輸':'獨立通道探測'))} · 本輪 ${esc((r.run_id||'—').slice(0,8))} · 序號 ${r.seq??'—'}<br>往返 ${r.rtt_ms?.toFixed(1)??'—'} ms · 傳輸 ${r.total_ms??'—'} ms · 排隊 ${r.queue_ms??'—'} ms · 送出間隔 ${r.send_gap_ms?.toFixed(1)??'—'} ms · ${esc(r.detail||'')}</article>`);}catch(e){notify(e);}};
  $('livePIDPrepare').onclick=async()=>{try{
   if(window.PIDField?.isDirty())throw Error('請先保存或恢復參數草稿');
   const current=await api('/api/pid/live/status');if(current.run?.state==='running')throw Error('請先結束本輪');
   if(chosenPath()!=='legacy9'&&!(current.path_algorithms||[]).some(a=>a.id===chosenPath()&&a.preview))throw Error('此算法尚未部署即時預覽；請在02／06查看離線結果');
   const d=await api('/api/document/visual-pid');if(!d.rev)throw Error('請先保存PID參數');preparedDoc=null;
   if(d.data.parameters.curve_outward_px_640 && current.curve_outward_version!==1)throw Error('彎道外移尚未部署Orin；不停止現有預覽');
   if(d.data.parameters.curve_outward_px_640 && !['junction-k-lane-v3','junction-k-integrated-r2'].includes(chosenPath()))throw Error('彎道外移目前支援K路徑；其他算法請設0');
   if(PIDSession.reusable(current,d,$('livePIDModel').value||'v4-default',chosenPath())){preparedDoc=d;$('livePIDRecovery').textContent='沿用既有模型，已核對保存參數，沒有重新載入。';await tick();return;}
   if(current.state!=='idle'){await command('preview-stop');for(let i=0;i<20;i++){await new Promise(r=>setTimeout(r,250));if((await api('/api/pid/live/status')).state==='idle')break;}}
   await command('prepare',{model_id:$('livePIDModel').value||'v4-default',path_algorithm:chosenPath(),row_ratio:d.data.parameters.row_ratio,curve_outward_px_640:d.data.parameters.curve_outward_px_640??0,visual_center_offset_px_640:d.data.parameters.visual_center_offset_px_640??0});preparedDoc=d;
   $('livePIDParam').textContent=`準備v${d.rev}：PWM ${d.data.parameters.pwm}% · P/I/D ${d.data.parameters.kp}/${d.data.parameters.ki}/${d.data.parameters.kd}`;await tick();
  }catch(e){notify(e);}};
  LastInputs.bind('live-pid-options',['livePIDModel','livePIDPath','livePIDSeconds','livePIDFps']);
  $('livePIDFps').onchange=()=>{clearTimeout(imageTimer);imageTimer=null;imageShown=[];showImageRate();if(imageWanted&&!imageBusy)pullImage();};
  $('livePIDPath').addEventListener('change',()=>showPathStatus(last));
  showPathStatus(last);
  $('livePIDStart').onclick=async()=>{const b=$('livePIDStart');b.disabled=true;try{if(!controlCapable(last)||(last?.path_algorithm||'legacy9')!==chosenPath())throw Error('此算法目前僅可預覽／離線比較');if(window.PIDField?.isDirty())throw Error('請先保存或恢復參數草稿');if(!$('livePIDClear').checked)throw Error('請先確認現場與斷線仍繼續的限時模式');if(last?.autonomous_pid_version!==1)throw Error('Orin尚未支援限時自主PID');const d=await api('/api/document/visual-pid');if(!preparedDoc||d.rev!==preparedDoc.rev)throw Error('PID參數待核對，請按恢復連線；不必先停止模型');const seconds=Number($('livePIDSeconds').value);if(!Number.isFinite(seconds)||seconds<1)throw Error('時長須為至少1秒的有限數值');if(d.data.parameters.curve_outward_px_640 && last?.curve_outward_version!==1)throw Error('彎道外移尚未部署Orin');if(d.data.parameters.visual_center_offset_px_640 && last?.visual_center_version!==1)throw Error('視覺中心修正尚未部署Orin；請先完成部署');if(!last?.custom_duration_version&&seconds>20)throw Error('目前Orin仍為20秒上限版本，需部署新版車控後才可使用自訂長時長');const startSent=performance.now();const result=await command('start',{owner:client,path_algorithm:chosenPath(),camera_pose_confirmed:$('livePIDPose').checked,calibration_sha256:last?.latest?.path_diagnostic?.calibration_sha256,seconds:Number($('livePIDSeconds').value),area_clear:true,parameter_revision:d.rev,execution_mode:'orin_timed'});lastRTT=performance.now()-startSent;runMode=result.execution_mode||'browser_lease';owned=result.run_id;lastRunId=owned;seq=0;lastSend=0;lastAck=0;maxRTT=0;beatError='';event('start_ack');if(!host.isConnected||document.hidden||!document.hasFocus()||route!=='drive'){if(runMode!=='orin_timed')await stopOwned();return;}$('livePIDParam').textContent=`本輪${owned} · 固定參數v${d.rev} · PWM ${d.data.parameters.pwm}%`;beat();await tick();}catch(e){preparedDoc=null;notify('開始未完成或回覆未確認：'+e.message+'；請先按恢復連線查詢，勿重複開始。');}finally{if(b.isConnected)b.disabled=true;tick.lastPoll=0;}};
  $('livePIDStop').onclick=async()=>{event('user_stop');owned=null;try{await command('stop');await tick();}catch(e){notify('停止未確認，請按實體急停：'+e.message);}};
  $('livePIDPreviewStop').onclick=async()=>{owned=null;try{await command('preview-stop');await tick();}catch(e){notify(e);}};
  clearInterval(timer);clearInterval(beatTimer);beatTimer=setInterval(beat,250);timer=setInterval(tick,250);tick();showBeat();
 }
 window.addEventListener('pid-parameters-saved',()=>{preparedDoc=null;if($('livePIDStart'))$('livePIDStart').disabled=true;});
 new MutationObserver(mount).observe($('main'),{childList:true});mount();
})();
