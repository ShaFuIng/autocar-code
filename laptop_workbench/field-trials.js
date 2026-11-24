'use strict';
// Versioned trials; actions below target laptop records only.
let fieldTrialTimer=null,fieldTrialMounting=false,fieldTrialRuns=[],fieldTrialLoaded=[null,null];
function fieldTrialPanel(){return `<section class="card" id="fieldTrialPanel"><h2>單輪試驗與版本紀錄</h2><p class="ribbon">本版：合成路徑影子計算／現場唯讀紀錄。<b>不會啟用舵機或馬達</b>。限時只結束紀錄，不停止舊介面的手動駕駛；本區不執行自動循跡；即時PID另見04「即時視覺 PID 單次試跑」。</p>
<details id="trialCreate" ${route!=='replay'?'open':''}><summary>建立一輪（參數快照與時間線）</summary><div class="formgrid"><label>路段／目的<input id="trialSegment" value="直線與單彎診斷" maxlength="100"></label><label>模式<select id="trialMode"><option value="synthetic">合成路徑影子測試（不連Orin）</option><option value="observe">現場唯讀紀錄（狀態＋抽樣影格）</option></select></label><label>紀錄秒數（1～60）<input id="trialSeconds" type="number" value="3" min="1" max="60"></label><label>合成情境<select id="trialScenario"><option value="straight">直線</option><option value="left">左彎</option><option value="right">右彎</option><option value="dropout">中途掉路徑</option></select></label></div><p id="trialRevision"></p><label><input type="checkbox" id="trialRecordOnly"> 我了解「唯讀紀錄」不控制車子，結束紀錄不會停止手動駕駛。</label><div class="actions"><button id="trialBegin">建立並開始一輪紀錄</button><button id="trialEnd" class="secondary">結束目前紀錄（不送車控）</button><button id="trialRefresh" class="ghost">重新讀取紀錄</button><a href="#models">到06編輯下一輪提案</a></div><p>合成路徑假定後軸座標與外前輪幾何，不是影片辨識結果、不是閉環跑車成績。轉向查表在未量中心區／超出能力時輸出無效及零PWM提案，所有計算沒有硬體輸出。唯讀影格為至多2Hz快照，非原畫質連續錄影；完整錄影仍使用既有純錄影功能。</p></details>
<p id="trialMessage" role="status"></p><div id="trialList"></div>
<details id="trialPlayback" ${route==='replay'?'open':''}><summary>回放比較／時間點評語</summary><p>以兩輪各自開始的相對時間對齊；這不是自動辨識相同地點。可填B的時間偏移，以現場標記對齊。影格曝光時間未知，顯示請求到回覆的時間窗，不能宣稱影像與遙測精確同步。</p><div class="formgrid"><label>A紀錄<select id="trialA"></select></label><label>B紀錄<select id="trialB"></select></label><label>B偏移秒（B時間＝A時間＋偏移）<input id="trialOffset" type="number" value="0" min="-60" max="60" step="0.1"></label></div><button id="trialCompare">載入比較</button><label>相對時間 <input id="trialTime" type="range" min="0" max="60" value="0" step="0.1"></label><div id="trialCompareSummary"></div><div class="grid"><div id="trialViewA"></div><div id="trialViewB"></div></div><label>對A這個時間點追加觀察（偏行、壓線、人工介入等）<textarea id="trialNote" maxlength="2000"></textarea></label><button id="trialReview">保存時間點評語</button><div id="trialReviews"></div></details></section>`;}
function trialSample(d,t){return d?.events.filter(e=>e.t<=t).at(-1)||null;}
function drawTrial(d,t,host){
 if(!d){host.innerHTML='<p>尚未載入</p>';return;}
 const e=trialSample(d,t);if(!e||t<0||t>d.elapsed_s){host.innerHTML='<p>此時間沒有資料，不補零、不沿用超出紀錄的畫面。</p>';return;}
 const v=e.telemetry?.vehicle,p=e.proposal;
 host.innerHTML=`<h3>${esc(d.segment)} · ${(d.mode==='synthetic'?'合成影子':'現場唯讀')}</h3><p>v${d.parameter_snapshot.rev} · ${e.t.toFixed(2)}秒／選取${t.toFixed(2)}秒</p>${e.image?`<img alt="本輪抽樣影格" style="width:100%" src="/field-image/${d.id}/${e.image}"><p>影像請求／收到：${e.image_request_s.toFixed(3)}～${e.image_received_s.toFixed(3)}秒；遙測收到${e.status_received_s.toFixed(3)}秒，曝光時間未知。</p>`:'<canvas width="420" height="220" style="width:100%;background:#edf3f0" aria-label="合成路徑和前視點"></canvas>'}<pre style="max-height:190px;overflow:auto">${esc(JSON.stringify(p?{reason:p.reason,valid:p.valid,curvature:p.curvature_1pm,desired_us:p.desired_us,limited_us:p.proposal_us,pwm_proposal:e.proposal_pwm,hardware_output:false}:{state:v?.state,steer_us:v?.steer_us_command,right_pwm:v?.motor_a?.pwm_command,left_pwm:v?.motor_b?.pwm_command,encoder_right:v?.enc_a?.delta,encoder_left:v?.enc_b?.delta},null,2))}</pre>`;
 const canvas=host.querySelector('canvas');if(canvas){const c=canvas.getContext('2d');c.strokeStyle='#286e68';c.beginPath();for(const [i,[x,y]] of (e.path||[]).entries()){const a=210-y*90,b=210-x*85;i?c.lineTo(a,b):c.moveTo(a,b);}c.stroke();c.fillStyle='#555';c.fillRect(202,202,16,8);if(p?.target){c.fillStyle='#c77b24';c.beginPath();c.arc(210-p.target[1]*90,210-p.target[0]*85,5,0,7);c.fill();}c.fillStyle='#222';c.fillText('合成後軸座標：綠線路徑／橘點前視；不是實車軌跡',8,15);}
}
function renderTrialCompare(){
 const t=Number($('trialTime').value),offset=Number($('trialOffset').value);
 drawTrial(fieldTrialLoaded[0],t,$('trialViewA'));drawTrial(fieldTrialLoaded[1],t+offset,$('trialViewB'));
 const a=fieldTrialLoaded[0];if(a)pagedList('trialReviews',a.reviews,r=>`<article class="record-row">${r.t.toFixed(1)}秒 · ${esc(r.note)}</article>`);
}
async function refreshFieldTrials(){
 if(!$('trialList')||document.hidden)return;
 try{const [runs,proposal]=await Promise.all([api('/api/field-trials'),api('/api/document/tracking-proposal')]);if(!$('trialList'))return;fieldTrialRuns=runs;$('trialRevision').textContent=`下一輪凍結提案 v${proposal.rev}；修改提案不會改寫已建立的紀錄。`;$('trialBegin').dataset.revision=proposal.rev;
 pagedList('trialList',runs,r=>`<article class="record-row"><b>${esc(r.segment)}</b> · ${(r.mode==='synthetic'?'合成影子':'現場唯讀')} · v${r.parameter_revision} · ${esc(r.status)} · ${r.samples??0}筆 <small>${esc(r.reason||'記錄中')}</small><br>${esc(r.id)}</article>`);
 for(const id of ['trialA','trialB']){const old=$(id).value;$(id).innerHTML='<option value="">不選</option>'+runs.map(r=>`<option value="${r.id}">${esc(r.segment)} · v${r.parameter_revision} · ${r.id.slice(0,6)}</option>`).join('');if(runs.some(r=>r.id===old))$(id).value=old;}
 $('trialBegin').disabled=runs.some(r=>r.status==='recording');$('trialEnd').disabled=!runs.some(r=>r.status==='recording');
 clearTimeout(fieldTrialTimer);if(runs.some(r=>r.status==='recording'))fieldTrialTimer=setTimeout(refreshFieldTrials,1500);
 }catch(e){if($('trialMessage'))$('trialMessage').textContent=e.message;}
}
async function bindFieldTrials(){
 await refreshFieldTrials();if(!$('trialBegin'))return;LastInputs.bind('trial-create:'+route,['trialSegment','trialSeconds','trialScenario']);LastInputs.bind('trial-view',['trialA','trialB']);
 $('trialRefresh').onclick=refreshFieldTrials;
 $('trialBegin').onclick=async()=>{const b=$('trialBegin');b.disabled=true;try{const request_id=crypto.randomUUID().replaceAll('-','');const result=await api('/api/field-trial/start',{request_id,revision:Number(b.dataset.revision),mode:$('trialMode').value,seconds:Number($('trialSeconds').value),scenario:$('trialScenario').value,segment:$('trialSegment').value,record_only_confirmed:$('trialRecordOnly').checked});$('trialMessage').textContent='本輪 '+result.id+' 已建立；沒有送出硬體指令。';await refreshFieldTrials();}catch(e){$('trialMessage').textContent='未確認開始：'+e.message+'。先重新讀取紀錄，不自動重送。';await refreshFieldTrials();}};
 $('trialEnd').onclick=async()=>{try{const r=fieldTrialRuns.find(r=>r.status==='recording');if(r)await api('/api/field-trial/stop',{id:r.id});$('trialMessage').textContent='已請求結束紀錄，沒有送車輛停止命令。';await refreshFieldTrials();}catch(e){$('trialMessage').textContent=e.message;}};
 $('trialCompare').onclick=async()=>{try{fieldTrialLoaded=await Promise.all(['trialA','trialB'].map(id=>$(id).value?api('/api/field-trial/'+$(id).value):null));const [a,b]=fieldTrialLoaded;$('trialOffset').value=0;$('trialNote').value='';LastInputs.bind('trial-pair:'+a?.id+':'+b?.id,['trialOffset','trialNote'],document,true);const ap=a?.parameter_snapshot.data.parameters||{},bp=b?.parameter_snapshot.data.parameters||{};const diff=Object.keys({...ap,...bp}).filter(k=>ap[k]!==bp[k]).map(k=>`${k}: ${ap[k]??'未知'} → ${bp[k]??'未知'}`);$('trialCompareSummary').textContent='參數差異：'+(diff.join('；')||'無／未選兩輪')+'。合成情境僅驗證控制提案，不代表車輛軌跡或圈速改善。';$('trialTime').max=Math.max(1,a?.elapsed_s||b?.elapsed_s||60);$('trialTime').value=Math.min(.2,Number($('trialTime').max));renderTrialCompare();}catch(e){$('trialMessage').textContent=e.message;}};
 $('trialTime').oninput=renderTrialCompare;$('trialOffset').oninput=renderTrialCompare;
 $('trialReview').onclick=async()=>{try{const a=fieldTrialLoaded[0];if(!a)throw Error('先載入A紀錄');await api('/api/field-trial/review',{id:a.id,t:Number($('trialTime').value),note:$('trialNote').value});fieldTrialLoaded[0]=await api('/api/field-trial/'+a.id);$('trialNote').value='';renderTrialCompare();$('trialMessage').textContent='評語已追加保存，原紀錄保留。';}catch(e){$('trialMessage').textContent=e.message;}};
}
function tomorrowPanel(){return `<section class="card" id="tomorrowPlan"><h2>明天試跑計畫與本版界線</h2><p>目前可用：參數提案版本、單輪合成PP影子、唯讀抽樣紀錄、兩輪時間線比較與時間點評語。<b>尚未開放自動循跡實車輸出</b>；不要將紀錄按鈕當作車輛啟停。</p><ol><li>到場先確認無線、相機與資料新鮮度、供電與實體急停；不憑昨日狀態判定今日可用。</li><li>確認鏡頭前伸10cm的前軸基準、前輪直徑、地面投影尺度；用幾個已知距離點核對。單側白線與盲區不能標成真值。</li><li>靜止做唯讀紀錄，核對影格／遙測時間窗。抽樣快照不是完整錄影，必要時另外用原純錄影，保留其來源ID。</li><li>人工低速直線／單彎，03或04既有單次控制需獨立授權。紀錄凍結提案不等於手動設定已套用，要核對實際指令。結束由實車停止操作確認。</li><li>05以相同路段標記對齊兩輪，記壓線、偏行與介入；06改一組參數另存版本，再測同段。</li><li>啟用自動循跡前：確認實測曲率映射與中心未知區、完整地面路徑、影像／通訊失效停止、控制權交接、實車限時停止及輪胎掃掠餘裕。未通過就不做自動整圈或加速。</li></ol><details><summary>網站負擔與未解決限制</summary><p>本機靜態回應抽測：03介面約28.2 KiB、04約36.8 KiB，當次回應約14～17ms（不含SSH、圖像、WebSocket或Orin推論）。依程式計時，03閒置狀態與馬達紀錄各約1次／秒，相機預設0；04閒置狀態約1次／秒，啟用後維持約3.3次／秒；實際頻率仍受請求耗時影響。控制心跳未修改，Orin實際CPU與無線負載尚未量測。</p><ul><li>03／04實車iframe按需載入，切頁卸載；未啟用時降低狀態輪詢，相機預設關閉，選擇1／2／5／10FPS。活動控制心跳不降低。</li><li>06舊實驗報告改按需載入；表格／歷史固定高度與預設10筆。比較圖只讀選取時間，不循環拉取全部影片。</li><li>低頻唯讀擷取受SSH延遲影響，未做精確曝光同步；不能據此測毫秒級控制延遲。後端讀取逾時可能讓紀錄結束晚於設定時間，此處不是車控限時器。</li><li>原8000／8091服務仍為必要後端。本版不改動原服務、不燒錄；本機UI改善要重新開啟iframe才生效。</li><li>仍需現場量：停止距離、負載下轉向速率、反向回差、高PWM性能。影子模式用明示幾何假設，不當成完成標定。</li></ul></details></section>`;}
function mountFieldTrials(){
 if(fieldTrialMounting)return;
 
 if(!['drive','replay','models'].includes(route)||$('fieldTrialPanel')||!$('main').querySelector('h1')||window.WBArchive?.hold('fieldTrialPanel'))return;
 fieldTrialMounting=true;$('main').insertAdjacentHTML('afterbegin',fieldTrialPanel());bindFieldTrials().then(()=>{
 if(!$('fieldTrialPanel'))return;
 $('trialCreate').hidden=route==='replay';$('trialPlayback').hidden=route!=='replay';$('trialList').hidden=route!=='replay';
 if(route!=='replay'){
  const mode=route==='drive'?'observe':'synthetic';$('trialMode').value=mode;
  [...$('trialMode').options].forEach(o=>{o.disabled=o.value!==mode;});
  $('trialMode').closest('label').hidden=true;
  $('trialScenario').closest('label').hidden=mode==='observe';
  $('trialRecordOnly').closest('label').hidden=mode==='synthetic';
  $('fieldTrialPanel').querySelector('h2').textContent=mode==='observe'?'現場唯讀抽樣紀錄':'合成路徑研究實驗';
  $('fieldTrialPanel').insertAdjacentHTML('beforeend','<a href="#replay">到05回放與比較所有試驗</a>');
 }else $('fieldTrialPanel').querySelector('h2').textContent='抽樣／合成試驗：時間線比較';
 }).finally(()=>{fieldTrialMounting=false;mountFieldTrials();});
}
new MutationObserver(mountFieldTrials).observe($('main'),{childList:true});mountFieldTrials();
window.addEventListener('hashchange',()=>{clearTimeout(fieldTrialTimer);fieldTrialLoaded=[null,null];});
