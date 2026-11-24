'use strict';
// Same registry, sources and append-only reviews as the existing analysis page.
const shadowModelsBase=modelsPage;
modelsPage=async()=>{
 await shadowModelsBase();if(route!=='models'||!$('analysisSetup'))return;
 const [catalog,pid,history]=await Promise.all([api('/api/analysis'),api('/api/document/visual-pid'),api('/api/history/visual-pid')]);
 if(route!=='models'||!$('analysisSetup'))return;
 const card=$('analysisSetup').closest('section.card');
 card.querySelector('h2').textContent='離線影子測試：共用辨識 → 路徑比較 → PID推演';
 const legacy=document.createElement('details');legacy.innerHTML='<summary>既有多模型 A／B／C 分析（保留）</summary>';
 card.insertBefore(legacy,$('analysisSetup'));legacy.append($('analysisSetup'),$('analysisStart'));
 const host=document.createElement('div');host.id='shadowSetup';card.insertBefore(host,legacy);
 const geometry=catalog.jobs.find(j=>j.spec?.groups?.some(g=>g.algorithm==='geometry-mismatch-offline-v1'));
 if(geometry){const box=document.createElement('div');box.innerHTML='<h3>2:40後：幾何失配離線比較</h3><p>原共用車道K／候選，同一V4遮罩。單側寬度不可核對與雙側幾何失配分開標示；未部署、PID關閉。</p><button id="geometryCompare">開啟修改前後遮罩與路徑</button>';card.insertBefore(box,host);$('geometryCompare').onclick=()=>loadAnalysis(geometry.id).catch(notify);}
 const prepared=catalog.jobs.find(j=>j.source==='shadow-k-video003-000-256'&&!j.spec?.groups?.some(g=>g.algorithm==='geometry-mismatch-offline-v1'));
 if(prepared){const note=document.createElement('div');note.innerHTML='<p><b>新測試片段：video_003 0:00～2:56 · A／K各版本比較 · PID關閉</b></p><button id="shadowPreparedClip">'+(prepared.state==='completed'?'開啟新片段比較結果':'新片段分析中，可查看已完成部分')+'</button>';if(prepared.validation_summary){const v=prepared.validation_summary;note.insertAdjacentHTML('beforeend',`<p><strong>長片結果：本版K空路徑 ${v.k_empty}/${v.frames}；原A空路徑 ${v.a_empty}/${v.frames}${v.original_k_empty===undefined?'':`；原K空路徑 ${v.original_k_empty}/${v.frames}`}。路徑是否正確仍需核對，空路徑數不是準確率。</strong></p>`);}card.insertBefore(note,host);$('shadowPreparedClip').onclick=()=>loadAnalysis(prepared.id).catch(notify);}
 const revisions=[pid,...history.map(h=>({rev:h.rev,data:h.data}))].filter((x,i,a)=>x.rev&&a.findIndex(y=>y.rev===x.rev)===i);
 const models=catalog.models.filter(m=>m.available&&['v4-640','v4-960'].includes(m.id));
 let groups=['legacy9','calibrated-v7','calibrated-v7'].map((algorithm,i)=>({label:String.fromCharCode(65+i),model:'v4-640',algorithm,lane_width_cm:46,matrix:'',pid_revision:pid.rev,pid_enabled:true}));
 const draftKey='offline-shadow-draft-v1';
 let draft;try{draft=JSON.parse(localStorage.getItem(draftKey))}catch{}
 if(!draft){const previous=catalog.jobs.filter(j=>j.kind==='shadow'&&j.spec?.groups?.length).sort((a,b)=>(b.created||0)-(a.created||0))[0];if(previous){draft={...previous.spec,groups:previous.spec.groups.map(g=>({...g,matrix:g.matrix?JSON.stringify(g.matrix):''}))};}}
 const incoming=sessionStorage.getItem('pidCompareSource');if(incoming){draft={...(draft||{}),source:incoming,start:0,end:Math.max(0,(catalog.sources.find(s=>s.id===incoming)?.frames||1)-1)};sessionStorage.removeItem('pidCompareSource');}
 if(draft?.groups)groups=draft.groups;
 host.innerHTML=`<p>只分析本機已同步影片，不連Orin。相同V4設定共用遮罩；改校正／算法只重算路徑，改PID版本只回算PID。PID參數只從04保存版本讀取，不另設編輯來源。</p>
 ${select('shadowSource','影片',catalog.sources.map(s=>[s.id,`${s.title} · ${s.frames}影格`]),draft?.source)}
 <img id="shadowPreview" alt="所選影片原始影格" style="max-width:480px;width:100%">
 <p id="shadowSourceInfo"></p><input id="shadowPreviewFrame" type="range" min="0" value="0" aria-label="來源預覽影格">
 <div class="formgrid">${field('shadowFrom','分析起始影格（含）',draft?.start??0,'number')}${field('shadowTo','分析結束影格（含）',draft?.end??0,'number')}${field('shadowStride','每隔幾張取一張（1＝逐張）',draft?.stride??1,'number')}
 ${select('shadowPose','與校正相機姿態',['unknown','matched','different'].map((v,i)=>[v,['未知：公分結果僅探索','已確認相符','不同：公分結果僅探索'][i]]),draft?.pose||'unknown')}</div>
 <p>K · 路口融合＋A雙線接回：採用你確認的K流程，直行路口使用影像融合，離開後交回A；不使用v7校正。已比對10fps原兩段，抽幀會改變確認所需時間；新影片仍需回看。</p><p>區間起點冷啟動，暖機列保留。抽幀仍使用來源時間，可能觸發既有max_gap。影片為已同步解碼影格，不是重新拍攝。</p>
 <div id="shadowGroups" class="grid three"></div><div class="actions"><button id="shadowAdd">複製最後一組</button><button id="shadowRun">建立影子比較</button></div>
 <p id="shadowPlan" role="status"></p>`;
 function collect(){return {source:$('shadowSource').value,start:Number($('shadowFrom').value),end:Number($('shadowTo').value),stride:Number($('shadowStride').value),pose:$('shadowPose').value,groups};}
 function save(){const d=collect();localStorage.setItem(draftKey,JSON.stringify(d));LastInputs.put('shadow-interval:'+d.source,{shadowFrom:String(d.start),shadowTo:String(d.end),shadowStride:String(d.stride),shadowPose:d.pose});plan();}
 function plan(){$('shadowPlan').textContent=`${new Set(groups.map(g=>g.model)).size}種V4設定（相同設定只辨識一次／可沿用快取） → ${groups.length}組路徑 → ${groups.filter(g=>g.pid_enabled).length}組PID。保存PID目前v${pid.rev}，中心${pid.data.parameters?.center_us??'未保存'}µs。`;}
 function drawGroups(){
  $('shadowGroups').innerHTML=groups.map((g,i)=>`<article data-group="${i}"><h3>${String.fromCharCode(65+i)} 比較組</h3>
   <label>名稱<input data-key="label" value="${esc(g.label)}"></label>
   <label>V4設定<select data-key="model">${models.map(m=>`<option ${m.id===g.model?'selected':''}>${esc(m.id)}</option>`).join('')}</select></label>
   <label>路徑<select data-key="algorithm">${['legacy9','temporal-v2','calibrated-v7','entrance-image-v1','entrance-temporal-v1','entrance-v7-v1','entrance-image-v2','entrance-temporal-v2','entrance-v7-v2','road-image-v3','road-memory-v3','road-memory-v3.1','junction-straight-v1','junction-k-v1','junction-k-mask-v1','road-entrance-v3.2','junction-k-entrance-v2','junction-k-lane-v3','road-v7-v3'].map(a=>`<option value="${a}" ${a===g.algorithm?'selected':''}>${esc(pathName(a))}</option>`).join('')}</select></label>
   ${['calibrated-v7','entrance-v7-v1','entrance-v7-v2','road-v7-v3'].includes(g.algorithm)?`<label>淨寬cm<input data-key="lane_width_cm" type="number" step="any" value="${g.lane_width_cm}"></label><details><summary>投影校正候選</summary><p>空白沿用v7。可貼3×3矩陣JSON，僅凍結於本輪，不覆寫02對位或v7原檔；座標遵守v7的640×360與前輪參考；50～120cm外，新入口候選保留為投影探索，原v7仍限制該區間。</p><textarea data-key="matrix" placeholder="留空使用v7">${esc(g.matrix||'')}</textarea></details>`:'<p>此算法不使用v7校正；不顯示無效校正設定。</p>'}
   <p>用途：離線推演 · 算法代號：${esc(g.algorithm)}</p>
   <label>唯讀PID版本<select data-key="pid_revision">${revisions.map(d=>`<option value="${d.rev}" ${Number(g.pid_revision)===d.rev?'selected':''}>v${d.rev} · 中心${d.data.parameters?.center_us} · 近列${d.data.parameters?.row_ratio}</option>`).join('')}</select></label>
   <label><input type="checkbox" data-key="pid_enabled" ${g.pid_enabled?'checked':''}> PID推演</label>
   <button class="ghost" data-remove="${i}" ${groups.length===1?'disabled':''}>移除此組</button></article>`).join('');
  $('shadowGroups').onchange=e=>{const key=e.target.dataset.key;if(!key)return;const g=groups[Number(e.target.closest('[data-group]').dataset.group)];g[key]=key==='pid_enabled'?e.target.checked:['pid_revision','lane_width_cm'].includes(key)?Number(e.target.value):e.target.value;save();if(key==='algorithm')drawGroups();};
  $('shadowGroups').oninput=e=>{if(['label','matrix','lane_width_cm'].includes(e.target.dataset.key))$('shadowGroups').onchange(e);};
  $('shadowGroups').onclick=e=>{if(e.target.dataset.remove!==undefined){groups.splice(Number(e.target.dataset.remove),1);drawGroups();save();}};
  $('shadowAdd').disabled=groups.length>=3;plan();
 }
 host.addEventListener('load-shadow-parameters',e=>{
  try{
   const job=e.detail,spec=job.spec;
   const availableAlgorithms=[...$('shadowGroups').querySelector('[data-key="algorithm"]').options].map(o=>o.value);
   if(spec.groups.some(g=>!models.some(m=>m.id===g.model)||!availableAlgorithms.includes(g.algorithm)))throw Error('此紀錄含目前未提供的模型或算法，未變更設定。');
   const loaded=spec.groups.map(g=>({label:g.label,model:g.model,algorithm:g.algorithm,
    lane_width_cm:g.calibration?.lane_width_cm??46,matrix:g.calibration?.matrix?JSON.stringify(g.calibration.matrix):'',
    pid_revision:g.pid_revision,pid_enabled:g.pid_enabled}));
   for(const g of spec.groups)if(!revisions.some(r=>Number(r.rev)===Number(g.pid_revision)))revisions.push({rev:g.pid_revision,data:{parameters:g.parameters}});
   groups=loaded;$('shadowStride').value=spec.stride??1;
   drawGroups();save();
   $('shadowPlan').textContent='已讀取「'+job.title+'」的比較組、校正矩陣、PID版本與抽幀設定；目前影片、起訖影格及相機姿態標記保留。尚未開始分析。';
   host.scrollIntoView({behavior:'smooth',block:'start'});
  }catch(error){notify(error);}
 });
 let sourceRows=[],previewToken=0;
 async function sourceChanged(reset){
  const source=$('shadowSource').value,token=++previewToken;
  const r=await api('/api/replay/'+source);if(token!==previewToken||!host.isConnected)return;sourceRows=r.rows;
  $('shadowPreviewFrame').max=sourceRows.length-1;$('shadowPreviewFrame').value=0;
  if(reset){$('shadowFrom').value=0;$('shadowTo').value=sourceRows.length-1;}for(const id of ['shadowFrom','shadowTo']){$(id).min=0;$(id).max=sourceRows.length-1;}LastInputs.bind('shadow-interval:'+source,['shadowFrom','shadowTo','shadowStride','shadowPose']);
  const t=sourceRows.at(-1)?.capture_s-sourceRows[0]?.capture_s;
  $('shadowSourceInfo').textContent=`${sourceRows.length}影格 · 首尾時間差${Number(t).toFixed(3)}秒 · 原始時間戳保留`;
  preview();save();
 }
 function preview(){const i=Number($('shadowPreviewFrame').value),r=sourceRows[i];if(r)$('shadowPreview').src='/asset/'+$('shadowSource').value+'/'+String(r.frame??i).padStart(6,'0')+'.jpg';}
 $('shadowPreviewFrame').oninput=preview;
 $('shadowSource').onchange=()=>sourceChanged(true).catch(notify);
 for(const id of ['shadowFrom','shadowTo','shadowStride','shadowPose']){$(id).onchange=save;$(id).oninput=save;}
 $('shadowAdd').onclick=()=>{groups.push(JSON.parse(JSON.stringify(groups.at(-1))));groups.at(-1).label=String.fromCharCode(64+groups.length);drawGroups();save();};
 $('shadowRun').onclick=async()=>{
  const b=$('shadowRun');b.disabled=true;
  try{save();const p=collect();p.groups=p.groups.map(g=>({...g,matrix:['calibrated-v7','entrance-v7-v1','entrance-v7-v2','road-v7-v3'].includes(g.algorithm)&&g.matrix.trim()?JSON.parse(g.matrix):null}));
   const job=await api('/api/analysis/start',{...p,kind:'shadow',models:[...new Set(groups.map(g=>g.model))]});
   $('shadowPlan').textContent='已建立 '+job.id+'；離開頁面仍繼續，重新整理不會重送。';updateAnalysisJobs((await api('/api/analysis')).jobs);
  }catch(e){notify(e);}finally{b.disabled=false;}
 };
 drawGroups();if(catalog.sources.length)await sourceChanged(!draft||!catalog.sources.some(s=>s.id===draft.source));
};

const shadowJobsBase=updateAnalysisJobs;
updateAnalysisJobs=jobs=>{
 shadowJobsBase(jobs);const host=$('analysisJobs');if(!host)return;
 host.querySelectorAll('[data-shadow-action]').forEach(n=>n.remove());
 for(const j of jobs.filter(j=>j.kind==='shadow')){
  const row=[...host.querySelectorAll('.record-row')].find(n=>n.querySelector('[data-job]')?.dataset.job===j.id);
  if(row)continue;
  // Active/partial job actions outside pagination remain bounded (one active job).
  if(['queued','running'].includes(j.state)){
   const button=document.createElement('button');button.dataset.shadowAction='cancel';button.textContent='取消本機影子工作（保留已完成部分）';
   button.onclick=async()=>{button.disabled=true;try{await api('/api/analysis/cancel',{job:j.id});}catch(e){notify(e);button.disabled=false;}};
   host.prepend(button);
  }
 }
};
const shadowLoadBase=loadAnalysis;
loadAnalysis=async id=>{
 const d=await api('/api/analysis/result/'+id);
 if(d.kind!=='shadow')return shadowLoadBase(id);
 const host=$('analysisCompare');if(!host)return;host.hidden=false;
 const names=Object.keys(d.models),n=Math.max(0,...names.map(k=>d.models[k].length));
 host.innerHTML=`<h2>離線影子同影格比較 ${d.partial?'（部分完成）':''}</h2>
 <p>辨識新算${d.cache.inferred}張、遮罩沿用${d.cache.mask_reused}張、路徑沿用${d.cache.path_reused}張；姿態：${esc(d.spec.pose)}。沒有重新執行已存在快取的模型。</p>
 ${d.geometry_candidate?`<p>A＝保留原共用車道K；B＝幾何失配後處理候選。<button id="shadowBaseline">開啟保留的整段原版（同影格）</button></p>`:d.mask_switch_candidate?`<p>${d.orin_lane_candidate?"A＝入口約束v3.2；B＝筆電凍結K；C＝Orin離線K（相同原圖與遮罩，重新計算光流與路徑）。":d.lane_candidate?"A＝入口約束v3.2；B＝前版遮罩K；C＝共用車道約束K。":d.entrance_candidate?"A＝原道路v3.1；B＝入口約束v3.2；C＝入口約束A＋遮罩切換K。":"A＝原道路v3.1；B＝原K；C＝遮罩切換K。"}<button id="shadowBaseline">開啟保留的原版</button>。灰色分帶＝遮罩檢查區；橘點＝分支證據。共用車道版只計入與近處邊界連通的白線，仍須跨格和入口核對。</p><div>${(d.orin_lane_candidate?[23.4,71.4,79,97.3,108.9,117.7,142,148.9]:[23.4,71.4,79,96.4,96.8,97.3,108.9,109.8,117.3,117.7,142,143.2]).map(t=>`<button data-road-frame="${Math.round(t*10)}">${t}秒</button>`).join(" ")}</div>`:d.baseline_job?`<p>前版ABC保留：<button id="shadowBaseline">開啟前版同影格</button>。黃色實線＝實際PID取樣；新版依可見支持往下調整。候選分數為啟發式，不是準確率。</p><div>${Object.entries(d.comparison||{}).map(([k,v])=>`<p>${esc(k)}：原PID ${v.baseline_pid} → 新路徑固定黃線 ${v.fixed_row_on_new_path} → 動態黃線 ${v.new_pid}；共${n}張。</p>`).join('')}</div>`:''}
 ${d.road_comparison?`<h3>道路範圍比較（影像垂直跨度，不是準確率）</h3><p>每格顯示「前版 → 新版可見跨度（PID所用連續段）」；斷開的道路不會硬接給PID。</p><table><thead><tr><th>影格</th>${names.map(k=>`<th>${esc(k)} · px</th>`).join('')}</tr></thead><tbody>${Object.keys(d.road_comparison.A).map(i=>`<tr><td><button data-road-frame="${i}">${i}</button></td>${names.map(k=>{const v=d.road_comparison[k][i];return `<td>${v.old_span_px.toFixed(0)} → ${v.new_coverage.visible_span_px.toFixed(0)}（${v.new_coverage.control_span_px.toFixed(0)}）</td>`;}).join('')}</tr>`).join('')}</tbody></table>`:''}
 <p>${d.limitations.map(esc).join('<br>')}</p><details><summary>凍結設定與來源</summary><pre>${esc(JSON.stringify({spec:d.spec,recognition:d.recognition},null,2))}</pre></details>
 <div class="actions"><button id="shadowPlay">播放</button><button id="shadowPrev">上一張</button><button id="shadowNext">下一張</button><label><input id="shadowMask" type="checkbox" checked>遮罩</label><label>遮罩透明度<input id="shadowAlpha" aria-label="比較遮罩透明度" type="range" min="0" max="100" value="25"></label><label><input id="shadowCandidates" type="checkbox">灰色候選</label><label><input id="shadowRaw" type="checkbox">只看原圖</label><label><input id="shadowPath" type="checkbox" checked>估計道路中心／PID取樣線</label><label><input id="shadowEdges" type="checkbox" checked>道路邊界</label><label><input id="shadowMemory" type="checkbox" checked>顯示道路記憶</label></div>
 <p>K比較組：藍虛線＝K輸出中心。其他路口候選：藍虛線＝入口方向延伸，黃線段＝出口候選，粉紅＝選中出口，橘虛線＝推估過渡路徑。道路來源：綠實線＝雙線支持；綠虛線＝單線推估；紫虛線＝寬度範圍外推估；橘虛線＝歷史或歷史融合（越淡越不確定）。青線＝當前選用白線內緣；黃色線／圓點＝PID取樣。箭頭是估計道路方向，不是車輛軌跡；斷段表示未連接，不硬補。</p>
 <input id="shadowSeek" aria-label="影子比較影格" type="range" min="0" max="${Math.max(0,n-1)}" value="0"><p id="shadowTime"></p>
 <div class="grid three">${names.map(k=>`<article><h3>${esc(k+' 比較組 · '+((d.geometry_candidate||d.orin_lane_candidate)?d.spec.groups.find(g=>g.id===k).label:pathName(d.spec.groups.find(g=>g.id===k).algorithm)))}</h3><canvas id="shadowCanvas${k}" width="640" height="360" style="width:100%"></canvas><p id="shadowInfo${k}"></p><details><summary>影像外道路記憶（相對影像示意，非公分俯視）</summary><canvas id="shadowMemoryCanvas${k}" width="640" height="320" style="width:100%"></canvas><p>框內為目前相機視野；框外為對齊後歷史或推估，不是已走過的車輛軌跡。</p></details><details><summary>候選品質／PID提案</summary><pre id="shadowDiag${k}"></pre></details></article>`).join('')}</div>
 <h3>轉向提案時序（點選跳到影格）</h3><canvas id="shadowChart" width="960" height="220" style="width:100%;height:220px"></canvas>
 <div>${names.map(k=>{const s=d.summaries[k];return `<p>${esc(k)}：路徑有效${s.path_valid}/${s.frames} · PID有效${s.pid_valid}/${s.frames} · 輸出受限${s.limited}張 · 單輪${s.first_stop?'在來源影格'+s.first_stop.frame+'停止：'+esc(s.first_stop.reason):s.started?'區間內未觸發停止':'未達模擬開始條件'}</p>`;}).join('')}</div>
 <h3>模式切換／失效事件</h3><div id="shadowEvents"></div>
 <h3>區間評語（追加保存）</h3><div class="formgrid">${select('shadowReviewGroup','比較組',names)}${field('shadowReviewFrom','起始結果索引',0,'number')}${field('shadowReviewTo','結束結果索引',Math.max(0,n-1),'number')}${select('shadowVerdict','判讀',['未確認','正確','錯誤','不確定'])}<label>評語<textarea id="shadowNotes" maxlength="5000"></textarea></label></div><button id="shadowReviewSave">保存評語</button><div id="shadowReviews"></div>`;
 if(d.geometry_candidate){const note=document.createElement('div');note.innerHTML='<h3>原K → 幾何失配候選 · 遮罩完全相同</h3><p>候選只修改路徑。單側寬度無法核對不等於已識別坡道；縮短路徑也不等於準確率提高。兩組均未執行PID，黃色列僅為保存的取樣設定。</p><div>'+[160,164,168,174,174.1,174.2,175.9].map(t=>`<button data-road-frame="${Math.round(t*10)-d.baseline_offset}">${Math.floor(t/60)}:${(t%60).toFixed(1).padStart(4,'0')}</button>`).join(' ')+'</div>';host.insertBefore(note,host.children[1]);}
 if(d.geometry_candidate)host.querySelector('.grid.three').style.gridTemplateColumns='repeat(2,minmax(0,1fr))';
 if($('shadowBaseline'))$('shadowBaseline').onclick=async()=>{const i=Number($('shadowSeek').value)+(d.baseline_offset||0);await loadAnalysis(d.baseline_job);$('shadowSeek').value=i;$('shadowSeek').dispatchEvent(new Event('input'));};
 if(!n){$('shadowTime').textContent='取消前尚未產生路徑；辨識快取已保留，可建立下一輪沿用。';return;}
 let token=0,playing=false;
 async function draw(){
  const mine=++token,i=Number($('shadowSeek').value);$('shadowTime').textContent=`結果索引${i} · 來源影格${d.frames[i].frame} · 原始時間${d.frames[i].capture_s.toFixed(3)}秒${d.frames[i].player_s===undefined?'':` · 播放器${d.frames[i].player_s.toFixed(1)}秒 · 原片影格${d.frames[i].source_video_frame}`}`;
  for(const k of names){
   const row=d.models[k][i],c=$('shadowCanvas'+k).getContext('2d');c.clearRect(0,0,640,360);
   if(!row){$('shadowInfo'+k).textContent='此組尚未計算此張';continue;}
   const image=new Image();image.src=`/analysis-file/${id}/raw/${String(i).padStart(6,'0')}.jpg`;await image.decode();if(mine!==token||!host.isConnected)return;c.drawImage(image,0,0,640,360);
   if($('shadowMask').checked&&!$('shadowRaw').checked){const mask=new Image();mask.src=`/analysis-file/${id}/${k}/${String(i).padStart(6,'0')}.png`;await mask.decode();if(mine!==token||!host.isConnected)return;VisionLayers.mask(c,mask,Number($('shadowAlpha').value)/100);}
   const g=d.spec.groups.find(g=>g.id===k),r=row.A;
   function stroke(points,color,dashed=false,scale=1){if(!points?.length)return;c.strokeStyle=color;c.lineWidth=3;c.setLineDash(dashed?[6,4]:[]);c.beginPath();points.forEach(([x,y],j)=>j?c.lineTo(x/scale,y/scale):c.moveTo(x/scale,y/scale));c.stroke();c.setLineDash([]);}
   if(!$('shadowRaw').checked){
    if($('shadowCandidates').checked)(r.boundaries||[]).forEach(tr=>stroke(tr.map(([y,l,h])=>[(l+h)/2,y]),'#9ba8ae',true));
    if($('shadowEdges').checked)(r.observed_edges||[]).forEach(edge=>stroke(edge,'#00dfff'));
    if(r.k?.mask_gate&&$('shadowEdges').checked){for(const y of [150,210,270,334])stroke([[0,y],[640,y]],'#cccccc',true);c.fillStyle='#ff9700';for(const [x,y] of r.k.mask_gate.branch_points){c.beginPath();c.arc(x,y,2,0,Math.PI*2);c.fill();}}
   }
   if(!$('shadowRaw').checked&&r.junction){
    stroke(r.junction.entry_direction,'#248bff',true);
    r.junction.candidates.forEach((candidate,index)=>stroke(candidate.points,index===r.junction.selected?'#ff44c8':'#ffb400',index!==r.junction.selected));
   }
   if($('shadowPath').checked&&!$('shadowRaw').checked){
    const colors={junction_inferred:'#ffb020',double:'#00ff66',single:'#00ff66',single_metric:'#00ff66',single_extrapolated:'#ce79ff',history:'#ff8c24',single_history:'#ff8c24'};
    if($('shadowMemory').checked)(r.history_segments||[]).forEach(seg=>{c.globalAlpha=Math.max(.12,Math.min(.5,seg.confidence));stroke(seg.points,'#ff8c24',true,2);});c.globalAlpha=1;
    if(r.road_segments)r.road_segments.forEach(seg=>{c.globalAlpha=Math.max(.55,Math.min(1,seg.confidence));stroke(seg.points,colors[seg.source]||'#00ff66',seg.source!=='double',2);});
    else {stroke(r.path,'#00ff66',false,2);stroke(r.history_path,'#ff8c24',true,2);}c.globalAlpha=1;
    if(r.k)stroke(r.path,'#1681ff',true,2);
    const points=r.path;if(points.length>2){const [x,y]=points[0].map(v=>v/2),[bx,by]=points[Math.min(4,points.length-1)].map(v=>v/2),a=Math.atan2(y-by,x-bx);c.strokeStyle=r.history_path?.length?'#ff8c24':'#00ff66';c.beginPath();c.moveTo(x-9*Math.cos(a-.5),y-9*Math.sin(a-.5));c.lineTo(x,y);c.lineTo(x-9*Math.cos(a+.5),y-9*Math.sin(a+.5));c.stroke();}
    const rowY=(row.pid?.sample_row_ratio??g.parameters.row_ratio)*719;
    stroke([[0,rowY/2],[640,rowY/2]],'#ffdd00');
    for(let j=1;j<points.length;j++){const [x0,y0]=points[j-1],[x1,y1]=points[j];if(y0<=rowY&&rowY<=y1&&y1>y0){const x=x0+(x1-x0)*(rowY-y0)/(y1-y0);c.fillStyle='#ffdd00';c.beginPath();c.arc(x/2,rowY/2,4,0,Math.PI*2);c.fill();break;}}
   }
   const mc=$('shadowMemoryCanvas'+k).getContext('2d');mc.lineWidth=3;mc.font='18px sans-serif';mc.fillStyle='#eef3f1';mc.fillRect(0,0,640,320);
   const mem=$('shadowMemory').checked?(r.history_segments||[]):[],all=[...(r.road_segments||[]),...mem],coords=all.flatMap(seg=>seg.points);
   const xs=coords.map(p=>p[0]/2),ys=coords.map(p=>p[1]/2),minx=Math.min(-80,...xs),maxx=Math.max(720,...xs),miny=Math.min(-40,...ys),maxy=Math.max(400,...ys),scale=Math.min(600/(maxx-minx),270/(maxy-miny));
   const mx=x=>20+(x-minx)*scale,my=y=>30+(y-miny)*scale;
   mc.strokeStyle='#52656b';mc.strokeRect(mx(0),my(0),640*scale,360*scale);mc.fillStyle='#223';mc.fillText('目前影像框；外部線段為記憶／推估，非量測地圖',15,18);
   all.forEach(seg=>{mc.strokeStyle=mem.includes(seg)?'#d87518':seg.source==='single_extrapolated'?'#9d49db':'#168b56';mc.globalAlpha=Math.max(.45,seg.confidence);mc.setLineDash(seg.source==='double'?[]:[4,3]);mc.beginPath();seg.points.forEach(([x,y],j)=>j?mc.lineTo(mx(x/2),my(y/2)):mc.moveTo(mx(x/2),my(y/2)));mc.stroke();});mc.globalAlpha=1;mc.setLineDash([]);
   $('shadowInfo'+k).textContent=`${(r.mode||'—').split('+').map(x=>({double:'雙線',single:'單線',single_metric:'單線投影',single_extrapolated:'單線寬度外推',history:'歷史',single_history:'單線＋歷史',junction_inferred:'路口直行推估',no_evidence:'無可用證據'}[x]||x)).join('／')} · ${r.coverage?'可見跨度'+r.coverage.visible_span_px.toFixed(0)+'px／控制連續段'+r.coverage.control_span_px.toFixed(0)+'px · ':''}黃線${((row.pid?.sample_row_ratio??g.parameters.row_ratio)*100).toFixed(1)}% · 路徑${r.valid?'有效':'無效'} · PID ${row.pid?.reason??'關閉'} · 單輪${row.simulated_run} · ${r.path_ms.toFixed(1)}ms（${r.execution_host==='orin'?"Orin離線路徑，非即時控制延遲":r.timing_scope?"快取參考後切換計算，非完整耗時":"筆電路徑"}）`;
   if(d.entrance_candidate&&(d.lane_candidate||k!=='A')){const rejected=(r.rejections||[]).filter(x=>x.reason==='FAR_PAIR_NOT_CONNECTED_TO_ENTRANCE').length;$('shadowInfo'+k).textContent+=' · 遠方未接入口配對排除 '+rejected+' 組 · 歷史對齊 '+(r.memory_state===undefined?'即時入口未回報此欄，詳見motion診斷':r.memory_state.aligned?'成功':'不可用；不直接套舊座標');}
   if(r.k?.mask_gate){const g=r.k.mask_gate;const reasons={FOLLOW_A:'一般道路，維持A',LANE_SUSPECT_KEEP_A:'本車道開口待確認，維持A',K_ENTRANCE_MISMATCH_KEEP_A:'K入口位置或方向未通過確認，維持A',SAME_LANE_OPENING_AND_REFERENCE:'本車道連通開口跨格成立，K入口核對通過',LANE_EXIT_CONFIRMED:'本車道出口恢復，接回A',SUSPECT_KEEP_A:'疑似路口，維持A',MASK_CONFIRMED_BUT_NO_K_REFERENCE_KEEP_A:'有路口證據但K無參考，維持A',MASK_BRANCH_SEQUENCE_AND_K_REFERENCE:'遮罩分支連續成立且K有參考，接管',CROSS_KEEP_REFERENCE:'穿越中，保留K參考',REFERENCE_LOST_RETURN_A:'K參考失效，撤銷接管回A',CLEAR_CURRENT_ROAD_CANCEL_JUNCTION:'開口消失且當前道路連續，撤銷路口',EXIT_DOUBLE_CONFIRMED:'出口雙線連續確認',BLEND_BACK_TO_A:'平順接回A',HANDOFF_COMPLETE:'完成接回A',EXIT_DOUBLE_LOST:'接回期間雙線不足',RETURN_A_NO_REFERENCE:'參考失效，回A'};$('shadowInfo'+k).textContent+=' · 切換原因：'+(reasons[g.reason]||g.reason)+' · 遠／中／近分支列 '+g.zones.map(z=>z.branch_rows).join('/')+(g.raw_zone_counts?' · K通過入口確認 ':' · K參考 ')+(g.reference_ready?'有':'無');if(g.raw_zone_counts)$('shadowInfo'+k).textContent+=' · 全畫面分支 '+g.raw_zone_counts.join('/')+' → 本車道 '+g.zones.map(z=>z.branch_rows).join('/')+' · K入口 '+(g.reference_compatible?'符合':'不符或不足')+' · 接近開口票 '+g.near_votes; }
   if(r.geometry){const g=r.geometry;$('shadowInfo'+k).textContent+=' · '+({BASELINE:'維持原路徑（失配未連續確認）',SINGLE_WIDTH_UNVERIFIABLE:'單側寬度無法核對，只留近段低信心推算',CURRENT_PAIR_MISMATCH:'失配連續確認，改用當前雙線支持段'}[g.state]||g.state)+' · 寬度證據年齡 '+g.width_age_s.toFixed(2)+'s · 同列中心差 '+(g.residual_px===null?'無雙側可核對':g.residual_px.toFixed(1)+'px')+' · 連續失配 '+g.mismatch_votes+'格';}
   $('shadowDiag'+k).textContent=JSON.stringify({geometry:r.geometry,k:r.k,junction:r.junction,memory_state:r.memory_state,motion:r.motion,reason:r.reason,transition:r.transition,candidates:r.candidates,rejections:r.rejections,history_points:r.history_points,path_semantics:r.path_semantics,coverage:r.coverage,segments:r.road_segments,pid:row.pid,steer_change_us:row.steer_change_us},null,2);
  }
  drawChart();
 }
 host.querySelectorAll('[data-road-frame]').forEach(b=>b.onclick=()=>{$('shadowSeek').value=b.dataset.roadFrame;draw().catch(notify);});
 const events=names.flatMap(k=>d.summaries[k].events.map(e=>({...e,group:k})));
 pagedList('shadowEvents',events,e=>`<p><button class="ghost" data-jump="${e.index}">${esc(e.group)} · ${e.index}</button> ${esc(e.text)}</p>`);
 $('shadowEvents').onclick=e=>{if(e.target.dataset.jump!==undefined){$('shadowSeek').value=e.target.dataset.jump;draw().catch(notify);}};
 const chart=$('shadowChart'),ctx=chart.getContext('2d'),values=names.flatMap(k=>d.models[k].map(r=>r.pid?.steer_us).filter(Number.isFinite));
 const lo=Math.floor((Math.min(...values,1360)-20)/100)*100,hi=Math.ceil((Math.max(...values,1360)+20)/100)*100;
 const tStart=d.frames[0].capture_s,tEnd=d.frames[n-1].capture_s;
 const timeX=t=>52+(t-tStart)/Math.max(.001,tEnd-tStart)*875,valueY=v=>175-(v-lo)/(hi-lo)*145;
 function drawChart(){
  ctx.fillStyle='#eef3f1';ctx.fillRect(0,0,960,220);ctx.font='11px sans-serif';ctx.fillStyle='#223';ctx.fillText('轉向提案 µs（非道路曲線）；下方短刻痕＝區段來源切換',52,13);
  for(let v=lo;v<=hi;v+=100){ctx.strokeStyle='#d3ddda';ctx.beginPath();ctx.moveTo(52,valueY(v));ctx.lineTo(927,valueY(v));ctx.stroke();ctx.fillStyle='#223';ctx.fillText(String(v),8,valueY(v)+4);}
  for(let t=Math.ceil(tStart/5)*5;t<=tEnd;t+=5){ctx.strokeStyle='#d3ddda';ctx.beginPath();ctx.moveTo(timeX(t),30);ctx.lineTo(timeX(t),175);ctx.stroke();ctx.fillStyle='#223';ctx.fillText(t.toFixed(0)+'s',timeX(t)-6,212);}
  ctx.strokeStyle='#666';ctx.setLineDash([5,4]);ctx.beginPath();ctx.moveTo(52,valueY(1360));ctx.lineTo(927,valueY(1360));ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='#333';ctx.fillText('中心 1360',835,valueY(1360)-4);
  names.forEach((k,z)=>{ctx.strokeStyle=['#087f5b','#1971c2','#c2255c'][z];ctx.beginPath();let pen=false;d.models[k].forEach((r,i)=>{const v=r.pid?.steer_us;if(!Number.isFinite(v)){pen=false;return;}const x=timeX(d.frames[i].capture_s),y=valueY(v);pen?ctx.lineTo(x,y):ctx.moveTo(x,y);pen=true;});ctx.stroke();
   ctx.fillStyle=ctx.strokeStyle;ctx.fillText(k,850+z*30,13);d.models[k].forEach((r,i)=>{if(i&&r.A.mode!==d.models[k][i-1].A.mode)ctx.fillRect(timeX(d.frames[i].capture_s),181+z*6,1,5);});});
  const selected=d.frames[Number($('shadowSeek').value)].capture_s;ctx.strokeStyle='#111';ctx.beginPath();ctx.moveTo(timeX(selected),25);ctx.lineTo(timeX(selected),200);ctx.stroke();
 }
 drawChart();
 chart.onclick=e=>{const time=tStart+Math.max(0,Math.min(1,(e.offsetX/chart.clientWidth*960-52)/875))*(tEnd-tStart);let best=0;d.frames.forEach((f,i)=>{if(Math.abs(f.capture_s-time)<Math.abs(d.frames[best].capture_s-time))best=i;});$('shadowSeek').value=best;draw().catch(notify);};
 $('shadowAlpha').oninput=()=>draw().catch(notify);
 $('shadowSeek').oninput=()=>draw().catch(notify);for(const k of ['shadowMask','shadowPath','shadowEdges','shadowMemory','shadowCandidates','shadowRaw','shadowAlpha'])$(k).onchange=()=>draw().catch(notify);
 $('shadowPrev').onclick=()=>{$('shadowSeek').value=Math.max(0,Number($('shadowSeek').value)-1);draw().catch(notify);};
 $('shadowNext').onclick=()=>{$('shadowSeek').value=Math.min(n-1,Number($('shadowSeek').value)+1);draw().catch(notify);};
 $('shadowPlay').onclick=()=>{playing=!playing;$('shadowPlay').textContent=playing?'暫停':'播放';if(!playing)return;const first=Number($('shadowSeek').value),start=performance.now(),t0=d.frames[first].capture_s;const tick=()=>{if(!playing||!host.isConnected)return;let i=Number($('shadowSeek').value);const t=t0+(performance.now()-start)/1000;while(i+1<n&&d.frames[i+1].capture_s<=t)i++;if(i!==Number($('shadowSeek').value)){$('shadowSeek').value=i;draw().catch(notify);}if(i===n-1){playing=false;$('shadowPlay').textContent='播放';}else requestAnimationFrame(tick);};tick();};
 const reviewKey='shadow-review-draft-'+id,fields=['shadowReviewGroup','shadowReviewFrom','shadowReviewTo','shadowVerdict','shadowNotes'];
 LastInputs.bind('shadow-view',['shadowMask','shadowAlpha','shadowCandidates','shadowRaw','shadowPath','shadowEdges','shadowMemory']);
 let reviewDraft;try{reviewDraft=JSON.parse(localStorage.getItem(reviewKey))}catch{}
 for(const f of fields){if(reviewDraft?.[f]!==undefined)$(f).value=reviewDraft[f];$(f).oninput=()=>localStorage.setItem(reviewKey,JSON.stringify(Object.fromEntries(fields.map(x=>[x,$(x).value]))));}
 async function reviews(){pagedList('shadowReviews',await api('/api/analysis/reviews/'+id),r=>`<p>${esc(r.model)} · ${r.start}～${r.end} · ${esc(r.verdict)}：${esc(r.notes)}</p>`);}
 $('shadowReviewSave').onclick=async()=>{try{await api('/api/analysis/review',{job:id,start:Number($('shadowReviewFrom').value),end:Number($('shadowReviewTo').value),model:$('shadowReviewGroup').value,target:'A',verdict:$('shadowVerdict').value,notes:$('shadowNotes').value});localStorage.removeItem(reviewKey);await reviews();}catch(e){notify(e);}};
 await draw();await reviews();
};
