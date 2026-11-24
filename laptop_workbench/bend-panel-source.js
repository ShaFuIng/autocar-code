/* Offline full-track comparison. Never calls vehicle endpoints. */
(()=>{
'use strict';
const $=id=>document.getElementById(id),base='/experiment/bend-cases/';
const names={mask:'V4 白線遮罩',legacy:'原9列路徑（重算）',v1:'v1 候選（舊投影情境）',v2:'v2 雙線影像中心',
 left:'僅沿左線等距（推估）',right:'僅沿右線等距（推估）',priority:'雙線優先／單線備援（實驗）',
 A:'歷史 A 路徑',B:'歷史 B 路徑',C:'歷史 C 路徑',
 tracked_left:'新版：追蹤左邊界等距（暫估）',tracked_right:'新版：追蹤右邊界等距（暫估）'};
let loading=false,dispose=()=>{};
async function read(url){const r=await fetch(url);if(!r.ok)throw Error('讀取失敗 '+r.status);return r.json();}
async function mount(){
 if(location.hash!=='#models'){dispose();return;}
 if(loading||$('bendCases')||!$('curvePanel'))return;
 loading=true;
 const host=document.createElement('section');host.className='card';host.id='bendCases';
 host.innerHTML='<h2>完整跑道：大彎／小彎算法比較</h2><p>讀取離線案例…</p>';
 $('curvePanel').before(host);
 try{
 const manifest=await read(base+'manifest.json');if(!host.isConnected)return;
 host.innerHTML=`<h2>完整跑道：大彎／小彎算法比較</h2>
 <p><b>只有錄影回放，不會動車。</b>固定V4辨識，分別看不同路徑算法。原片是10/16完整跑道，早於相機下壓；單線等距使用舊姿態的暫估情境，不能拿疊圖量準確公分。</p>
 <label>比較路段<select id="bendCase"></select></label>
 <p>「沿一條線走」是保持指定距離，直線與彎道皆可。它與雙線方法共用參考路徑介面，後續只由一個控制器輸出。實車建議還需邊界可信度、切換遲滯、平順交接與車輪餘裕檢查。</p>
 <details><summary>本次比較規則與限制</summary><p>左右單線各沿其地面曲線法線向內偏移25.25cm（白線中心到道路中心）。這不是輪胎到白線的間距；淨寬46cm、車寬32cm時置中僅有約7cm名義單側餘裕。若要更靠近某側，須另定目標與掃掠範圍。</p><p>優先策略：當前雙線 → 當前可用的原選定單側 → 另一側；切換需連續3格證據，不保留未對齊的過去路徑。這只是可觀察的實驗基線，尚無平順交接，也未驗證實車。資料不足時不畫路徑。</p><p id="bendLimits"></p><a href="https://github.com/f1tenth/f1tenth_coursekit/blob/master/assignments/labs/lab3.rst" target="_blank" rel="noopener">F1TENTH 等距沿牆：控制概念參考，感測器不同</a></details>
 <div id="bendLayers" style="max-height:250px;overflow:auto"></div>
 <div class="actions"><button id="bendPlay">播放</button><label>速度<select id="bendSpeed"><option value=".5">0.5×</option><option value="1" selected>1×</option><option value="2">2×</option></select></label><button class="ghost" id="bendSave">保存本次比較與評語</button></div>
 <label>比較評語<input id="bendNote" placeholder="例如：哪一段單線推估偏移不合理"></label>
 <p id="bendState" role="status"></p>
 <canvas id="bendCanvas" width="640" height="360" aria-label="完整跑道原圖與各算法路徑" style="display:block;width:100%;max-width:800px;border-radius:8px"></canvas>
 <label>原片影格<input id="bendFrame" type="range" min="0" step="1" value="0" style="width:100%"></label>
 <p id="bendInfo"></p><p id="bendDiagnosis"></p><p>新版只保留通過相鄰形狀比對的邊界身分，路徑全部來自當前遮罩；不是已融合陀螺儀的空間記憶。舊版與原優先策略保留作對照。地面距離仍未標定。</p><p id="bendStats"></p>`;
 $('bendLimits').textContent=manifest.limitations.join(' ');
 $('bendCase').replaceChildren(...manifest.cases.map(c=>{const o=document.createElement('option');o.value=c.id;o.textContent=`${c.label} · 原片 ${c.start.toFixed(1)}–${c.end.toFixed(1)}秒`;return o;}));
 let layers=Object.keys(names).map((k,i)=>({key:k,color:['#dc28dc','#2196ff','#ffff00','#00ee32','#ff9900','#3ccfff','#ffffff','#ff6688','#aabbff','#ff33aa','#ff8000','#00cddd'][i],show:['mask','tracked_left','tracked_right'].includes(k)}));
 try{const stored=JSON.parse(localStorage.getItem('bend-layers-v2'));if(stored?.length===12&&stored.every(l=>names[l.key]&&/^#[0-9a-f]{6}$/i.test(l.color)&&typeof l.show==='boolean'))layers=stored;}catch{}
 let data=null,index=0,playing=false,timer=null,serial=0,loadSerial=0,cache=new Map();
 const pause=()=>{playing=false;clearTimeout(timer);if($('bendPlay'))$('bendPlay').textContent='播放';};
 dispose=()=>{pause();serial++;};
 const saveLocal=()=>{try{localStorage.setItem('bend-layers-v2',JSON.stringify(layers));}catch{}};
 $('bendLayers').replaceChildren(...layers.map((l,i)=>{
  const row=document.createElement('label');row.style.cssText='display:flex;gap:10px;align-items:center;margin:8px 0';
  const on=document.createElement('input');on.type='checkbox';on.checked=l.show;on.setAttribute('aria-label',names[l.key]);on.onchange=()=>{l.show=on.checked;saveLocal();draw();};
  const color=document.createElement('input');color.type='color';color.value=l.color;color.style.cssText='width:50px;height:32px;padding:2px';color.setAttribute('aria-label',names[l.key]+' 顏色');color.oninput=()=>{l.color=color.value;saveLocal();draw();};
  row.append(on,color,document.createTextNode(names[l.key]));return row;
 }));
 function image(url){if(!cache.has(url)){cache.set(url,new Promise((resolve,reject)=>{const im=new Image();im.onload=()=>resolve(im);im.onerror=()=>reject(Error('原圖載入失敗'));im.src=url;}));if(cache.size>10)cache.delete(cache.keys().next().value);}return cache.get(url);}
 async function draw(){
  if(!data)return;
  const ticket=++serial,f=data.frames[index],root=base+data.id+'/'+String(f.frame).padStart(6,'0');
  const ctx=$('bendCanvas').getContext('2d');ctx.clearRect(0,0,640,360);
  try{
   const [im,mask]=await Promise.all([image(root+'-raw.jpg'),layers.some(l=>l.show&&l.key==='mask')?image(root+'-mask.png'):Promise.resolve(null)]);
   if(ticket!==serial||!host.isConnected)return;
   ctx.drawImage(im,0,0);
   for(const l of layers.filter(l=>l.show)){
    if(l.key==='mask'){const off=document.createElement('canvas');off.width=640;off.height=360;const oc=off.getContext('2d');oc.drawImage(mask,0,0);oc.globalCompositeOperation='source-in';oc.fillStyle=l.color;oc.fillRect(0,0,640,360);ctx.globalAlpha=.4;ctx.drawImage(off,0,0);ctx.globalAlpha=1;}
    else{ctx.strokeStyle=l.color;ctx.fillStyle=l.color;ctx.lineWidth=1.5;
     for(const [x,y] of (f.paths[l.key]||[])){ctx.beginPath();ctx.arc(x,y,['left','right'].includes(l.key)?2.5:1.8,0,Math.PI*2);['left','right'].includes(l.key)?ctx.stroke():ctx.fill();}
    }
   }
   const modes={double:'雙線',left:'沿左線',right:'沿右線',confirming:'等待連續證據',no_evidence:'無足夠證據'};
   const why={OK:'偏移有輸出（距離未標定）',TRACK_ASSOCIATED:'延續同一邊界身分',VISIBLE_ANCHOR:'當前可見錨點確認',NO_OBSERVATION:'找不到足夠連續白線',IDENTITY_AMBIGUOUS:'左右身分證據不足',PROJECTION_OUT_OF_RANGE:'有效地面投影不足',INSUFFICIENT_DISTINCT_POINTS:'不同投影點不足',INSUFFICIENT_ARC_LENGTH:'可用曲線太短',OFFSET_FOLDBACK:'偏移曲線折返，已拒絕',OFFSET_OUT_OF_RANGE:'偏移後超出有效範圍'};
   $('bendDiagnosis').textContent='新版診斷：'+Object.entries(f.identity_diagnostics||{}).map(([s,d])=>`${s==='left'?'左':'右'}：${why[d.reason]||d.reason}；${why[d.identity?.reason]||''}；觀測 ${d.observed_points||0} 點／投影 ${d.projected_points||0} 點`).join(' ｜ ');
   $('bendInfo').textContent=`原片 ${f.t.toFixed(2)}秒 · 原影格 ${f.source_frame} · ${index+1}/${data.frames.length} · 舊版優先策略（對照）：${modes[f.mode]||f.mode}${f.pending?'，待確認 '+(modes[f.pending]||f.pending):''}。舊版身分錨點：左${f.side_anchors?.left?'通過':'未通過'}／右${f.side_anchors?.right?'通過':'未通過'}。本格無輸出：${Object.keys(f.paths).filter(k=>!f.paths[k].length).map(k=>names[k]).join('、')||'無'}。`;
  }catch(e){$('bendState').textContent=e.message;pause();}
 }
 async function tick(){
  if(!playing||!host.isConnected){pause();return;}
  if(index>=data.frames.length-1){pause();return;}
  const delay=Math.max(15,(data.frames[index+1].t-data.frames[index].t)*1000/Number($('bendSpeed').value));
  timer=setTimeout(async()=>{if(!playing)return;index++;$('bendFrame').value=index;await draw();tick();},delay);
 }
 async function load(){
  pause();const ticket=++loadSerial;serial++;$('bendState').textContent='讀取路段…';
  const d=await read(base+$('bendCase').value+'/result.json');if(ticket!==loadSerial||!host.isConnected)return;
  data=d;index=0;cache.clear();$('bendFrame').max=d.frames.length-1;$('bendFrame').value=0;
  $('bendStats').textContent='各路徑輸出影格（不是準確率）：'+Object.entries(d.availability).map(([k,v])=>names[k]+' '+v+'/'+d.frames.length).join('；');
  $('bendState').textContent='已載入；所有運算為離線結果';await draw();
 }
 $('bendCase').onchange=()=>load().catch(e=>$('bendState').textContent=e.message);
 $('bendFrame').oninput=()=>{pause();index=Number($('bendFrame').value);draw();};
 $('bendPlay').onclick=()=>{if(!data)return;if(playing)pause();else{if(index>=data.frames.length-1)index=0;playing=true;$('bendPlay').textContent='暫停';draw();tick();}};
 $('bendSave').onclick=async()=>{
  if(!data)return;pause();
  const payload={key:'bend-review-'+crypto.randomUUID(),rev:0,data:{case:data.id,source_frame:data.frames[index].source_frame,
   source_time:data.frames[index].t,video_sha256:manifest.video_sha256,model:manifest.model,layers,note:$('bendNote').value,geometry:manifest.geometry,hardware_output:false}};
  try{const r=await fetch('/api/document',{method:'POST',headers:{'Content-Type':'application/json','X-Workbench':'1'},body:JSON.stringify(payload)});if(!r.ok)throw Error('保存失敗');$('bendState').textContent='已保存本次影格、圖層與評語 '+payload.key.slice(-8)+'；未套用實車';}catch(e){$('bendState').textContent=e.message;}
 };
 await load();
 }catch(e){host.append(document.createTextNode('案例尚未就緒：'+e.message));}
 finally{loading=false;}
}
new MutationObserver(mount).observe(document.getElementById('main'),{childList:true});
window.addEventListener('hashchange',mount);mount();
})();
