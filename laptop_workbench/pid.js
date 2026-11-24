'use strict';
(()=>{
const labels={visual_center_offset_px_640:'視覺中心偏移 px（640寬，正值向右）',kp:'P 增益（µs／正規化偏差）',ki:'I 增益（µs／偏差秒，先保持0）',kd:'D 增益（µs秒／偏差）',left_gain:'左轉倍率',right_gain:'右轉倍率',center_us:'中心 µs（提案，不修改共用中心）',min_us:'左側限位 µs',max_us:'右側限位 µs',steer_rate:'轉向速度上限 µs／秒',pwm:'固定 PWM 提案 %',row_ratio:'前視影像列 0～1（上→下）',derivative_tau:'D 濾波時間 秒',integral_limit:'積分累計上限 偏差秒',max_gap:'最長有效影格間隔 秒'};
let mounting=false;
async function mount(){
 if(route==='overview'&&!$('pidReadiness')&&$('main').querySelector('.heading')){
  $('main').insertAdjacentHTML('beforeend','<section class="card" id="pidReadiness"><h2>PID現場測試與待辦</h2><p><b>Orin限時自主PID：</b>依使用者需求，04新增網頁斷線／失焦仍繼續本輪的模式，最多15秒。筆電與模擬測試已完成；Orin部署／重啟待授權，04會依車端能力禁止未支援的啟動。保留實體急停、限時、影像／路徑失效、內部控制與通訊故障停止。實際斷線試跑尚未驗收。</p><p><b>換向回差補償：</b>04已加入方向記憶、左右獨立偏置與離線回算；預設關閉。已獲授權更新並重啟Orin車控，版本支援已確認；重啟後三次LOCKED、兩輪PWM及編碼器增量皆0，舵機輸出關閉。Orin六項補償計算測試通過；實際間隙與落地補償效果尚未驗證。</p><p><b>尚未通過長直線循跡：使用者確認持續右偏、多次壓線。</b>04已集中調參、參數版本、單輪紀錄、路徑回放及現場評語。先核對換胎後中心1360µs、兩輪一致性、修正方向／間隙，再做短輪測試；彎道與車身壓線自動判讀尚未驗證。</p><a href="#drive">前往04調參與紀錄 →</a><details><summary>歷次驗證背景（依時間記錄）</summary><p>04／06已提供視覺PID參數版本、實錄回算、P/I/D分項與命令比較。IMU僅預留時間戳／角速度／品質接口，保持停用。</p><p><b>即時PID接口已部署，尚待架空與落地驗證。</b>04提供v4預覽與限時單輪試跑；目前必須辨識到雙側白線。已驗證v4即時預覽、無可靠路徑拒絕啟動，以及拒絕後LOCKED／兩輪0PWM；42項無硬體測試通過。已獲授權更新車控，沒有燒錄或替換模型。現場已完成3秒零PWM測試：兩輪回報0、無新增編碼器脈波、PID命令約1301µs、限時後LOCKED。跑道幾乎置中，尚未確認左右實際修正方向；失線／急停、地面循跡與彎道仍待驗證。 首輪15%／10秒落地測試於約2.2秒因路徑失效提前停止，最後LOCKED／兩輪0；未通過直線循跡，紀錄已同步05。尚需釐清轉向修正幅度、機構間隙與兩輪實際速度。 最新馬達檢查（10/19 19:13）：最後一輪15%平均右2114／左2124 counts/s，差0.46%；前一輪同15%右2166／左1341，差38.1%。僅確認單輪恢復一致，間歇異常原因未明，不能標示修復；等待確認接線與架空條件變動後再安排PID。 使用者表示按壓左輪後恢復；接續授權架空10／15／20%各5秒均完成（排除起步1秒）：右／左983／948、2174／2218、3185／3238 counts/s，差3.56%、2.00%、1.63%。未復現6700異常，僅確認當次三輪正常，未證明間歇故障消失。 換胎後使用者確認共用中心1360µs，PID版本v7已同步；15%／3秒直線單輪完成，無失線，兩輪計數接近，最後LOCKED／零PWM。影像偏差由+0.075降至約−0.017（正規化值，不是公分）；尚待現場確認行走軌跡，未驗證彎道。換胎前的輪徑與轉向映射需重新核對。 使用者已確認3秒輪直行。接續1360µs／15%／5秒完成，無失線，兩輪計數接近、停止後LOCKED／0PWM；前視偏差由+0.101越過中央至−0.093，尚未證明收斂，需結合接近左彎與現場軌跡判讀，未自動續跑。 後續15%／10秒長直線測試約6.02秒因路徑品質降低提前停止：前視偏差−0.013逐步至−0.384，轉向命令逐步向左降至1315µs，兩輪計數接近；未通過長直線維持，修正實效與換胎後間隙仍需確認。 後續P240／I0／D12、15%／10秒（v11）跑滿10.01秒，兩輪計數接近，命令後段1276µs；影像偏差仍逐步至−0.341，未證明收斂。按時停止不等於循跡通過，待現場確認前輪修正與是否壓線，不自動續跑。</p><a href="#models">前往06離線演算法研究 →</a></details></section>');
 }
 if(route!=='models'||$('pidPanel')||mounting||!$('main').querySelector('.heading'))return;
 mounting=true;const page=route;
 try{
  const [catalog,stored]=await Promise.all([api('/api/pid/catalog'),api('/api/document/visual-pid')]);
  if(route!==page||$('pidPanel'))return;
  const host=document.createElement('section');host.id='pidPanel';host.className='card';
  host.innerHTML=`<span class="pill warn">PID／PD · 實錄回算 · 不輸出硬體 · IMU未使用</span><h2>共用PID：既有影片離線回算</h2><p>從已分析影片的中央路徑，在指定影像列計算偏差：右偏為正、左偏為負。P修正偏差，D抑制偏差變化，I補償長期偏差。初始增益僅供比較，未經實車調整。</p><p class="ribbon">本區引用04保存參數進行離線回算，不會動車；實車單輪入口在04，啟動時讀取已保存版本。回算會沿用原影片，調整PWM不會改變影片中的車速或路徑；不表示車會跑得更好。</p><section class="pid-source"><h3>共用PID參數（唯讀）</h3><p>唯一編輯入口在04；本頁回算使用保存版本，含近遠預判及首次／換向補償。研究提案不會自動取代實車參數。</p><a class="button secondary" href="#drive">到04編輯下一輪參數</a><button id="pidRestore" class="ghost">重新讀取保存版本</button><p id="pidSaveState"></p><pre id="pidSavedParameters"></pre><details><summary>早期06未保存草稿（唯讀保留）</summary><pre id="pidLegacyDraft"></pre></details></section><h3>選實際錄影的分析結果回算</h3><div class="formgrid">${select('pidJob','已完成的影子分析',catalog.jobs.map(j=>[j.id,j.title+' · '+j.id.slice(0,8)]))}<label>辨識模型<select id="pidModel"></select></label>${select('pidLayer','中央路徑來源',['A','B','C'],'A')}</div><p class="note">A／B／C是路徑演算法結果，不是三個PID。選定前視列沒有路徑、時間不連續時，輸出提案歸零；首張影格只初始化。固定1280×720分析座標。</p><div class="actions"><button id="pidRun">以已保存參數回算並留紀錄</button><a class="button secondary" href="#drive">到04查看即時PID試跑</a></div><p id="pidState" role="status"></p><canvas id="pidChart" width="1000" height="260" style="width:100%;background:#f4f6f3;border-radius:8px" aria-label="PID舵機命令時間圖"></canvas><div id="pidRows"></div><details><summary>本次與上次回算比較</summary><p id="pidCompare">同一影片、模型與路徑才適合比較輸出差異；不能推論實車誤差改善。</p></details><button id="pidLoadRuns" class="ghost">讀取已保存回算</button><div id="pidRuns"></div><details><summary>現場啟用前還缺什麼</summary><ol><li>恢復新鮮相機影像，確認白線中央路徑與左右符號。</li><li>架空確認修正方向、限時、影像失效停止；限時自主PID另驗證網頁斷線仍執行至限時、重連只恢復觀測。不可把軟體測試當成實體停止驗收。</li><li>已更新後端接口，未燒錄或替換v4；參數需先保存，單輪執行時凍結版本。</li><li>先不動車檢查，再由你在場確認後低速單次試跑；IMU先保持關閉。</li></ol></details>`;
  $('main').append(host);LastInputs.bind('pid-replay',['pidJob','pidLayer']);let doc=stored,last=null,job=null;
  const connected=()=>host.isConnected;
  const state=message=>{if(connected())$('pidSaveState').textContent=message;};
  const display=()=>{state(`引用04共用參數 v${doc.rev}；回算不送車控`);$('pidSavedParameters').textContent=JSON.stringify(doc.data.parameters,null,2);};
  display();$('pidLegacyDraft').textContent=localStorage.getItem('visual-pid-draft')||'無歷史草稿';
  $('pidRestore').onclick=async()=>{try{const d=await api('/api/document/visual-pid');if(!connected())return;doc=d;display();}catch(e){state(e.message);}};
  async function source(){const id=$('pidJob').value;if(!id){$('pidRun').disabled=true;return;}const d=await api('/api/analysis/result/'+id);if(!connected()||$('pidJob').value!==id)return;job=d;$('pidModel').innerHTML=options(Object.keys(d.models));LastInputs.bind('pid-model:'+id,['pidModel'],document,true);$('pidRun').disabled=false;}
  $('pidJob').onchange=()=>source().catch(e=>state(e.message));await source();if(!connected())return;
  function show(r){
   if(!connected())return;
   $('pidState').textContent=`保存回算 ${r.id.slice(0,8)} · 參數v${r.parameter_revision} · 有效 ${r.valid_frames}/${r.total_frames}；無硬體輸出`;
   const canvas=$('pidChart'),ctx=canvas.getContext('2d'),w=canvas.width,h=canvas.height;ctx.clearRect(0,0,w,h);ctx.font='16px sans-serif';ctx.fillStyle='#243a3b';ctx.fillText('舵機命令提案 µs（空白段＝無有效輸出）',16,23);
   const times=r.rows.map(x=>x.t),start=times[0]??0,span=Math.max(.001,(times.at(-1)??0)-start),p=r.parameters;
   ctx.strokeStyle='#1b7068';ctx.lineWidth=2;ctx.beginPath();let pen=false;for(const x of r.rows){if(!x.valid){pen=false;continue;}const px=60+(x.t-start)/span*(w-85),py=35+(p.max_us-x.steer_us)/(p.max_us-p.min_us)*(h-75);if(pen)ctx.lineTo(px,py);else ctx.moveTo(px,py);pen=true;}ctx.stroke();ctx.fillText(`${p.max_us}`,8,45);ctx.fillText(`${p.min_us}`,8,h-35);ctx.fillText(`${start.toFixed(2)}s → ${(start+span).toFixed(2)}s`,60,h-8);
   pagedList('pidRows',r.rows,x=>`<article class="record-row">影格${x.frame} · ${x.t.toFixed(3)}秒 · ${esc(x.reason)}<p>偏差 ${x.error?.toFixed(3)??'—'} · P/I/D ${[x.p,x.i,x.d].map(v=>v?.toFixed(2)??'—').join(' / ')} · 提案 ${x.steer_us?.toFixed(1)??'停用'} µs · PWM ${x.pwm}% ${x.limited?'（限幅／限速）':''}</p></article>`);
   if(last&&last.analysis_id===r.analysis_id&&last.model===r.model&&last.layer===r.layer){const diffs=r.rows.map((x,i)=>x.valid&&last.rows[i]?.valid?Math.abs(x.steer_us-last.rows[i].steer_us):null).filter(x=>x!==null);$('pidCompare').textContent=`v${last.parameter_revision} → v${r.parameter_revision}；共同有效影格${diffs.length}，平均命令差 ${diffs.length?(diffs.reduce((a,b)=>a+b,0)/diffs.length).toFixed(1):'—'} µs。這不是實車偏差改善。`;}else $('pidCompare').textContent='尚無相同來源的上一筆比較；本次結果與參數已保存。';last=r;
  }
  $('pidRun').onclick=async()=>{const b=$('pidRun');b.disabled=true;try{const latest=await api('/api/document/visual-pid');if(latest.rev!==doc.rev)throw Error('04參數已更新，請先重新讀取保存版本再回算');const r=await api('/api/pid/replay',{job:job.id,model:$('pidModel').value,layer:$('pidLayer').value,revision:doc.rev});show(r);}catch(e){if(connected())$('pidState').textContent=e.message;}finally{if(connected())b.disabled=false;}};
  $('pidLoadRuns').onclick=async()=>{try{const rows=await api('/api/pid/runs');if(!connected())return;pagedList('pidRuns',rows,r=>`<article class="record-row">v${r.parameter_revision} · ${esc(r.model)}／${esc(r.layer)} · ${new Date(r.created*1000).toLocaleString()} <button data-pid-run="${esc(r.id)}">回看</button></article>`);$('pidRuns').onclick=async e=>{const id=e.target.dataset.pidRun;if(id)try{const r=await api('/api/document/pid-run-'+id);show(r.data);}catch(error){state(error.message);}};}catch(e){state(e.message);}};
 }catch(e){notify(e);}finally{mounting=false;if(route!==page)mount();}
}
new MutationObserver(mount).observe($('main'),{childList:true});mount();
})();

/* CURVE_REPLAY_PANEL_V1 */
/* Local replay only. No vehicle endpoints. */
(()=>{
'use strict';
let base='/experiment/curve-v2/';
const $=id=>document.getElementById(id);
let loading=false;
async function request(url,body){
 const r=await fetch(url,body?{method:'POST',headers:{'Content-Type':'application/json','X-Workbench':'1'},body:JSON.stringify(body)}:{});
 const d=await r.json();if(!r.ok)throw Error(d.error||r.status);return d;
}
const defaults={lookahead:1,pwm:20,rate:300,slow:.35,center:1360,initialLeft:30,initialRight:50,reverseLeft:75,reverseRight:90,compensation:1};
function compute(frames,p){
 let last=p.center,offset=0,dir=0,extreme=p.center,previousT=null;
 return frames.map(f=>{
  const dt=previousT===null?0:Math.max(0,Math.min(.5,f.t-previousT));previousT=f.t;
  const target=f.new.path.find(([x,z])=>Math.hypot(x,z)>=p.lookahead);
  if(!f.new.valid||!target){last=p.center;offset=0;dir=0;extreme=p.center;return {valid:false,pwm:0,reason:'沒有足夠可見路徑',t:f.t};}
  const [x,z]=target,k=2*x/(x*x+z*z),a=Math.abs(k);
  // Outside-front-wheel radius inversion is an explicitly uncalibrated scenario.
  const source=k<0?[[1210,2.15],[1120,1.5],[1030,1.065],[940,.825]]:[[1600,1.95],[1750,1.065],[1900,.525]];
  const table=[[0,p.center],...source.map(([u,r])=>[1/(Math.sqrt(r*r-.23*.23)-.155),u])];
  let raw=table.at(-1)[1],reason=a<table[1][0]?'中心附近映射未量測：插值假設':'換胎前外輪映射：幾何暫估';
  const feasible=a<=table.at(-1)[0];
  for(let i=1;i<table.length;i++)if(a<=table[i][0]){const [ka,ua]=table[i-1],[kb,ub]=table[i];raw=ua+(ub-ua)*(a-ka)/(kb-ka);break;}
  let trigger='';
  if(p.compensation){
   if(!dir&&Math.abs(raw-p.center)>=10){dir=Math.sign(raw-p.center);offset=dir<0?-p.initialLeft:p.initialRight;extreme=raw;trigger='首次';}
   else if(dir){if((raw-extreme)*dir>0)extreme=raw;else if((extreme-raw)*dir>=10){dir=-dir;offset=dir<0?-p.reverseLeft:p.reverseRight;extreme=raw;trigger='換向';}}
  }
  const desired=Math.max(940,Math.min(1900,raw+offset));
  last+=Math.max(-p.rate*dt,Math.min(p.rate*dt,desired-last));
  const anticipated=Math.max(a,...f.new.path.filter(([xx,zz])=>Math.hypot(xx,zz)<=2).map(([xx,zz])=>Math.abs(2*xx/(xx*xx+zz*zz))));
  return {valid:feasible,t:f.t,k,raw,desired,us:last,offset,trigger,anticipated,
   pwm:feasible?Math.round(p.pwm/(1+p.slow*anticipated)*10)/10:0,
   reason:feasible?reason:'超出歷史映射範圍：提案停止',target};
 });
}
async function mount(){
 const route=location.hash.slice(1)||'overview',main=$('main');
 if(!main)return;
 if(route==='config'&&!$('curveCalibrationGuide')&&main.querySelector('.heading')&&!window.WBArchive?.hold('curveCalibrationGuide')){
  const box=document.createElement('section');box.className='card';box.id='curveCalibrationGuide';
  box.innerHTML='<details><summary>彎道研究：下一步地面標定準備（尚未完成）</summary><p>保持目前相機固定、車子靜止。以鏡頭正下方的地面投影為原點，沿車頭方向放置50、80、120cm三排標記；每排分別放在車身中線左右20cm處，共六點。位置只是建議，若超出視野則選可見位置並記錄實量。</p><p>拍照前需量出每一點實際的前向／側向距離，保留原始1280×720畫面。我們之後用部分點估算地面投影、其餘點驗證誤差；不能只把近地32cm當作整幅畫面的精準標定。鏡頭畸變明顯時另補鏡頭標定。</p><p>現在不用移動車或更改已保存尺寸。06的幾何仍標示暫估，完成標定後另建版本，不覆蓋昨日紀錄。</p></details>';
  main.append(box);
 }
 if(route==='overview'&&!$('curveChecklist')&&main.querySelector('.heading')){
  const box=document.createElement('section');box.className='card';box.id='curveChecklist';
  box.innerHTML='<h2>彎道路徑研究：下一次現場項目</h2><p>目前僅離線比較，未部署到 Orin、未驗證能過彎。保留 V4 與既有 PID。</p><ol><li>固定相機，量地面標記位置，完成投影標定（目前最近地面32cm、俯角約4.7°仍含估算）。</li><li>換胎後複核中心、左右轉向映射；確認 PCA 故障已解除。</li><li>先零動力核對左右方向，再低速單輪驗證直線入彎、失線停止與車輪餘裕。</li><li>補左彎與長時間單線案例；現有影片以右彎為主。</li></ol><a href="#models">到06查看離線彎道比較 →</a>';
  main.append(box);
 }
 if(route!=='models'||$('curvePanel')||loading||!main.querySelector('.heading'))return;
 loading=true;
 const host=document.createElement('section');host.className='card';host.id='curvePanel';
 host.innerHTML='<h2>彎道與單／雙線 · 離線比較</h2><p role="status">讀取已保存分析…</p>';main.querySelector('.heading').after(host);
 try{
 let [manifest,saved]=await Promise.all([request(base+'manifest.json'),request('/api/document/curve-replay-settings')]);
 if(!host.isConnected)return;
 let doc=saved,result=null,proposals=[],page=0,current=0,p={...defaults,...saved.data.parameters},prior=null,layerData=null;
 host.innerHTML=`<h2>彎道與單／雙線 · 離線比較</h2>
 <p><b>這是錄影回算，不會控制車輛。</b>地面投影與換胎前轉向映射均為暫估；有效路徑比例不是準確率。PWM變更只改命令提案，不會改變錄影中的行進速度。</p>
 <label>後處理版本<select id="curveAlgorithm"><option value="2">v2 · 連續邊界＋對齊後短期記憶</option><option value="1">v1 · 原先逐列推估（保留作比較）</option></select></label>
 <div id="curveLegend" style="padding:16px;background:#f3f6f1;border-left:4px solid #19766e"></div>
 <p id="curveMotionSummary" role="status"></p>
 <p id="curveEvidence"></p>
 <label>實際試跑影片<select id="curveRun"></select></label>
 <details><summary>方法、參考與限制</summary><p>v2沿遮罩骨架追蹤整段邊界，再依影像橫列配對，沒有以未標定的公分寬度排除兩條可見線。歷史點先經LK光流與RANSAC平面對齊；品質不合格即捨棄。保留原觀測時間，最長0.65秒，不將推估當作新觀測。</p><p>這是自訂幾何追蹤器，不是Transformer或RVLD模型。參考影片車道追蹤的時序對齊觀念；尚未接編碼器／陀螺儀融合，也未完成車下盲區地圖。遠處可見邊界會顯示，但超出可信投影範圍不產生公尺尺度控制提案。</p><p>使用原錄影重新解碼與本機V4推論，不能還原停止當刻的即時JPEG、延遲或硬體反應。這裡不作「不壓線」判定。PP轉向採換胎前外輪軌跡假設換算，中心附近缺少量測。</p><a href="https://openaccess.thecvf.com/content/ICCV2023/html/Jin_Recursive_Video_Lane_Detection_ICCV_2023_paper.html" target="_blank" rel="noopener">RVLD：影片車道追蹤</a> · <a href="https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm" target="_blank" rel="noopener">Coulter Pure Pursuit</a></details>
 <details><summary>調整離線控制提案參數（PWM、預看與轉向補償）</summary><div class="formgrid" id="curveFields"></div></details>
 <div class="actions"><button id="curveCalc">套用到離線比較</button><button id="curveSave" class="secondary">保存參數與本次比較</button><button id="curveRestore" class="ghost">恢復已保存值</button></div>
 <details><summary>參數歷史版本</summary><button id="curveHistoryLoad" class="ghost">讀取版本</button><label>版本<select id="curveVersion"></select></label><button id="curveVersionUse" class="ghost">載入版本作離線比較</button><p>載入只改本頁試算；另按保存才追加新版本。</p></details>
 <p id="curveState" role="status"></p><p id="curveSummary"></p>
 <h3>同影格圖層比較</h3><p>每列可選結果、勾選顯示並換色。這三段影片已有同影格的 V4 與三種路徑算法；其他模型尚無對應分析，不能拿不同影片的結果混疊。原9列路徑是快取遮罩重算，並非原即時JPEG結果。</p>
 <div id="curveLayers" style="max-height:280px;overflow:auto"></div>
 <div class="actions"><button class="ghost" id="curveLayerAdd">新增比較圖層</button><button class="ghost" id="curveLayerReset">恢復預設圖層</button></div>
 <p id="curveLayerStatus" role="status"></p>
 <canvas id="curveImage" width="640" height="360" aria-label="同影格原圖與可選模型算法圖層" style="display:block;width:100%;max-width:800px;background:#14272b;border-radius:8px"></canvas>
 <p id="curveDiagnosis"></p>
 <label>影格 <input id="curveFrame" type="range" min="0" value="0" step="1" style="width:100%"></label><p id="curveFrameInfo"></p>
 <canvas id="curveChart" width="960" height="210" style="width:100%;background:#f3f6f1" aria-label="實際舵機紀錄與離線Pure Pursuit命令比較"></canvas>
 <p>深灰＝原實車舵機命令；綠色＝新PP命令提案。兩者不是輪角，也不是新的實車軌跡。降速會參考可見路徑中後軸前方2m內的追蹤曲率；沒有足夠遠處觀測時不補造。</p>
 <div class="actions"><label>每頁 <select id="curveSize"><option>10</option><option>20</option><option>50</option></select> 筆</label><button id="curvePrev" class="ghost">上一頁</button><span id="curvePage"></span><button id="curveNext" class="ghost">下一頁</button></div>
 <div style="max-height:360px;overflow:auto"><table style="min-width:650px;width:100%"><thead><tr><th>秒</th><th>舊／新路徑</th><th>當前雙線／歷史或推估列</th><th>實錄／提案µs</th><th>提案PWM</th><th>限制</th></tr></thead><tbody id="curveRows"></tbody></table></div>
 <details><summary>分析完整性與待驗證事項</summary><p id="curveLimits"></p><p>新停止診斷程式僅在筆電原始碼完成，尚未部署Orin；不能補回昨日遺失的觸發影格。請先完成02相機地面標定，再判定公分尺度與車輪邊界餘裕。</p></details>`;
 const fields=[['lookahead','預看距離 m（後軸原點）',.2,3,.05],['pwm','直線 PWM %',0,25,1],['rate','轉向速度 µs/s',50,800,10],['slow','彎曲降速係數',0,2,.05],['center','中心 µs（情境值）',941,1899,1],['compensation','補償：0關／1開',0,1,1],['initialLeft','首次向左補償 µs',0,200,5],['initialRight','首次向右補償 µs',0,200,5],['reverseLeft','換向左補償 µs',0,200,5],['reverseRight','換向右補償 µs',0,200,5]];
 $('curveFields').innerHTML=fields.map(([key,label,min,max,step])=>`<label>${label}<input id="curve-${key}" type="number" min="${min}" max="${max}" step="${step}" value="${p[key]}"></label>`).join('');
 LastInputs.bind('curve-parameters',fields.map(([k])=>'curve-'+k));
 $('curveLimits').textContent=manifest.limitations.join(' ');
 const fill=()=>{fields.forEach(([k])=>$('curve-'+k).value=p[k]);LastInputs.capture('curve-parameters',fields.map(([k])=>'curve-'+k));};
 function read(){const next={};for(const [k,label,min,max] of fields){const input=$('curve-'+k),v=Number(input.value);if(input.value.trim()===''||!Number.isFinite(v)||v<min||v>max||(k==='compensation'&&![0,1].includes(v)))throw Error(label+' 超出範圍');next[k]=v;}return next;}
 function options(){
  $('curveAlgorithm').value=String(manifest.version);
  $('curveLegend').textContent='模型負責白線像素；路徑算法產生一串中心點；Pure Pursuit從路徑選一個追蹤點。三者分開勾選。下方圖層可同時比較三版；此處版本選單只決定控制提案及PP追蹤點使用哪版路徑。預設黃色是v1路徑，白色圈十字才是PP追蹤點；實際顏色以你的圖層設定為準。';
  $('curveRun').replaceChildren(...manifest.runs.map(r=>{const o=document.createElement('option');o.value=r.id;o.textContent=r.id.slice(0,8)+' · v'+r.parameter_revision+' · '+r.frame_count+'影格';return o;}));
  $('curveLimits').textContent=manifest.limitations.join(' ');
 }
 LastInputs.bind('curve-version',['curveAlgorithm']);if($('curveAlgorithm').value!==String(manifest.version)){base='/experiment/curve-v'+$('curveAlgorithm').value+'/';manifest=await request(base+'manifest.json');if(!host.isConnected)return;}options();LastInputs.bind('curve-view',['curveSize']);LastInputs.bind('curve-source',['curveRun']);
 const catalog={mask:'模型 V4 · 白線遮罩',legacy:'原即時算法 · 9列中間路徑（重算）',v1:'算法 v1 · 中間路徑',v2:'算法 v2 · 中間路徑',boundaries:'算法 v2 · 本格左右邊界',memory:'算法 v2 · 對齊歷史邊界',target:'Pure Pursuit · 單一追蹤點（目前版本／參數）'};
 const defaultLayers=()=>[
  {source:'mask',color:'#dc28dc',show:true},{source:'legacy',color:'#2196ff',show:true},
  {source:'v1',color:'#ffff00',show:true},{source:'v2',color:'#00ee32',show:true},
  {source:'boundaries',color:'#00d8ff',show:false},{source:'memory',color:'#ffaa00',show:false},
  {source:'target',color:'#ffffff',show:true}];
 let layers=defaultLayers(),renderSerial=0,imageCache=new Map();
 try{const stored=JSON.parse(localStorage.getItem('curve-overlay-layers'));if(Array.isArray(stored)&&stored.length<=12&&stored.every(x=>catalog[x.source]&&/^#[0-9a-f]{6}$/i.test(x.color)&&typeof x.show==='boolean'))layers=stored;}catch{}
 function rememberLayers(){try{localStorage.setItem('curve-overlay-layers',JSON.stringify(layers));}catch{}draw();}
 function layerControls(){
  $('curveLayers').replaceChildren(...layers.map((layer,i)=>{
   const row=document.createElement('div');row.style.cssText='display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin:8px 0';
   const toggle=document.createElement('input');toggle.type='checkbox';toggle.checked=layer.show;toggle.setAttribute('aria-label','顯示圖層 '+(i+1));toggle.onchange=()=>{layer.show=toggle.checked;rememberLayers();};
   const select=document.createElement('select');select.style.cssText='flex:1;min-width:220px';select.setAttribute('aria-label','圖層 '+(i+1)+' 結果');
   select.replaceChildren(...Object.entries(catalog).map(([id,name])=>{const o=document.createElement('option');o.value=id;o.textContent=name;return o;}));select.value=layer.source;
   select.onchange=()=>{layer.source=select.value;rememberLayers();};
   const color=document.createElement('input');color.type='color';color.value=layer.color;color.style.cssText='width:52px;height:38px;padding:2px';color.setAttribute('aria-label','圖層 '+(i+1)+' 顏色');color.oninput=()=>{layer.color=color.value;rememberLayers();};
   const remove=document.createElement('button');remove.className='ghost';remove.textContent='移除';remove.setAttribute('aria-label','移除圖層 '+(i+1));remove.onclick=()=>{layers.splice(i,1);layerControls();rememberLayers();};
   row.append(toggle,select,color,remove);return row;
  }));
 }
 function image(url){
  if(!imageCache.has(url)){const promise=new Promise((resolve,reject)=>{const im=new Image();im.onload=()=>resolve(im);im.onerror=()=>reject(Error('無法載入原圖或遮罩'));im.src=url;});imageCache.set(url,promise);if(imageCache.size>8)imageCache.delete(imageCache.keys().next().value);}
  return imageCache.get(url);
 }
 function project([x,z]){
  const g=result.geometry,a=g.pitch_deg*Math.PI/180,zz=z-g.camera_ahead_rear_m,depth=zz*Math.cos(a)+g.height_m*Math.sin(a);
  if(depth<=0)return null;
  return [319.5+320/Math.tan(g.hfov_deg*Math.PI/360)*x/depth,179.5+180/Math.tan(g.vfov_deg*Math.PI/360)*(g.height_m*Math.cos(a)-zz*Math.sin(a))/depth];
 }
 async function draw(){
  if(!layerData||!result)return;
  const ticket=++renderSerial,f=layerData.frames[current],proposal=proposals[current],root='/experiment/curve-layers/'+result.id+'/'+String(f.frame).padStart(6,'0');
  const canvas=$('curveImage'),ctx=canvas.getContext('2d');ctx.clearRect(0,0,640,360);$('curveLayerStatus').textContent='載入同影格圖層…';
  try{
   const [raw,mask]=await Promise.all([image(root+'-raw.jpg'),layers.some(l=>l.show&&l.source==='mask')?image(root+'-mask.png'):Promise.resolve(null)]);
   if(ticket!==renderSerial||!host.isConnected)return;
   ctx.drawImage(raw,0,0);
   function points(list,color,hollow=false){
    ctx.fillStyle=color;ctx.strokeStyle=color;ctx.lineWidth=1.5;
    for(const [x,y] of list){ctx.beginPath();ctx.arc(x,y,hollow?2.5:1.8,0,Math.PI*2);hollow?ctx.stroke():ctx.fill();}
   }
   const active=[];
   for(const l of layers.filter(l=>l.show)){
    let count=0;
    if(l.source==='mask'){
     const off=document.createElement('canvas');off.width=640;off.height=360;const oc=off.getContext('2d');oc.drawImage(mask,0,0);oc.globalCompositeOperation='source-in';oc.fillStyle=l.color;oc.fillRect(0,0,640,360);ctx.globalAlpha=.45;ctx.drawImage(off,0,0);ctx.globalAlpha=1;count=1;
    }else if(l.source==='target'){
     const point=proposal?.target?project(proposal.target):null;
     if(point){const [x,y]=point;ctx.strokeStyle=l.color;ctx.lineWidth=2;ctx.beginPath();ctx.arc(x,y,7,0,Math.PI*2);ctx.moveTo(x-11,y);ctx.lineTo(x+11,y);ctx.moveTo(x,y-11);ctx.lineTo(x,y+11);ctx.stroke();count=1;}
    }else{
     const list=['boundaries','memory'].includes(l.source)?Object.values(f[l.source]).flat():f[l.source];
     points(list,l.color,l.source==='memory');count=list.length;
    }
    active.push(catalog[l.source]+(count?'':'：本格無輸出'));
   }
   $('curveLayerStatus').textContent=active.length?active.join('；'):'只顯示原始影格（所有疊圖關閉）';
   const d=f.diagnostic;
   $('curveDiagnosis').textContent=`v2診斷：左邊界 ${d.left_points} 點，右邊界 ${d.right_points} 點，配對 ${d.paired_rows} 列。`+(!d.valid?`本格不輸出有效中心路徑。${!d.left_points||!d.right_points?'有一側未取得可用邊界；':''}${!d.motion.valid?'歷史對齊也未通過，不能沿用缺失側。':''}目前沒有獨立的單線＋已知寬度中心推估，只有經對齊的短期歷史補足。`:'中心路徑有輸出；仍不代表地面尺度已標定。');
  }catch(e){if(ticket===renderSerial)$('curveLayerStatus').textContent=e.message;}
 }
 layerControls();
 $('curveLayerAdd').onclick=()=>{if(layers.length>=12)return;layers.push({source:'v2',color:'#ff6688',show:true});layerControls();rememberLayers();};
 $('curveLayerReset').onclick=()=>{layers=defaultLayers();layerControls();rememberLayers();};
 function table(){
  const size=Number($('curveSize').value),pages=Math.max(1,Math.ceil(result.frames.length/size));page=Math.min(page,pages-1);
  $('curveRows').innerHTML=result.frames.slice(page*size,(page+1)*size).map((f,j)=>{
   const c=proposals[page*size+j];return `<tr><td>${f.t.toFixed(2)}</td><td>${f.old_valid?'有':'無'}／${f.new.valid?'有':'無'}</td><td>${f.new.double_rows}／${f.new.single_rows}</td><td>${f.actual_us?.toFixed(0)??'—'}／${c.us?.toFixed(0)??'—'}</td><td>${c.pwm}</td><td>${c.reason}</td></tr>`;
  }).join('');$('curvePage').textContent=`${page+1}/${pages} · ${result.frames.length}筆`;$('curvePrev').disabled=page===0;$('curveNext').disabled=page===pages-1;
 }
 function frame(){const f=result.frames[current],c=proposals[current];draw();
  $('curveFrameInfo').textContent=`${f.t.toFixed(2)}秒 · ${c.k===undefined?'無目標':Math.abs(c.k)<.15?'目標接近車頭方向':c.k>0?'向右追蹤':'向左追蹤'} · 雙線列${f.new.double_rows}／推估列${f.new.single_rows} · ${c.reason}。近處目標接近正前方，不代表遠處沒有彎道。`;
  if(f.new.motion){
   const m=f.new.motion,age=Math.max(0,...f.new.rows.map(r=>r.age_s||0));
   $('curveFrameInfo').textContent=`${f.t.toFixed(2)}秒 · 當前雙線 ${f.new.double_rows} 列／歷史補足 ${f.new.single_rows} 列 · 歷史最久 ${age.toFixed(2)}秒 · 對齊${m.valid?'通過':'未通過，僅用當前觀測'}（${m.inliers}/${m.tracks}特徵，${m.reason}） · ${c.reason}`;
  }
 }
 function calculate(){
  p=read();proposals=compute(result.frames,p);table();frame();
  const canvas=$('curveChart'),ctx=canvas.getContext('2d');ctx.clearRect(0,0,960,210);
  const t0=result.frames[0].t,span=Math.max(.01,result.frames.at(-1).t-t0);
  ctx.font='14px sans-serif';ctx.fillStyle='#243a3b';ctx.fillText('1900 µs',5,18);ctx.fillText('940 µs',5,198);
  for(const [color,values] of [['#687478',result.frames.map(f=>f.actual_us)],['#16876e',proposals.map(c=>c.valid?c.us:null)]]){
   ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=2;let drawing=false;
   values.forEach((v,i)=>{if(v==null){drawing=false;return;}const x=80+(result.frames[i].t-t0)/span*870,y=190-(v-940)/960*170;if(drawing)ctx.lineTo(x,y);else ctx.moveTo(x,y);drawing=true;});ctx.stroke();
  }
  $('curveSummary').textContent=`舊路徑可用 ${result.old_valid}/${result.frame_count}；候選 ${result.new_valid}/${result.frame_count}。僅代表產生路徑，非準確率。`+(prior?' 已保存另一組參數可恢復比較。':'');
  $('curveMotionSummary').textContent=manifest.version===2?`此片對齊通過 ${result.motion_valid}/${Math.max(0,result.frame_count-1)} 次相鄰影格；歷史補足 ${result.memory_frames} 影格。錄影紋理重複／模糊時，記憶多數不生效；目前不能宣稱已具可靠的車身周圍地圖。`:'正在查看舊版，沒有相機運動對齊。';
 }
 let seq=0;
 async function load(){const token=++seq,id=$('curveRun').value;const [data,overlays]=await Promise.all([request(base+id+'/result.json'),request('/experiment/curve-layers/'+id+'/layers.json')]);if(!host.isConnected||token!==seq)return;result=data;layerData=overlays;current=0;page=0;$('curveFrame').max=data.frames.length-1;$('curveFrame').value=0;calculate();
  $('curveEvidence').textContent='';
  if(manifest.version===2){
   const evidence=await request(base+'acceptance.json');if(!host.isConnected||token!==seq)return;
   const m=evidence.metrics.find(m=>m.run===id);
   $('curveEvidence').textContent=`離線檢查：${evidence.test_count}項測試通過，${evidence.source_files.length}個來源檔SHA256一致。此片共同有效相鄰影格${m.common_adjacent_pairs}組；y=260附近中心變化P95：v1 ${m.v1_p95_change_px?.toFixed(1)} → v2 ${m.v2_p95_change_px?.toFixed(1)}像素。此量含真實車輛移動，不是準確率或純雜訊，也未證明實車不壓線。`;
  }
 }
 const action=fn=>async()=>{try{await fn();}catch(e){if(host.isConnected)$('curveState').textContent=e.message;}};
 $('curveAlgorithm').onchange=action(async()=>{const id=$('curveRun').value;base='/experiment/curve-v'+$('curveAlgorithm').value+'/';manifest=await request(base+'manifest.json');options();$('curveRun').value=id;await load();$('curveState').textContent='已切換離線算法 v'+manifest.version+'；實車設定未變';});
 $('curveRun').onchange=action(load);$('curveCalc').onclick=action(()=>{calculate();$('curveState').textContent='已更新離線提案；未保存、未套用實車';});
 $('curveSave').onclick=action(async()=>{calculate();prior=doc.data.parameters;doc=await request('/api/document',{key:'curve-replay-settings',rev:doc.rev,data:{overlay_layers:layers,parameters:p,source_run:result.id,source_version:result.source_version,geometry:manifest.geometry,hardware_output:false,calibrated:false}});
  const key='curve-eval-'+crypto.randomUUID();await request('/api/document',{key,rev:0,data:{overlay_layers:layers,parameters:p,parameter_revision:doc.rev,source_run:result.id,source_version:result.source_version,algorithm_version:manifest.version,geometry:manifest.geometry,proposals,hardware_output:false}});
  $('curveState').textContent='已保存獨立離線參數 v'+doc.rev+' 與比較紀錄 '+key.slice(-8)+'；不改PID或實車設定';});
 let versions=[];
 $('curveHistoryLoad').onclick=action(async()=>{versions=await request('/api/history/curve-replay-settings');$('curveVersion').replaceChildren(...versions.map(v=>{const o=document.createElement('option');o.value=v.rev;o.textContent='v'+v.rev+' · PWM '+v.data.parameters.pwm+' · 預看 '+v.data.parameters.lookahead+'m';return o;}));});
 $('curveVersionUse').onclick=action(()=>{const v=versions.find(v=>String(v.rev)===$('curveVersion').value);if(!v)throw Error('請先讀取版本');p={...defaults,...v.data.parameters};fill();calculate();$('curveState').textContent='試算使用歷史 v'+v.rev+'，尚未保存新版本';});
 $('curveRestore').onclick=action(async()=>{doc=await request('/api/document/curve-replay-settings');p={...defaults,...doc.data.parameters};if(Array.isArray(doc.data.overlay_layers)&&doc.data.overlay_layers.every(l=>catalog[l.source]&&/^#[0-9a-f]{6}$/i.test(l.color))){layers=doc.data.overlay_layers.map(l=>({...l}));layerControls();rememberLayers();}fill();calculate();$('curveState').textContent='已恢復保存版本 v'+doc.rev;});
 $('curveFrame').oninput=()=>{current=Number($('curveFrame').value);frame();};
 $('curveSize').onchange=()=>{page=0;table();};$('curvePrev').onclick=()=>{page--;table();};$('curveNext').onclick=()=>{page++;table();};
 $('curveState').textContent='獨立離線參數版本 v'+doc.rev;await load();
 }catch(e){if(host.isConnected){const p=document.createElement('p');p.textContent='分析尚未就緒：'+e.message;host.append(p);}}
 finally{loading=false;}
}
new MutationObserver(mount).observe(document.getElementById('main'),{childList:true});
window.addEventListener('hashchange',()=>setTimeout(mount,0));mount();
})();

/* BEND_CASES_PANEL_V1 */
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
 LastInputs.bind('bend-view',['bendCase','bendSpeed']);
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

/* CAMERA_MEASURE_PANEL */
// Perpendicular pixel distance of the 80 cm endpoint to the 50–120 cm line.
// This is an image-space consistency check, not a metric calibration threshold.
function cameraCollinearity(lines,side){
 const points=[50,80,120].map(cm=>lines.find(l=>l.cm===cm)?.[side]);
 if(points.some(p=>!Array.isArray(p)||p.length!==2||!p.every(Number.isFinite)))return null;
 const [a,b,c]=points,dx=c[0]-a[0],dy=c[1]-a[1],length2=dx*dx+dy*dy;
 if(length2<1e-12)return null;
 const t=((b[0]-a[0])*dx+(b[1]-a[1])*dy)/length2;
 const projection=[a[0]+t*dx,a[1]+t*dy];
 return {points,projection,deviation:Math.hypot(b[0]-projection[0],b[1]-projection[1])};
}
/* Camera measurement UI: snapshot analysis and documents only; no actuator API. */
(()=>{
'use strict';
const $=id=>document.getElementById(id);
let busy=false,cleanup=()=>{};
async function api(path,body){const r=await fetch(path,body?{method:'POST',headers:{'X-Workbench':'1','Content-Type':'application/json'},body:JSON.stringify(body)}:{});const d=await r.json();if(!r.ok)throw Error(d.error||r.status);return d;}
async function mount(){
 if(location.hash!=='#config'){cleanup();return;}
 if(busy||$('cameraMeasure')||!document.getElementById('main').querySelector('.heading'))return;busy=true;
 const host=document.createElement('section');host.id='cameraMeasure';host.className='card';
 host.innerHTML=`<h2>相機地面量測：拖曳距離參考線</h2>
 <p>重測時以前輪輪胎最前緣為0，沿地面量出50、80、120 cm；每一條線的左右端點須落在同一距離的橫向截面及左右白線<b>內緣</b>。拖線身移動整條線，拖圓點個別調整。紫色只是V4辨識區域，不是內緣真值。</p>
 <p><b>這是人工對位草稿，不是已完成標定。</b>初始線位置僅方便操作；拖曳不會改變實車參數。車輛與相機須保持不動。三條線本身不足以確定完整地面座標，仍需確認橫向位置及另留驗證點。</p>
 <div class="actions"><button id="cmCapture">拍攝凍結快照並辨識白線</button><button id="cmLive" class="ghost">開啟即時調整</button><button id="cmRestore" class="ghost">恢復最後保存</button></div>
 <label>預覽目標幀率<select id="cmFps"><option value="1">1 FPS</option><option value="2" selected>2 FPS</option><option value="5">5 FPS</option><option value="10">10 FPS</option><option value="15">15 FPS</option><option value="0">不限（上一張到就取下一張）</option></select></label><p>即時模式直接在下方畫面拖曳三條線；不疊舊遮罩。拍攝凍結辨識後保留線的位置，再確認保存。幀率受連線速度限制。</p>
 <p id="cmStatus" role="status">尚未拍攝；參考線尚未量測。</p>
 <label><input type="checkbox" id="cmMask" checked>顯示本張快照的紫色白線辨識</label>
 <canvas id="cmCanvas" width="640" height="360" style="width:100%;max-width:960px;display:block;touch-action:none;background:#18252b" aria-label="可拖曳50、80、120公分量測線的凍結相機畫面"></canvas>
 <p>黃／青／橙橫線＝50／80／120 cm的待對位位置；綠色左側、藍色右側折線＝各側三個端點連線；同色虛線＝50至120 cm兩端點的直線參考，短白線＝80 cm端點到參考線的垂直偏差。亦可用下方原圖像素欄位微調。</p>
 <div id="cmGeometry" role="status" aria-live="polite"></div>
 <p>偏差以原圖像素（px）計算，不隨畫面放大改變。只適用於實際為直線的道路內緣；偏差小不代表距離正確，偏差大也可能來自道路彎曲或鏡頭畸變。沒有設定「合格」門檻，不會自動拉直或阻擋保存。</p>
 <details><summary>原本三條線怎麼來的／明天重測步驟</summary><p id="cmSource"></p><p>未載入保存資料時，640×360示意座標為50 cm：(100,300)→(540,300)；80 cm：(160,235)→(480,235)；120 cm：(220,170)→(420,170)。這是寫死的操作起點，不是相機內參、量尺讀值、跑道寬度或V4推算出的公分投影。</p><ol><li>固定車輛與相機，確認量尺零點及三個距離的實體橫向標記。</li><li>拍攝新的凍結快照；舊線只供開始拖曳，不沿用為新量測結果。</li><li>依實體標記對位六個端點，再看左右共線偏差；不要為了歸零而改掉真實量測點。</li><li>逐條確認後保存新修訂，舊修訂保留。</li></ol></details>
 <div id="cmRows" style="max-height:350px;overflow:auto"></div>
 <label>量尺零點／相機姿態／補充說明<input id="cmNote" placeholder="記錄前輪輪胎最前緣零點、相機姿態與量測日期"></label>
 <div class="actions"><button id="cmSave">保存本張快照與量測草稿</button></div>
 <p id="cmSaved"></p>`;
 document.getElementById('main').querySelector('.heading').after(host);
 let savedMetadata={},data=null,raw=null,mask=null,rev=0,live=false,timer=null,poll=null,dead=false,drag=null,dirty=false,previewBusy=false,epoch=0,lastFrame=0;
 let lines=[50,80,120].map((cm,i)=>({cm,left:[100+i*60,300-i*65],right:[540-i*60,300-i*65],confirmed:false}));
 const colors=['#ffe600','#00e5ff','#ff8b33'];
 cleanup=()=>{dead=true;live=false;epoch++;clearTimeout(timer);clearTimeout(poll);};
 const canvas=$('cmCanvas'),ctx=canvas.getContext('2d');
 $('cmSource').textContent='目前為程式示意座標；尚未載入保存草稿。';
 function draw(){
  ctx.clearRect(0,0,canvas.width,canvas.height);if(!raw)return;ctx.drawImage(raw,0,0);
  if(mask&&$('cmMask').checked){const c=document.createElement('canvas');c.width=canvas.width;c.height=canvas.height;const x=c.getContext('2d');x.drawImage(mask,0,0);x.globalCompositeOperation='source-in';x.fillStyle='#d92de0';x.fillRect(0,0,c.width,c.height);ctx.globalAlpha=.4;ctx.drawImage(c,0,0);ctx.globalAlpha=1;}
  ctx.font=`bold ${Math.max(14,canvas.width/45)}px sans-serif`;
  lines.forEach((l,i)=>{ctx.strokeStyle=colors[i];ctx.fillStyle=colors[i];ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(...l.left);ctx.lineTo(...l.right);ctx.stroke();
   for(const p of [l.left,l.right]){ctx.beginPath();ctx.arc(...p,7,0,Math.PI*2);ctx.fill();}
   ctx.fillText(`${l.cm} cm${l.confirmed?' 已確認':' 待對位'}`,Math.min(l.left[0]+10,canvas.width-160),Math.max(20,l.left[1]-12));});
  const hints=[];
  for(const [side,label,color] of [['left','左側','#63ff88'],['right','右側','#56b7ff']]){
   const result=cameraCollinearity(lines,side);
   if(!result){hints.push(label+'：無法計算（端點重合或資料不完整）');continue;}
   const {points,projection,deviation}=result;
   ctx.strokeStyle=color;ctx.lineWidth=2;ctx.setLineDash([]);
   ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.stroke();
   ctx.setLineDash([6,4]);ctx.beginPath();ctx.moveTo(...points[0]);ctx.lineTo(...points[2]);ctx.stroke();
   ctx.setLineDash([]);ctx.strokeStyle='#fff';ctx.beginPath();ctx.moveTo(...points[1]);ctx.lineTo(...projection);ctx.stroke();
   hints.push(`${label}共線偏差：${deviation.toFixed(1)} px`);
  }
  $('cmGeometry').textContent=hints.join('；')+'。比較80 cm端點與50–120 cm參考直線；0 px僅代表三點共線，不能證明公分標記正確。';
 }
 function rows(){
  $('cmRows').replaceChildren(...lines.map((l,i)=>{
   const box=document.createElement('div');box.style.cssText='border-left:4px solid '+colors[i]+';padding:10px;margin:8px 0';
   const title=document.createElement('b');title.textContent=l.cm+' cm ';box.append(title);
   for(const side of ['left','right'])for(const axis of [0,1]){
    const label=document.createElement('label');label.style.cssText='display:inline-block;margin:4px';
    label.append(document.createTextNode((side==='left'?'左':'右')+(axis?' y':' x')+' '));
    const input=document.createElement('input');input.type='number';input.step='1';input.min='0';input.max=String((axis?canvas.height:canvas.width)-1);input.value=Math.round(l[side][axis]);input.style.width='80px';input.setAttribute('aria-label',l.cm+'cm '+(side==='left'?'左':'右')+(axis?' y':' x'));
    input.oninput=()=>{const n=Number(input.value);if(!Number.isFinite(n)||input.value==='')return;l[side][axis]=Math.max(0,Math.min(Number(input.max),n));l.confirmed=false;dirty=true;draw();box.querySelector('input[type=checkbox]').checked=false;};label.append(input);box.append(label);
   }
   const label=document.createElement('label'),check=document.createElement('input');check.type='checkbox';check.checked=l.confirmed;check.onchange=()=>{l.confirmed=check.checked;dirty=true;draw();};label.append(check,document.createTextNode('我已用量尺確認此線距離'));box.append(label);return box;
  }));
 }
 function point(e){const b=canvas.getBoundingClientRect();return [(e.clientX-b.left)*canvas.width/b.width,(e.clientY-b.top)*canvas.height/b.height];}
 canvas.onpointerdown=e=>{
  if(!raw)return;const p=point(e),tolerance=20*canvas.width/canvas.getBoundingClientRect().width;
  let best=null,dist=tolerance;
  lines.forEach((l,i)=>{for(const side of ['left','right']){const d=Math.hypot(p[0]-l[side][0],p[1]-l[side][1]);if(d<dist){dist=d;best={i,side};}}
   const a=l.left,b=l.right,dx=b[0]-a[0],dy=b[1]-a[1],u=Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy||1))),d=Math.hypot(p[0]-a[0]-u*dx,p[1]-a[1]-u*dy);
   if(!best&&d<dist){dist=d;best={i,side:'both'};}});
  if(best){drag={...best,p,left:[...lines[best.i].left],right:[...lines[best.i].right]};canvas.setPointerCapture(e.pointerId);}
 };
 canvas.onpointermove=e=>{if(!drag)return;const p=point(e),l=lines[drag.i],clamp=(v,max)=>Math.max(0,Math.min(max-1,v));
  if(drag.side==='both'){let dy=p[1]-drag.p[1];dy=Math.max(-Math.min(drag.left[1],drag.right[1]),Math.min(canvas.height-1-Math.max(drag.left[1],drag.right[1]),dy));let dx=p[0]-drag.p[0];dx=Math.max(-Math.min(drag.left[0],drag.right[0]),Math.min(canvas.width-1-Math.max(drag.left[0],drag.right[0]),dx));l.left=[drag.left[0]+dx,drag.left[1]+dy];l.right=[drag.right[0]+dx,drag.right[1]+dy];}
  else l[drag.side]=[clamp(p[0],canvas.width),clamp(p[1],canvas.height)];
  l.confirmed=false;dirty=true;draw();
 };
 canvas.onpointerup=canvas.onpointercancel=()=>{drag=null;rows();};
 function image(url){return new Promise((resolve,reject)=>{const im=new Image();im.onload=()=>resolve(im);im.onerror=()=>reject(Error('快照載入失敗'));im.src=url;});}
 async function show(d,restored){
  const root='/experiment/camera-measure/'+d.id+'/';
  const ims=await Promise.all([image(root+'raw.jpg'),image(root+'mask.png')]);if(dead)return;
  data=d;[raw,mask]=ims;canvas.width=d.width;canvas.height=d.height;
  if(!restored){lines.forEach(l=>l.confirmed=false);$('cmSource').textContent='新快照沿用上一組端點作為拖曳起點；三條線已重設為待對位，尚未依這張照片重新量測。';}
  rows();draw();$('cmStatus').textContent=`凍結快照：${new Date(d.received_at*1000).toLocaleString()}（筆電收到時間，非曝光時間）· ${d.width}×${d.height} · 紫色與原圖為同一張V4分析。`;
 }
 async function watch(){
  try{const d=await api('/api/camera-measure/status');if(dead)return;
   if(d.state==='running'){poll=setTimeout(watch,1000);return;}
   $('cmCapture').disabled=false;$('cmLive').disabled=false;$('cmRestore').disabled=false;$('cmSave').disabled=false;
   if(d.state==='failed')throw Error(d.error);
   if(d.state==='completed'){await show(d,false);dirty=true;}
  }catch(e){if(!dead){$('cmStatus').textContent=e.message;$('cmCapture').disabled=false;$('cmLive').disabled=false;$('cmRestore').disabled=false;$('cmSave').disabled=false;}}
 }
 $('cmCapture').onclick=async()=>{
  stopLive();data=null;mask=null;$('cmLive').disabled=true;$('cmRestore').disabled=true;$('cmSave').disabled=true;
  $('cmCapture').disabled=true;$('cmStatus').textContent='正在取得快照並在筆電執行V4；不啟動車控…';
  try{await api('/api/camera-measure/capture',{});await watch();}catch(e){$('cmStatus').textContent=e.message;$('cmCapture').disabled=false;$('cmLive').disabled=false;$('cmRestore').disabled=false;$('cmSave').disabled=false;}
 };
 function stopLive(){live=false;epoch++;clearTimeout(timer);$('cmLive').textContent='開啟即時調整';}
 async function preview(){
  if(!live||dead||document.hidden||previewBusy)return;
  previewBusy=true;const token=epoch,start=performance.now();
  try{const im=await image('/orin-camera.jpg?t='+Date.now());
   if(token!==epoch||dead||!live)return;
   if(canvas.width!==im.naturalWidth||canvas.height!==im.naturalHeight){
    const sx=im.naturalWidth/canvas.width,sy=im.naturalHeight/canvas.height;
    lines.forEach(l=>{for(const side of ['left','right'])l[side]=[l[side][0]*sx,l[side][1]*sy];});
    canvas.width=im.naturalWidth;canvas.height=im.naturalHeight;rows();
   }
   raw=im;mask=null;data=null;draw();const now=performance.now();
   $('cmStatus').textContent=`即時調整 · 目標 ${Number($('cmFps').value)>0?$('cmFps').value+' FPS':'不限'} · 實際 ${lastFrame?(1000/(now-lastFrame)).toFixed(1):'—'} FPS · ${new Date().toLocaleTimeString()} 收到影像 · 紫色遮罩暫停，三條線可直接拖曳。`;lastFrame=now;
  }catch(e){if(token===epoch&&!dead)$('cmStatus').textContent='相機更新失敗，畫面停留在上一張：'+e.message;}
  finally{previewBusy=false;if(live&&!dead)timer=setTimeout(preview,Math.max(0,(Number($('cmFps').value)>0?1000/Number($('cmFps').value):0)-(performance.now()-start)));}
 }
 LastInputs.bind('camera-view',['cmFps','cmMask']);
 $('cmLive').onclick=()=>{if(live){stopLive();$('cmStatus').textContent='已暫停預覽；請拍攝凍結辨識後保存。';}else{live=true;epoch++;lastFrame=0;mask=null;data=null;lines.forEach(l=>l.confirmed=false);rows();draw();$('cmLive').textContent='暫停即時調整';preview();}};
 $('cmFps').onchange=()=>{clearTimeout(timer);if(live)preview();};
 const visibility=()=>{if(!document.hidden&&live)preview();};document.addEventListener('visibilitychange',visibility);
 const stop=cleanup;cleanup=()=>{stop();document.removeEventListener('visibilitychange',visibility);};
 $('cmMask').onchange=draw;
 async function restore(){
  stopLive();
  const doc=await api('/api/document/camera-ground-measure');if(dead)return;rev=doc.rev;savedMetadata=doc.data||{};
  if(doc.data?.snapshot){lines=doc.data.lines;$('cmNote').value=doc.data.note||'';await show(doc.data.snapshot,true);dirty=false;$('cmSaved').textContent='已恢復草稿版本 '+rev;$('cmSource').textContent=`目前顯示保存草稿 v${rev} 的原圖像素座標；不是重新推算的投影。確認狀態沿用該修訂，明天重測請先拍新快照。`;}
 }
 $('cmRestore').onclick=()=>restore().catch(e=>$('cmSaved').textContent=e.message);
 $('cmSave').onclick=async()=>{
  if(live||!data){$('cmSaved').textContent='請先按「拍攝凍結快照並辨識白線」，再確認保存；參考線位置會保留。';return;}
  if(lines.some(l=>l.left[0]>=l.right[0])){$('cmSaved').textContent='左端點須位於右端點左側，請檢查。';return;}
  try{const d=await api('/api/document',{key:'camera-ground-measure',rev,data:{...savedMetadata,snapshot:data,lines,note:$('cmNote').value,origin:savedMetadata.origin||'量測起點待確認',endpoint_reference:'白線內緣（人工對位）',metric_calibrated:false,hardware_output:false}});rev=d.rev;dirty=false;$('cmSaved').textContent=`已保存版本 ${rev}；${lines.filter(l=>l.confirmed).length}/3 條已由你確認。仍是標定草稿，未套用車控。`;}catch(e){$('cmSaved').textContent=e.message;}
 };
 try{await restore();}catch(e){$('cmSaved').textContent=e.message;}
 busy=false;
}
new MutationObserver(mount).observe(document.getElementById('main'),{childList:true});
window.addEventListener('hashchange',mount);mount();
})();

/* GROUND_CALIBRATION_PANEL */
/* Saved candidate report. Reads local measurements; no hardware output. */
(()=>{
let loading=false;
async function mount(){
 if(location.hash!=='#config'||loading||document.getElementById('groundCandidate')||!document.getElementById('cameraMeasure')||window.WBArchive?.hold('groundCandidate'))return;
 loading=true;
 const host=document.createElement('section');host.id='groundCandidate';host.className='card';
 document.getElementById('cameraMeasure').after(host);
 try{
  const base='/experiment/ground-calibration/';
  const get=async p=>{const r=await fetch(p);if(!r.ok)throw Error('尚無候選投影報告');return r.json();};
  const index=await get(base+'latest.json'),root=base+index.folder+'/';
  const r=await get(root+'result.json'),current=await get('/api/document/camera-ground-measure');
  if(!host.isConnected)return;
  host.innerHTML=`<h2>地面投影候選與獨立距離驗證</h2>
  <p><b>已建立暫定投影；尚未通過精度驗證，未套用實車。</b></p>
  <p id="gcVersion"></p><p id="gcCoordinates"></p>
  <div style="overflow:auto"><table><thead><tr><th>檢查</th><th>結果</th><th>解讀</th></tr></thead><tbody>
  <tr><td>六端點擬合誤差</td><td>${r.fit.rmse_cm.toFixed(2)} cm RMS</td><td>只衡量貼合輸入，不是實際準確率</td></tr>
  <tr><td>100 cm棒子（未參與擬合）</td><td>${r.validation.predicted_cm.toFixed(1)} cm；差 ${r.validation.error_cm.toFixed(1)} cm</td><td>近側邊緣的人工選點；擺放位置與車身未移動仍需確認</td></tr>
  ${r.held_out_rows.map(x=>`<tr><td>不使用${x.distance_cm} cm線，改用另兩線推算</td><td>最大端點差 ${x.max_error_cm.toFixed(1)} cm</td><td>${x.distance_cm===80?'內插一致性檢查':'外插較敏感；不是判定此線量錯'}</td></tr>`).join('')}
  </tbody></table></div>
  <p>棒子選點上下1像素對應 ${r.validation.pixel_only_range_cm.map(x=>x.toFixed(1)).join('～')} cm；這不是完整信賴區間，未包含量尺±3 cm、線端點、寬度、相機移動及畸變誤差。</p>
  <div style="display:flex;flex-wrap:wrap;gap:16px;align-items:start">
   <figure style="margin:0;flex:2;min-width:260px"><img src="${root}validation.jpg" style="width:100%;max-width:640px" alt="100公分棒子獨立驗證原圖"><figcaption>驗證原圖；棒子近側在量尺附近，人工讀點。</figcaption></figure>
   <figure style="margin:0;flex:1;min-width:180px"><img src="${root}validation-top.png" style="width:100%;max-width:320px" alt="候選地面俯視圖，紅圈標示棒子估算位置"><figcaption>候選俯視圖：紅圈為棒子選點；黑區未驗證，不補畫道路。</figcaption></figure>
  </div>
  <details><summary>使用假設與六端點誤差</summary><ul id="gcAssumptions"></ul><p>內緣淨寬使用46 cm；80 cm截面已由使用者確認，誤差±0.5 cm。圖中橫向0為跑道中心，不表示鏡頭或車身就在中央；尚未換算後軸控制座標。</p><a href="${root}result.json" target="_blank" rel="noopener">查看矩陣與完整數據</a></details>
  <h3>你接下來只需複核這些</h3>
  <ol><li>確認棒子靠近車子的一側仍是前輪最前緣往前100 cm，且拍攝兩張照片間沒有移動車或相機。</li>
  <li>優先複核120 cm：兩端是否真在同一條實體橫向截面、是否都對準白線內緣。再看50 cm與80 cm是否也如此；不要為了讓數字通過而拉線。</li>
  <li>已完成：80 cm截面內緣淨寬46±0.5 cm，不必重複量測。</li></ol>
  <p>若以上皆正確，就保留目前誤差記錄，再安排額外平面點或鏡頭畸變校正；不要求你反覆換棒子。修改後先保存量測並告知，候選投影需要重新計算，不會自動改動控制器。</p>`;
  host.querySelector('#gcVersion').textContent=`來源量測 v${r.source_revision}；目前保存 v${current.rev}。${r.confirmation_note||""}。`+(current.rev!==r.source_revision?'資料已更新，此報告已過期，請重新計算。':'來源版本一致。')+' 驗證照片為保存快照，不是即時畫面。';
  host.querySelector('#gcCoordinates').textContent=r.coordinate_system;
  host.querySelector('#gcAssumptions').replaceChildren(...r.assumptions.map(a=>{const li=document.createElement('li');li.textContent=a;return li;}));
  try{
   const effect=await get(root+'paths-result.json'),m=effect.metrics;
   const panel=document.createElement('div');
   panel.innerHTML=`<h3>套用本組投影：單線與雙線效果</h3>
   <p><b>橘色＝只沿左白線內緣，向內23 cm；青色＝只沿右白線內緣，向內23 cm；綠色＝雙線中心。</b>紫色為原V4白線遮罩。</p>
   <p>左右單線推算中心平均相差 <b>${m.single_path_separation_mean_cm.toFixed(1)} cm</b>，最大 ${m.single_path_separation_max_cm.toFixed(1)} cm。兩者尚未一致；不能視為可實車循跡。</p>
   <p>辨識內緣的投影寬度 ${m.projected_inner_width_range_cm.map(x=>x.toFixed(1)).join('～')} cm，與實量46±0.5 cm不一致。可能涉及人工端點、分割內緣或投影；此結果尚不能分離原因。</p>
   <div style="display:flex;gap:16px;flex-wrap:wrap;align-items:start">
   <img src="${root}paths-overlay.png" alt="橘色左線推算、青色右線推算、綠色雙線中心疊圖" style="width:100%;max-width:640px">
   <img src="${root}paths-top.png" alt="單線雙線候選路徑俯視比較" style="width:240px;max-width:100%"></div>
   <p>本張有效比較範圍：前輪最前緣往前 ${m.compared_z_cm.map(x=>x.toFixed(1)).join('～')} cm。三條路徑中心都在觀測走廊內，並未出現偏移折返；這不保證整個車身不壓線。</p>
   <p>只檢查當前直線快照，沒有驗證連續影格穩定性或過彎。量尺／棒子也可能被V4辨識成白線，不能直接把全部紫色區域當成賽道邊界。這份投影不適用調整相機前的舊影片。</p>
   <a href="${root}paths-result.json" target="_blank" rel="noopener">查看路徑與量化結果</a>`;
   host.querySelector('h3').before(panel);
  }catch{/* Candidate may not yet have an effects report. */}
 }catch(e){host.textContent='候選投影：'+e.message;}finally{loading=false;}
}
new MutationObserver(mount).observe(document.getElementById('main'),{childList:true});
window.addEventListener('hashchange',mount);mount();
})();
