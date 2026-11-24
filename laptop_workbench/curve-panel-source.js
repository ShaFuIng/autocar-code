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
 if(route==='config'&&!$('curveCalibrationGuide')&&main.querySelector('.heading')){
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
 $('curveLimits').textContent=manifest.limitations.join(' ');
 const fill=()=>fields.forEach(([k])=>$('curve-'+k).value=p[k]);
 function read(){const next={};for(const [k,label,min,max] of fields){const input=$('curve-'+k),v=Number(input.value);if(input.value.trim()===''||!Number.isFinite(v)||v<min||v>max||(k==='compensation'&&![0,1].includes(v)))throw Error(label+' 超出範圍');next[k]=v;}return next;}
 function options(){
  $('curveAlgorithm').value=String(manifest.version);
  $('curveLegend').textContent='模型負責白線像素；路徑算法產生一串中心點；Pure Pursuit從路徑選一個追蹤點。三者分開勾選。下方圖層可同時比較三版；此處版本選單只決定控制提案及PP追蹤點使用哪版路徑。預設黃色是v1路徑，白色圈十字才是PP追蹤點；實際顏色以你的圖層設定為準。';
  $('curveRun').replaceChildren(...manifest.runs.map(r=>{const o=document.createElement('option');o.value=r.id;o.textContent=r.id.slice(0,8)+' · v'+r.parameter_revision+' · '+r.frame_count+'影格';return o;}));
  $('curveLimits').textContent=manifest.limitations.join(' ');
 }
 options();
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
