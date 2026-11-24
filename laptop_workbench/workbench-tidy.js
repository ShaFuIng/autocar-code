'use strict';
// 舊展示歸檔、各頁歷史索引與06「目前方法」卡（2026-10-31）。
// 歸檔只決定是否掛載：各卡片的掛載函式在讀資料前先問 WBArchive.hold(id)。
// 原始資料、程式、API與卡片ID都保留；從頁面底部索引按需載入，只在本次瀏覽有效。
window.WBArchive=(()=>{
 const groups={
  current:'目前路口方法的舊版與基準（10/30–10/31）',
  old06:'舊控制分支與路口演進（10/28–10/30）',
  k:'K系列與早期十字路口（K歷史資料）',
  warm:'暖光／隧道候選',
  data:'資料集與標註',
  report:'舊驗收與接入報告',
  dup:'重複入口（主位置在05）',
  dev:'開發測試（只作用於筆電模擬器）',
  config:'舊相機投影／標定測試',
  field:'歷史影片'
 };
 // [頁面, 卡片ID, 群組, 名稱, 模式]；模式hide＝卡片內容不讀資料，只收起。
 const items=[
  ['models','entranceReferenceReview','current','手推雙路口核對 r2 · 入口方向與路寬'],
  ['models','straightEntryReview','current','直行入口 r1 · 分段出口比較'],
  ['models','widthContinuityPlayer','current','路寬連續與出彎比較 r1'],
  ['models','postR2Analysis','current','r2兩輪：出彎與直線擺盪分析'],
  ['models','previewTurnR2Report','old06','獨立線估計 v2.1 · 彎中持續轉向'],
  ['models','previewTurnReport','old06','獨立線估計 v2 · 提前轉向'],
  ['models','standalonePIDReport','old06','獨立線估計 v1 · 可選PID候選'],
  ['models','outwardCompare','old06','彎道外移探索 · 原版／5／10px'],
  ['models','cpuPriority','old06','控制發布優先 · CPU優化比較'],
  ['models','corridorMemoryReport','old06','新相機角度彙整：畫面外中心r2、69088、雙邊界r1、相機下壓四輪、近遠共同目標、四輪回授、回授收集、路口雙邊界、單目標方向、即時綠線v1～v1.2'],
  ['models','threeZoneReview','old06','近中遠連續選線 r1 · 離線判讀'],
  ['models','straightV3Report','old06','獨立線估計 v3 · 下一岔口直行一次'],
  ['models','archReplay','old06','架構檢討：完整版／消融／簡化循線與人工判讀'],
  ['models','kAdaptiveReport','k','完整K · 動態PID取樣 r1'],
  ['models','approachReport','k','完整K · 路口接近時序 r1'],
  ['models','branchTemporalReport','k','第二入口 · 分支時序核對'],
  ['models','integratedKRelease','k','完整K r2 · 即時入口整合版本'],
  ['models','deployedGeometry','k','十字路口測試版（10/23當時標為「最終」）'],
  ['models','geometryGuardReport','k','第438～442格 · 邊界分段與拼接'],
  ['models','geometryFullReview','k','0:00～2:56長影片 · 95格PID差異'],
  ['models','selectedChainCompare','k','路口修正版 · 可信路段保留／接回A／K'],
  ['models','historyGapCompare','k','歷史對齊補缺口'],
  ['models','fullCurveHistory','k','完整影片彎道 · A帶歷史／局部重置'],
  ['models','junctionCases','k','十字路口專項 A～K 版（含預期出口人工標註）'],
  ['models','visibleFusion','k','可見融合 v1～v3'],
  ['models','tunnelReferenceReport','warm','隧道暖光 · 遠雙線核對'],
  ['models','warmNoLabelCompare','warm','暖光免標註校色'],
  ['models','warmSmoothCompare','warm','暖光平滑校色併入K前比較'],
  ['models','newRecordingsCompare','data','10/24新錄兩路口與斜坡 · 原版／完整K'],
  ['models','cvatPackages1024','data','10/24補標 · 四個CVAT原圖包'],
  ['models','v7PidDetails','report','校正v7 → 路徑算法 → 視覺PID（舊接入檢討）'],
  ['models','pidOfflineDetails','report','新路徑算法：離線與假控制端接入驗證'],
  ['models','fieldTrialPanel','dup','合成路徑研究實驗（與05「抽樣／合成試驗」相同）'],
  ['drive','developerTools','dev','假硬體任務、故障注入與舊示範預覽','hide'],
  ['drive','fieldTrialPanel','dup','現場唯讀抽樣紀錄（與05相同）'],
  ['config','groundCandidate','config','地面投影候選與獨立距離驗證'],
  ['config','curveCalibrationGuide','config','彎道研究：地面標定準備（舊計畫）'],
  ['config','legacyConfig','config','早期配置與預覽設定（唯讀歷史）','hide'],
  ['replay','fieldArchive1024','field','10/24離場保存 · 筆電本機影片']
 ];
 const shown=new Set();let indexOpen=false;
 const page=()=>location.hash.slice(1)||'overview';
 const find=id=>items.find(i=>i[0]===page()&&i[1]===id);
 const esc=s=>String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
 function hold(id){return !!find(id)&&!shown.has(page()+':'+id);}
 function focus(id){
  const n=document.getElementById(id);if(!n)return false;
  n.hidden=false;for(let p=n;p;p=p.parentElement)if(p.tagName==='DETAILS')p.open=true;
  n.scrollIntoView({block:'start'});return true;
 }
 function reveal(id){
  const it=find(id);if(!it)return;shown.add(page()+':'+id);indexOpen=true;
  if(it[4]==='hide'){focus(id);renderIndex();return;}
  // 重新繪製本頁，讓原本的掛載流程照常載入這張卡。
  if(typeof navigate==='function')navigate().catch(()=>{});
  let tries=0;const t=setInterval(()=>{if(focus(id)||++tries>120)clearInterval(t);},250);
 }
 function renderIndex(){
  const main=document.getElementById('main'),list=items.filter(i=>i[0]===page());
  if(!main||!list.length||!main.querySelector('.heading'))return;
  let box=document.getElementById('wbArchiveIndex');
  if(!box){box=document.createElement('details');box.id='wbArchiveIndex';box.className='card';box.addEventListener('toggle',()=>{indexOpen=box.open;});}
  if(box.parentElement!==main||main.lastElementChild!==box)main.append(box);
  const html=`<summary>歷史與舊測試索引（${list.length}項 · 點選才載入）</summary><p class="note">這些展示已移出日常頁面，平常不讀資料也不啟動播放器；原始影片、遮罩、紀錄、評語與程式都保留。按「載入」只在本次瀏覽顯示，重新整理後恢復收起。</p>`+
   Object.keys(groups).filter(g=>list.some(i=>i[2]===g)).map(g=>`<h3>${esc(groups[g])}</h3><ul>`+list.filter(i=>i[2]===g).map(i=>{const on=shown.has(page()+':'+i[1]);return `<li>${esc(i[3])} <button class="secondary" data-wb-reveal="${i[1]}">${on?'跳到':'載入'}</button></li>`;}).join('')+'</ul>').join('');
  if(box.dataset.html!==html){box.innerHTML=html;box.dataset.html=html;}
  box.open=indexOpen;
 }
 document.addEventListener('click',e=>{const b=e.target.closest?.('[data-wb-reveal]');if(!b)return;const id=b.dataset.wbReveal;if(shown.has(page()+':'+id)&&focus(id))return;reveal(id);});
 function tick(){
  if(document.hidden)return;
  for(const i of items)if(i[0]===page()&&i[4]==='hide'){const n=document.getElementById(i[1]);if(n&&!shown.has(page()+':'+i[1]))n.hidden=true;}
  renderIndex();
 }
 setInterval(tick,1000);window.addEventListener('hashchange',()=>{indexOpen=false;setTimeout(tick,300);});
 return {hold,reveal,items};
})();

// 06首屏：目前車上版本、本輪待測與未解問題。內容是2026-10-31紀錄，不是即時車況。
(()=>{
 const root='/experiment/junction-live-20261031/';
 const html=`<section class="card" id="wbCurrentMethod"><h2>目前方法與本輪問題 <span class="pill">紀錄 2026-10-31 · 非即時車況</span></h2>
 <table><tr><th>項目</th><th>內容</th></tr>
 <tr><td>車上最後有紀錄的版本</td><td>路徑選項 camera-pair-v2-junction：路口切換 <b>switch handback-r5</b>、透視通道 <b>corridor r1.5</b>（b6baecb7 原始 pid.jsonl）。目前車端版本請到04唯讀查詢。</td></tr>
 <tr><td>筆電保存PID</td><td>v105：Kp240／Ki40／Kd150、積分上限1、PWM15、轉向速率600、取樣列0.55、視覺中心18px、外移0。</td></tr>
 <tr><td>已有證據</td><td>多輪兩路口通道接手並交還（c1fc3a04、7faeac29、589896bd 皆到期完成）；路口一走島左、路口二走島右已由影像與使用者確認。到期本身不證明選路正確。</td></tr>
 <tr><td>最新實跑</td><td>b6baecb7（v105）117.5秒，停在斜坡前拐角：右側無白線、基準沒涵蓋取樣列、通道只有弱支持。<a href="${root}handpush-9a2c/analysis.json">分析</a></td></tr>
 <tr><td>待測（未部署）</td><td><b>switch handback-r6</b>：通道單邊≥60列也可接手，保留r5殘段防呆；重播只改變手推拐角13格與b6baecb7停止格。預計2026-11-01部署並實跑，目前沒有實跑證據。<a href="${root}replay-r6.json">replay-r6</a> · <a href="${root}manifest.json">部署清單</a></td></tr>
 <tr><td>未解問題</td><td>坡頂（手推約61.4秒，俯仰使取樣列接近地平線；MPU6050待試）、虛線雙車道路段（約86～119秒）。兩者需要另做最小實驗，r6不處理。</td></tr>
 <tr><td>筆電已備、未部署</td><td>遠點比重／遠點列參數（預設與原行為逐格相同，需重啟車控才生效）。</td></tr></table>
 <p class="note">下方「透視通道 r1.3」與「切換 r1」是離線播放器，用的是當時的版本，不等於車上的 corridor r1.5／switch r5。舊版與K系列在本頁底部「歷史與舊測試索引」。</p></section>`;
 function mount(){
  if(location.hash!=='#models'||document.getElementById('wbCurrentMethod'))return;
  const heading=document.querySelector('#main > .heading');if(!heading)return;
  heading.insertAdjacentHTML('afterend',html);
 }
 setInterval(mount,1000);window.addEventListener('hashchange',()=>setTimeout(mount,300));
})();
