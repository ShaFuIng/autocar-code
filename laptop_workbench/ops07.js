/* 07 is an observer. No browser lease, unload stop, automatic arm or start. */
(() => {
  'use strict';
  const el = id => document.getElementById(id);
  const text = (id, value) => { if (el(id)) el(id).textContent = value; };
  const escape = value => String(value ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  let generation = 0, timer, frameTimer, catalog, selection, snapshot, dirty = false, saving = false;
  const pendingCommands = new Set();
  let imageURL = null, imageSeq = null, imageProgressAt = 0, frameAbort = null;
  const chosen = {package_id:'best-today-46db', traffic_light:false, front_car:false, location:'flag', moved:false, phase:'first', x_m:null};
  const locationPoints = {flag:[1235,106], '12_to_01':[950,106], '02_to_03_before':[1240,615], '02_to_03_after':[825,820], '04_to_05_before':[825,724], '04_to_05_after':[1190,724], '05_to_06':[1460,635], ramp:[1520,990], crest:[1255,430]};
  // Coordinates annotate the supplied map; this is a segment approximation,
  // not a calibrated metric projection. Crossings stay on their ordered branch.
  const segments = [
    ['downhill','10 → 11 下坡',0,4.45,[[1000,416],[805,520],[620,595],[520,625]]],
    ['curve4','11 → 12 第四彎',4.45,11.6,[[520,625],[390,613],[290,550],[230,440],[218,340],[253,235],[345,140],[465,105]]],
    ['straight_to_curve1','12 → 旗子 → 01',11.6,20.2,[[465,105],[1580,105]]],
    ['curve1','01 → 02 第一彎',20.2,23.54,[[1580,105],[1680,130],[1745,195],[1760,280],[1730,335],[1680,375]]],
    ['straight_to_curve2','02 → 03 斜向直線',23.54,35.3,[[1680,375],[1024,724],[495,970]]],
    ['curve2','03 → 04 第二彎',35.3,39.61,[[495,970],[420,986],[350,959],[310,904],[298,844],[312,784],[360,740],[430,720]]],
    ['straight_to_curve3','04 → 05 → 06',39.61,49,[[430,720],[1024,724],[1220,725],[1310,705],[1415,657],[1500,621]]],
    ['curve3','06 → 07 → 08 → 09',49,58.4,[[1500,621],[1585,606],[1670,642],[1750,720],[1786,800],[1775,917],[1720,991],[1630,1032],[1535,1004]]],
    ['ramp','09 後上坡',58.4,60.4,[[1535,1004],[1485,930],[1380,690],[1255,430]]],
    ['upper','坡頂 → 10 虛線轉彎',60.4,66.8,[[1255,430],[1220,370],[1180,350],[1140,352],[1100,369],[1000,416]]],
  ];
  function mapPosition(position) {
    if (!Number.isFinite(position?.s_m)) return null;
    const s = ((position.s_m % 66.8) + 66.8) % 66.8;
    const segment = segments.find(x => x[0] === position.segment) || segments.find(x => s >= x[2] && s < x[3]);
    if (!segment) return null;
    const points = segment[4], lengths = points.slice(1).map((p,i) => Math.hypot(p[0]-points[i][0],p[1]-points[i][1]));
    let remaining = Math.max(0,Math.min(1,(s-segment[2])/(segment[3]-segment[2]))) * lengths.reduce((a,b)=>a+b,0);
    for (let i=0;i<lengths.length;i++) {
      if (remaining <= lengths[i] || i===lengths.length-1) {
        const t=remaining/lengths[i]; return {point:points[i].map((v,j)=>v+(points[i+1][j]-v)*t), label:segment[1]};
      }
      remaining-=lengths[i];
    }
  }
  async function request(path, body, signal) {
    const response = await fetch('/api/ops07/'+path, {cache:'no-store', signal, ...(body === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json','X-Workbench':'1'},body:JSON.stringify(body)})});
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || '工作台暫時無法回覆。');
    return result;
  }
  function active(g) { return g===generation && location.hash==='#operations' && !!el('ops07'); }
  function dispose() {
    generation++; clearTimeout(timer); clearTimeout(frameTimer); frameAbort?.abort(); frameAbort=null;
    saving=false;
    if(imageURL) URL.revokeObjectURL(imageURL); imageURL=null; imageSeq=null; imageProgressAt=0;
    document.body.classList.remove('ops07-active');
  }
  function selectionFrom(value) {
    selection=value;
    Object.assign(chosen,{package_id:'best-today-46db',traffic_light:false,front_car:false,location:'flag',moved:false,phase:'first',x_m:null});
    for (const k of Object.keys(chosen)) if (k in value.data) chosen[k]=value.data[k];
    dirty=false;
  }
  function renderSetup() {
    el('opPackages').innerHTML=catalog.packages.map(p => `<label class="op-package ${p.id===chosen.package_id?'selected':''}"><input type="radio" name="op-package" value="${escape(p.id)}" ${p.id===chosen.package_id?'checked':''} ${p.verified?'':'disabled'}><span><strong>${escape(p.name)}</strong><small>${p.verified?({'slow-steer-7fbf':'轉向較慢 · 保守設定','best-today-46db':'歷史紀錄表現最佳','latest-2lap-97ed':'含路口、彎道與坡底修正'}[p.id]):'方案檢查未通過'}</small><span class="op-reference">${p.verified?`歷史單圈 ${p.reference.lap_s.toFixed(1)} 秒`:escape(p.error)}</span></span></label>`).join('');
    el('opTraffic').checked=chosen.traffic_light;
    el('opMoved').checked=chosen.moved;
    el('opPhase').value=chosen.phase;el('opDistance').value=chosen.x_m??'';
    el('opLocation').innerHTML=catalog.locations.map(l=>`<option value="${escape(l.id)}" ${l.id===chosen.location?'selected':''}>${escape(l.name)}</option>`).join('');
    el('opPackages').querySelectorAll('input').forEach(radio => radio.addEventListener('change',()=>{chosen.package_id=radio.value; markDirty();renderSetup();}));
    const current=catalog.packages.find(p=>p.id===chosen.package_id);
    el('opPackageInfo').innerHTML=current?.verified?`<p>這個方案的數值固定，07 不提供調整。</p><dl><dt>跑道辨識</dt><dd>V4</dd><dt>轉向中心</dt><dd>${current.parameters.center_us} 微秒</dd><dt>轉向速率上限</dt><dd>${current.parameters.steer_rate} 微秒／秒</dd><dt>程式完整性</dt><dd>${current.file_count} 個檔案核對通過</dd></dl><p>${current.id==='slow-steer-7fbf'?'已套用現役硬體相容數值，與原歷史版本有差異。':'保留當輪固定設定。'} 每個方案各有一輪完整兩圈紀錄；07 接續功能仍待現場驗證。切換未啟用方案需停止、備份並明確授權重啟車控。</p>`:'方案檢查未通過，無法讀取。';
    renderSelection(); renderMap();
  }
  function markDirty() { dirty=true; renderSelection(); renderMap(); }
  function renderSelection() {
    const loaded=selection?.data?.package_name;
    text('opLoad',saving?'正在核對方案…':dirty?'讀取所選方案':'讀取方案');
    el('opLoad').disabled=saving || !catalog.packages.find(p=>p.id===chosen.package_id)?.verified;
    text('opLoaded', dirty?'選擇已變更，請保存位置或載入方案。':loaded?`已選「${loaded}」${selection.data.applied_to_vehicle?' · 車端預覽已載入':''}`:'尚未讀取方案');
    text('opRunName', loaded?`${loaded}${selection.data.applied_to_vehicle?' · 車端已載入':' · 已選定'}`:'尚未選定本場方案');
    text('opTrafficState',chosen.traffic_light?'等待綠燈：由車端確認紅燈後的綠燈放行':'直接起跑：車端確認回正與有效路徑後放行');
  }
  function renderMap() {
    const saved=snapshot?.last_observation;
    const position=snapshot?.live?.position || saved?.position;
    const mapped=mapPosition(position);
    const current=!!(snapshot?.connected && snapshot?.live?.active && snapshot?.live?.position === position && position?.current && position.sample_age_s + (snapshot.received_age_s || 0) < 3);
    const point=locationPoints[snapshot?.session?.origin||chosen.location]||locationPoints.flag;
    el('opMapMarks').innerHTML=`<circle cx="${point[0]}" cy="${point[1]}" r="23" fill="none" stroke="#ffd484" stroke-width="7"/>${mapped?`<circle cx="${mapped.point[0]}" cy="${mapped.point[1]}" r="21" fill="${current?'#68eee1':'#b5bdc8'}" stroke="#15161d" stroke-width="7"/>`:''}`;
    text('opPosition',mapped?`${current?'目前估計':'上次觀測'}：${mapped.label}`:'尚無車端位置');
    text('opPositionNote',mapped?(current?'位置依車端里程估計，地圖僅標示大約進度。':'上次位置已保留；搬車或重啟後須重新確認，不當成目前位置。'):'金色空心點是你選的位置，不是車子的定位結果。');
    text('opLocationHint', chosen.moved?'已標記搬車：接續時當圈列為不完整圈，最初起點仍保留。':'同地點暫停會保留路段；搬車或重啟控制板後，請重新確認位置。');
  }
  function renderStatus() {
    const live=snapshot?.live, connected=!!snapshot?.connected, session=snapshot?.session;
    const supported=connected&&snapshot?.control_supported;
    const otherRun=connected&&live?.active&&live.execution_mode!=='orin_continuous';
    const matches=supported&&catalog.packages.find(p=>p.id===chosen.package_id)?.identity===session?.package_identity;
    const states={idle:'等待起跑',centering:'正在回正並確認路徑',ready:'準備完成',waiting:'等待綠燈',running:'持續行駛中',paused:'已暫停 · 保留路段',fault:'已停止 · 請排除原因'};
    text('opConnection',connected?'車端已連線':snapshot?.checking&&!snapshot?.transport_error?'正在連接車端':'車端未連線');
    el('opConnection').className='op-badge '+(connected?'good':'muted');
    text('opState',connected?(live?.active?'車端正在行駛':live?.hardware_state==='FAULT'?'車端故障':live?.hardware_state==='LOCKED'?'車子已鎖定':'等待起跑'):live?'連線中斷 · 車況待確認':'等待車端');
    text('opObserved',connected&&live?.active?'目前觀測既有車端任務；尚未由 07 啟動。':'07 尚未啟動行駛任務。');
    if(supported){text('opState',states[session.state]||session.state);text('opObserved','本場紀錄與放行由 Orin 執行，離開網頁不會停止本場。');}
    if(otherRun){text('opState','其他頁面正在行駛');text('opObserved','目前是既有車控任務，依原本模式執行；07 尚未開始本場。');}
    let issue;
    if(!connected) issue={title:'尚未確認車端狀態',remedy:'確認 Orin 電源與網路。離線時無法確認停止是否送達；需要停車請使用實體停止按鈕。',level:'warning'};
    else issue=(live.issues || []).find(x=>x.level==='danger') || (live.issues || []).find(x=>x.level==='warning') || live.issues?.[0] || {title:'持續行駛功能待車端接入',remedy:'可先讀取方案、確認畫面與接續位置；開始與接續按鈕會在車端功能完成後開放。',level:'info'};
    text('opIssueTitle',issue.title);text('opIssueRemedy',issue.remedy);el('opIssue').className='op-issue '+issue.level;
    el('opPause').disabled=pendingCommands.size>0 || !(connected&&live?.active);
    el('opStop').disabled=pendingCommands.has('stop');
    if(supported&&!(live.issues||[]).some(x=>x.level==='danger'||x.level==='warning')){
      text('opIssueTitle',session.state==='waiting'?'等待起跑訊號':session.state==='fault'?'本場已停止':'07 車端功能已接通');
      text('opIssueRemedy',session.state==='waiting'?(session.flag===null?'尚未取得起跑訊號，請確認場地訊號機與網路；車子保持等待。':session.saw_zero?'已看到紅燈，等待綠燈。':'先等待紅燈，再確認綠燈；不使用先前留下的綠燈。'):session.state==='fault'?'排除故障後確認目前位置，再按接續。控制板重啟後不會自動起跑。':'選擇起跑位置，載入方案並確認跑道畫面。暫停與接續會使用保存中心回正。');
    }
    if(otherRun){text('opIssueTitle','其他頁面正在執行車控');text('opIssueRemedy','請先結束原任務，再從 07 開始。本頁可以觀測畫面或明確要求停止，不會接管既有任務。');}
    el('opStart').disabled=pendingCommands.size>0||dirty||!matches||!!session?.id||live?.active||live?.model_state!=='ready';
    el('opResume').disabled=pendingCommands.size>0||dirty||!matches||otherRun||!session?.id||!['paused','fault'].includes(session.state);
    if(live?.reason) text('opStopReason',({'PHYSICAL_ESTOP':'使用者按下實體停止按鈕','USER_STOP':'使用者停止','PID_VISION_STALE':'控制影像過期，車端已結束本輪','PID_DURATION_COMPLETE':'既有限時任務已到期（07 不使用此模式）'}[live.reason])||'車端已結束本輪，請查看診斷。');
    else text('opStopReason','尚無本輪停止事件');
    text('opDiagnostics',JSON.stringify({received_age_s:snapshot?.received_age_s,run_id:live?.run_id,mode:live?.execution_mode,...live?.diagnostics},null,2));
    // No local lap clock: track_pos.lap is anchored at the downhill, not flag.
    const clock=session?.id?session.clock:null, laps=clock?.laps||[], seconds=v=>Number.isFinite(v)?v.toFixed(1)+' 秒':'—';
    text('opLaps',clock?laps.filter(l=>l.valid).length:'—');text('opLapTime',seconds(clock?.lap_active));text('opLastLap',seconds(laps.at(-1)?.active_s));
    text('opTimingNote',clock?`${clock.partial?'本圈不完整 · ':''}旗子估計過線 · 行駛累計 ${seconds(clock.active_s)} · 含暫停 ${seconds(clock.wall_s)}`:'等待車端本場計時');
    text('opDiagnostics',JSON.stringify({session,received_age_s:snapshot?.received_age_s,run_id:live?.run_id,mode:live?.execution_mode,...live?.diagnostics},null,2));
    if(!connected || !['ready','running','no_reliable_path'].includes(live?.model_state)) {text('opFrameLabel',imageURL?'上次畫面 · 目前未更新':'尚無即時畫面');el('opCamera').classList.add('stale');}
    if(snapshot?.last_command?.acknowledged===false&&!pendingCommands.size) text('opCommandStatus','上次停止請求未收到完整回覆，不能確認車子已停止。請確認現場並使用實體停止按鈕。');
    renderMap();
  }
  async function poll(g) {
    try {
      const next=await request('status');if(!active(g))return;
      snapshot=next;
      if(!dirty&&!saving&&selection?.rev!==next.selection.rev) {selectionFrom(next.selection);renderSetup();}
      renderStatus();
    } catch(e) {
      if(!active(g))return;
      snapshot={...snapshot,connected:false,checking:false};renderStatus();
    } finally { if(active(g)) timer=setTimeout(()=>poll(g),2500); }
  }
  async function frame(g) {
    if(!active(g))return;
    try {
      if(document.hidden||!snapshot?.connected||!['ready','running','no_reliable_path'].includes(snapshot.live?.model_state))return;
      frameAbort=new AbortController();
      const timeout=setTimeout(()=>frameAbort?.abort(),16000);
      let response;
      try {response=await fetch('/api/ops07/frame',{cache:'no-store',signal:frameAbort.signal});} finally {clearTimeout(timeout);}
      if(!response.ok)throw new Error('frame unavailable');
      const blob=await response.blob();if(!active(g))return;
      const next=URL.createObjectURL(blob),old=imageURL;imageURL=next;el('opFrame').src=next;el('opFrame').hidden=false;if(old)URL.revokeObjectURL(old);
      el('opCameraEmpty').hidden=true;
      const seq=snapshot.live.preview_seq;
      if(seq!==imageSeq){imageSeq=seq;imageProgressAt=performance.now();}
      const stale=!imageProgressAt||performance.now()-imageProgressAt>5000;
      el('opCamera').classList.toggle('stale',stale);
      text('opFrameLabel',stale?'上次畫面 · 更新尚未確認':'車端辨識與路徑');
    } catch(e) {if(active(g)){text('opFrameLabel','畫面暫停更新');el('opCamera').classList.add('stale');}}
    finally {if(active(g))frameTimer=setTimeout(()=>frame(g),1200);}
  }
  async function load(saveOnly=false) {
    if(saving)return;const g=generation;const submitted={...chosen};saving=true;renderSelection();
    try {const result=await request(saveOnly===true?'save-selection':'load',{rev:selection.rev,...submitted});if(!active(g))return;selection=result;
      if(JSON.stringify(chosen)===JSON.stringify(submitted)){selectionFrom(result);text('opFeedback',result.data.applied_to_vehicle?'方案已載入車端，等待辨識畫面；尚未起跑。':'選擇已保存；載入方案後可開始。');}
      else {dirty=true;text('opFeedback','前一個選擇已保存；目前有新選擇尚未讀取。');}
      renderSetup();}
    catch(e) {if(active(g))text('opFeedback',e.message);}
    finally {saving=false;if(active(g))renderSelection();}
  }
  async function command(action) {
    if(pendingCommands.has(action)||(action==='pause'&&pendingCommands.size))return;
    const motion=['start','resume'].includes(action);
    if(motion&&!el('opAreaClear').checked){text('opCommandStatus','請先確認現場可以起跑，且可立即使用實體停止按鈕。');return;}
    const g=generation;pendingCommands.add(action);
    renderStatus();text('opCommandStatus','正在等待車端回覆…');
    try {const result=await request(action,{intent:action,...(motion?{area_clear:true,rev:selection.rev}:{})});if(active(g)){text('opCommandStatus',result.message);if(result.session)snapshot.session=result.session;}}
    catch(e) {if(active(g))text('opCommandStatus',e.message+'；不會自動重送。若需要停車請使用實體停止按鈕。');}
    finally {pendingCommands.delete(action);if(active(g))renderStatus();}
  }
  async function mount() {
    dispose();const g=generation;document.body.classList.add('ops07-active');snapshot=null;dirty=false;
    el('main').innerHTML='<div class="op-loading" role="status">正在讀取正式操作頁…</div>';
    try {catalog=await request('catalog');if(!activeRoute(g))return;selectionFrom(catalog.selection);}catch(e){if(activeRoute(g))el('main').innerHTML=`<div class="card">正式操作頁讀取失敗：${escape(e.message)}<p><button onclick="window.Ops07.mount()">重新讀取</button></p></div>`;return;}
    el('main').innerHTML=`<div id="ops07">
      <div class="op-heading"><div><span class="op-eyebrow">07 ／ 正式操作</span><h1>準備好，接著跑。</h1><p>固定方案 · 車端自主執行 · 網頁接收畫面</p></div><span id="opConnection" class="op-badge muted">正在連接車端</span></div>
      <div id="opIssue" class="op-issue info" role="status"><span class="op-issue-icon">i</span><div><strong id="opIssueTitle">正在確認車況</strong><p id="opIssueRemedy">讀取方案不會啟動車子。</p></div></div>
      <div class="op-layout"><section class="op-panel op-setup" aria-labelledby="opSetupTitle"><div class="op-panel-title"><h2 id="opSetupTitle">本場設定</h2><span class="op-badge">固定數值</span></div>
        <div class="op-recognition"><span>跑道辨識</span><strong>V4 <small>白線辨識</small></strong></div>
        <h3>選擇行駛方案</h3><div id="opPackages" class="op-packages"></div>
        <h3>起跑與停等</h3><label class="op-option"><input id="opTraffic" type="checkbox"><span>等待綠燈起跑<small>按開始後，由 Orin 等待紅燈轉綠燈</small></span></label>
        <label class="op-option unavailable"><input id="opFront" type="checkbox" disabled><span>遇到前車先停等 <span class="op-soon">待接入</span><small>目標：前車離開約 3 公尺後自動繼續</small></span></label>
        <button id="opLoad" class="op-primary">讀取方案</button><p id="opLoaded" class="op-note" aria-live="polite"></p><p id="opFeedback" class="op-feedback" role="status"></p>
        <details class="op-details"><summary>方案內容與限制</summary><div id="opPackageInfo"></div></details>
      </section>
      <div class="op-monitor"><section class="op-panel op-view"><div class="op-panel-title"><h2>跑道即時畫面</h2><span id="opFrameLabel" class="op-small">等待影像</span></div>
        <div id="opCamera" class="op-camera"><img id="opFrame" alt="車端提供的跑道辨識與路徑疊圖" hidden><div id="opCameraEmpty" class="op-camera-empty"><span class="op-camera-icon">◉</span><strong>等待車端畫面</strong><span>車端已有辨識預覽時，這裡會自動接收。</span></div><span class="op-camera-caption">車端原疊圖 · 不另疊不同影格的路徑</span></div>
        <div class="op-metrics"><div><span>完成圈數</span><strong id="opLaps">—</strong></div><div><span>本圈計時</span><strong id="opLapTime">—</strong></div><div><span>上一圈</span><strong id="opLastLap">—</strong></div><div class="op-continuous"><span>行駛模式</span><strong>持續行駛</strong><small>無倒數限制 · 直到暫停或故障</small></div></div><p id="opTimingNote" class="op-note op-inset"></p>
      </section>
      <section class="op-panel op-map-panel"><div class="op-panel-title"><div><h2>跑道位置</h2><p id="opPosition" class="op-small">尚無車端位置</p></div><button id="opLocationToggle" class="op-secondary" aria-expanded="false">確認接續位置</button></div>
        <div class="op-map"><img src="/ops07-track.webp" alt="正式跑道地圖，旗子起點依序經 01 至 12，含平面交叉與高架路段"><svg viewBox="0 0 1980 1213" aria-hidden="true"><g id="opMapMarks"></g></svg></div>
        <div class="op-map-footer"><span><i class="op-dot"></i> 車端位置估計</span><span><i class="op-dot hollow"></i> 選定接續位置</span></div><p id="opPositionNote" class="op-note op-inset"></p>
        <div id="opLocationEditor" class="op-location-editor" hidden><label for="opLocation">起跑／搬車後，車子放在哪裡？</label><select id="opLocation"></select><label>所在圈段<select id="opPhase"><option value="first">本次首次上坡前</option><option value="after_down">已完成下坡後</option></select></label><label id="opDistanceLabel">路口前：距前一彎結束幾公尺？<input id="opDistance" type="number" min="0" step="0.01" placeholder="僅路口前入口需要實量"></label><label class="op-option"><input type="checkbox" id="opMoved"><span>暫停後曾搬動車子／重啟控制板</span></label><p id="opLocationHint" class="op-note"></p><p class="op-note">彎道中間或下坡中間，先移到該段入口再確認；暫停及繼續都使用保存中心回正；I²C／急停故障時不強行回正。回正顯示是控制板命令確認，實際舵角仍需現場確認。</p><button id="opSaveLocation" class="op-secondary">保存選擇</button></div>
      </section></div></div>
      <section class="op-actions" aria-label="行駛控制"><div class="op-session"><span id="opState">等待車端</span><strong id="opRunName">尚未選定本場方案</strong><small id="opObserved"></small></div><label class="op-option"><input id="opAreaClear" type="checkbox"><span>現場可起跑，且可立即使用實體停止按鈕</span></label><div class="op-buttons"><button id="opStart" class="op-primary" disabled >開始行駛</button><button id="opResume" class="op-secondary" disabled >接續行駛</button><button id="opPause" class="op-secondary" disabled>暫停</button><button id="opStop" class="op-stop">■ 立即停止輸出</button></div><p class="op-note op-action-note">停止輸出後仍可能滑行；網路中斷時請使用實體停止按鈕。</p><p id="opCommandStatus" class="op-command-status" role="alert"></p></section>
      <div class="op-bottom"><details class="op-details"><summary>狀態、紀錄與功能進度</summary><p id="opTrafficState"></p><p id="opStopReason"></p><p>本場、起點、位置與計時由車端保存。旗子過線以里程與路段估計；搬車當圈不計完整圈。前車距離來源尚未提供，暫不開放。</p><p>07 暫停保留路段並回正，接續由車端確認新鮮路徑。故障或控制板重啟後需確認位置。關閉本頁不會送停止、心跳或重啟指令。</p><a href="#replay">查看行車紀錄 →</a><details><summary>技術診斷</summary><pre id="opDiagnostics"></pre></details></details><button id="opReset" class="op-text-button">重置本場與選擇</button></div>
      <dialog id="opResetDialog"><h2>重置本場與選擇？</h2><p>先停止車控，再清除本場與起點；歷史紀錄會保留。這個按鈕不代替停止。</p><div class="op-buttons"><button id="opResetCancel" class="op-secondary">保留選擇</button><button id="opResetConfirm" class="op-primary">重置選擇</button></div></dialog>
    </div>`;
    renderSetup();renderStatus();
    const lab=document.createElement("button");lab.textContent="進入離線流程測試";lab.className="op-secondary";lab.onclick=()=>window.Ops07Lab.mount();el("ops07").querySelector(".op-heading").append(lab);
    el('opLoad').addEventListener('click',()=>load(false));el('opSaveLocation').addEventListener('click',()=>load(true));
    el('opPhase').addEventListener('change',e=>{chosen.phase=e.target.value;markDirty();});
    el('opDistance').addEventListener('change',e=>{chosen.x_m=e.target.value===''?null:Number(e.target.value);markDirty();});
    el('opStart').addEventListener('click',()=>command('start'));el('opResume').addEventListener('click',()=>command('resume'));
    el('opTraffic').addEventListener('change',e=>{chosen.traffic_light=e.target.checked;markDirty();});
    el('opMoved').addEventListener('change',e=>{chosen.moved=e.target.checked;markDirty();});
    el('opLocation').addEventListener('change',e=>{chosen.location=e.target.value;markDirty();});
    el('opLocationToggle').addEventListener('click',()=>{const editor=el('opLocationEditor');editor.hidden=!editor.hidden;el('opLocationToggle').setAttribute('aria-expanded',String(!editor.hidden));});
    el('opPause').addEventListener('click',()=>command('pause'));el('opStop').addEventListener('click',()=>command('stop'));
    el('opReset').addEventListener('click',()=>el('opResetDialog').showModal());el('opResetCancel').addEventListener('click',()=>el('opResetDialog').close());
    el('opResetConfirm').addEventListener('click',async()=>{el('opResetConfirm').disabled=true;try{const result=await request('reset',{rev:selection.rev});if(!active(g))return;selectionFrom(result);renderSetup();text('opFeedback','已重置本場與選擇，歷史紀錄仍保留。');el('opResetDialog').close();}catch(e){if(active(g))text('opFeedback',e.message);}finally{if(active(g))el('opResetConfirm').disabled=false;}});
    poll(g);frame(g);
  }
  function activeRoute(g) { return generation===g&&location.hash==='#operations'; }
  window.Ops07={mount,dispose,mapPosition};
})();
