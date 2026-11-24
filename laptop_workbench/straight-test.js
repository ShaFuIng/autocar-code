/* 04 straight-line test card (laptop analysis only; never sends vehicle commands).
   Data: /api/straight/* (straight_test.py) on runs already synced with the existing
   one-way Orin sync. Presets save a new visual-pid revision after confirmation. */
(()=>{
 const q=id=>document.getElementById(id),h=v=>String(v??'—').replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
 const post=async(path,body)=>{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-Workbench':'1'},body:JSON.stringify(body)});const d=await r.json();if(!r.ok)throw Error(d.error||r.status);return d;};
 const get=async path=>{const r=await fetch(path);const d=await r.json();if(!r.ok)throw Error(d.error||r.status);return d;};
 let data=null,selected=null,onlyJunction=true;
 const STOP={PID_DURATION_COMPLETE:'跑滿',PID_INVALID_OR_STALE_PATH:'路徑無效／過期',PID_VISION_STALE:'影像過期',USER_STOP:'手動停止',PHYSICAL_ESTOP:'實體急停',MANUAL_SESSION_ENDED:'車控工作階段結束'};
 const fmt=(v,n=2)=>v==null?'—':Number(v).toFixed(n);
 function time(ts){if(!ts)return '—';const d=new Date(ts*1000);return `${String(d.getMonth()+1).padStart(2,'0')}/${String(d.getDate()).padStart(2,'0')} ${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}`;}
 function table(){
  const runs=(data?.runs||[]).filter(r=>!onlyJunction||r.path_algorithm==='camera-pair-v2-junction');
  q('stRuns').innerHTML=`<table><thead><tr><th>時間</th><th>參數版</th><th>Kp/Ki/Kd</th><th>取樣列</th><th>舵機速率</th><th>PWM</th><th>跑／設定</th><th>停止</th><th>直線秒數</th><th>直線平均誤差</th><th>直線擺幅(std)</th><th>直線峰對峰</th><th>過中心間隔</th><th>直線平均舵機</th><th>整輪擺幅(std)</th></tr></thead><tbody>${runs.map(r=>{const p=r.params||{},s=r.straight,w=r.whole;return `<tr data-id="${h(r.id)}" style="cursor:pointer${selected===r.id?';background:rgba(120,200,160,.18)':''}"><td>${time(r.started)}</td><td>v${h(r.parameter_revision)}</td><td>${h(p.kp)}/${h(p.ki)}/${h(p.kd)}</td><td>${h(p.row_ratio)}</td><td>${h(p.steer_rate)}</td><td>${h(p.pwm)}</td><td>${fmt(r.ran_s,1)}／${h(r.seconds_set)}秒</td><td>${h(STOP[r.stop]||r.stop)}</td><td>${s?fmt(s.seconds,1):'無'}</td><td>${s?fmt(s.e_mean,3):'—'}</td><td><b>${s?fmt(s.e_std,3):'—'}</b></td><td>${s?fmt(s.e_pp,2):'—'}</td><td>${s&&s.cross_interval_s!=null?fmt(s.cross_interval_s,1)+'秒':'—'}</td><td>${s?h(s.steer_mean):'—'}</td><td>${w?fmt(w.e_std,3):'—'}</td></tr>`;}).join('')}</tbody></table>`;
  q('stRuns').querySelectorAll('tr[data-id]').forEach(tr=>tr.onclick=()=>open(tr.dataset.id));
 }
 function plot(series,summary){
  const c=q('stPlot'),g=c.getContext('2d'),W=c.width,H=c.height,pad=40;g.clearRect(0,0,W,H);if(!series.length)return;
  const T=series[series.length-1].t||1,X=t=>pad+(W-2*pad)*t/T,half=(H-3*pad)/2,eY=v=>pad+half/2-v*half/2,uY=v=>2*pad+half+half/2-(v-1580)/600*half;
  g.fillStyle='rgba(80,180,140,.15)';for(const [a,b] of summary.segment_spans||[])g.fillRect(X(a),pad-10,X(b)-X(a),H-2*pad+10);
  g.strokeStyle='#888';g.lineWidth=1;[[eY(0),'誤差 0'],[uY(1580),'舵機 1580']].forEach(([y,l])=>{g.beginPath();g.moveTo(pad,y);g.lineTo(W-pad,y);g.stroke();g.fillStyle='#888';g.fillText(l,4,y-3);});
  g.fillStyle='#666';g.fillText('誤差（右＝目標在畫面右側，車偏左）',pad,pad-14);g.fillText('舵機命令 µs（上＝右轉）',pad,2*pad+half-14);
  const line=(f,col)=>{g.strokeStyle=col;g.lineWidth=1.6;g.beginPath();series.forEach((s,i)=>{const x=X(s.t),y=f(s);i?g.lineTo(x,y):g.moveTo(x,y);});g.stroke();};
  line(s=>eY(Math.max(-1,Math.min(1,s.e))),'#d9534f');line(s=>uY(s.u),'#2b7bd6');
  g.fillStyle='#666';for(let t=0;t<=T;t+=10){g.fillText(t+'s',X(t)-6,H-8);}
 }
 async function open(id){selected=id;table();try{const d=await get('/api/straight/run/'+id);const s=d.summary;q('stDetail').innerHTML=`<b>${h(id.slice(0,8))}</b> · v${h(s.parameter_revision)} · ${h(s.path_algorithm)} · 跑 ${fmt(s.ran_s,1)} 秒（${h(STOP[s.stop]||s.stop)}）· 綠底＝自動判定的直線段 · 直線段：${(s.straight_segments||[]).map((m,i)=>`#${i+1} ${fmt(m.seconds,1)}秒 平均${fmt(m.e_mean,2)} 擺幅${fmt(m.e_std,2)}`).join('；')||'無'}`;plot(d.series,s);}catch(e){q('stDetail').textContent='讀取失敗：'+e.message;}}
 async function presets(){try{const [doc,hist]=await Promise.all([get('/api/document/visual-pid'),get('/api/straight/presets/history')]);const p=doc.data.parameters;
  q('stPresets').innerHTML=`<p>目前保存：<b>v${doc.rev}</b> · Kp ${p.kp} · Ki ${p.ki}（積分上限 ${p.integral_limit}）· Kd ${p.kd} · 取樣列 ${p.row_ratio} · 舵機速率 ${p.steer_rate} · PWM ${p.pwm}</p><table><thead><tr><th>預設組</th><th>會改的值（目前 → 新）</th><th></th></tr></thead><tbody>${Object.entries(data.presets).map(([k,v])=>{const ch=Object.entries(v.params).filter(([a,b])=>p[a]!==b);return `<tr><td>${h(v.label)}</td><td>${ch.length?ch.map(([a,b])=>`${a} ${p[a]} → ${b}`).join('、'):'與目前相同'}</td><td>${ch.length?`<button class="secondary" data-preset="${k}" data-rev="${doc.rev}">存成新版本</button>`:''}</td></tr>`;}).join('')}</tbody></table><details><summary>套用紀錄（${hist.length}）</summary><ul>${hist.slice(-10).reverse().map(x=>`<li>${time(x.at)} · ${h(x.preset)} · v${x.from_rev} → v${x.to_rev} · ${h(JSON.stringify(x.changed))}</li>`).join('')}</ul></details>`;
  q('stPresets').querySelectorAll('button[data-preset]').forEach(b=>b.onclick=async()=>{const v=data.presets[b.dataset.preset];if(!confirm(`把「${v.label}」存成新的PID版本？\n其他參數保持目前保存值。存完後在04按「恢復連線」；取樣列有改時要重新載入預覽。`))return;b.disabled=true;try{const r=await post('/api/straight/preset',{preset:b.dataset.preset,expected_rev:Number(b.dataset.rev)});q('stMsg').textContent=`已存成 v${r.rev}：${JSON.stringify(r.changed)}${r.reload_preview?'；取樣列已改，請重新載入預覽':''}。開始前請按「恢復連線」。`;await presets();}catch(e){q('stMsg').textContent='未存：'+e.message;b.disabled=false;}});
 }catch(e){q('stPresets').textContent='讀取PID參數失敗：'+e.message;}}
 async function load(){q('stMsg').textContent='分析已同步紀錄中…';try{data=await get('/api/straight/runs');q('stMsg').textContent=`已同步PID紀錄 ${data.total} 筆，列出最近 ${data.runs.length} 筆。${data.note}`;table();await presets();if(!selected&&data.runs[0])open(data.runs.find(r=>!onlyJunction||r.path_algorithm==='camera-pair-v2-junction')?.id||data.runs[0].id);}catch(e){q('stMsg').textContent='讀取失敗：'+e.message;}}
 async function sync(){const b=q('stSync');b.disabled=true;q('stMsg').textContent='向Orin唯讀同步尚未下載的紀錄…（只讀取紀錄，不會控制車輛）';try{await post('/api/orin/sync',{missing:true});for(let i=0;i<120;i++){await new Promise(r=>setTimeout(r,1500));const s=(await get('/api/orin/records')).sync||{};q('stMsg').textContent=`同步中 ${s.completed||0}/${s.total||'?'} ${s.current||''}`;if(s.state&&s.state!=='running'){q('stMsg').textContent=`同步結束：${s.state}${s.errors?.length?'，錯誤 '+s.errors.length:''}`;break;}}await load();}catch(e){q('stMsg').textContent=/No matching/.test(e.message)?'沒有新紀錄需要同步。':'同步失敗：'+e.message;await load();}finally{b.disabled=false;}}
 function mount(){
  if(location.hash!=='#drive'||q('straightTest'))return;const main=q('main');if(!main||!main.querySelector('.card'))return;
  main.insertAdjacentHTML('beforeend',`<section class="card" id="straightTest"><h2>直線測試（筆電分析；不會發出車控指令）</h2>
  <p class="note">步驟：① 在最長直線擺正車子、車道置中；② 在上方限時PID用 orin_timed 6–8 秒開始；③ 跑完按「同步最新紀錄」；④ 在表中比較直線段指標。一次只改一項參數。<b>直線擺幅(std)</b>越小越穩；<b>直線平均誤差</b>越接近0越置中；過中心間隔反映擺盪週期。直線段由路徑形狀自動判定，非安全門檻。</p>
  <div class="actions"><button id="stSync">同步最新紀錄</button><button id="stReload" class="secondary">重新整理分析</button><label><input type="checkbox" id="stOnly" checked> 只看路口通道切換選項</label></div>
  <p id="stMsg" class="note"></p><div id="stRuns" style="overflow:auto;max-height:340px"></div>
  <p id="stDetail" class="note"></p><canvas id="stPlot" width="1000" height="360" style="width:100%;max-width:1000px;background:var(--panel,#fff);border-radius:12px"></canvas>
  <h3>預設參數組（存成新的PID版本，可隨時用另一組或04退回）</h3><p id="stPresetMsg" class="note">每組只改列出的值，其餘沿用目前保存值；存檔使用與04相同的驗證與版本檢查。</p><div id="stPresets"></div></section>`);
  q('stSync').onclick=sync;q('stReload').onclick=load;q('stOnly').onchange=e=>{onlyJunction=e.target.checked;table();};q('stMsg').textContent='尚未載入；按「重新整理分析」才從磁碟讀取已同步紀錄（不影響實測）。';
 }
 window.addEventListener('hashchange',()=>setTimeout(mount,800));setInterval(mount,1500);mount();
})();
