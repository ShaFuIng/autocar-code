// Preserve original control logic; paginate only the rendered history rows.
(()=>{let size=10,page=0;const table=document.getElementById('motorHistory');if(!table)return;const bar=document.createElement('div');table.parentElement.before(bar);table.parentElement.style.maxHeight='400px';const draw=()=>{const rows=[...table.tBodies[0].rows],pages=Math.max(1,Math.ceil(rows.length/size));page=Math.min(page,pages-1);rows.forEach((r,i)=>r.hidden=i<page*size||i>=(page+1)*size);bar.innerHTML=`每頁 <select>${[10,20,50].map(n=>`<option ${size===n?'selected':''}>${n}</option>`).join('')}</select> 筆 <button data-prev>上一頁</button> ${page+1}/${pages} · ${rows.length}筆 <button data-next>下一頁</button>`;bar.querySelector('select').onchange=e=>{size=Number(e.target.value);page=0;draw()};bar.querySelector('[data-prev]').onclick=()=>{page=Math.max(0,page-1);draw()};bar.querySelector('[data-next]').onclick=()=>{page=Math.min(pages-1,page+1);draw()}};new MutationObserver(draw).observe(table.tBodies[0],{childList:true});draw()})();

// Laptop-only form restoration; no synthetic events, requests, or control commands.
(()=>{
 if(!window.LastInputs)return;
 if(document.getElementById('motorRight')){
  LastInputs.bind('vehicle-motor',['motorSurface','motorWheel','motorDirection','motorRight','motorLeft','motorSeconds','motorWarmup','motorLabel','motorEqual','firstPulse','stepSize']);
  motorInputs();
  document.getElementById('minus').textContent='−'+stepSize()+' µs';
  document.getElementById('plus').textContent='＋'+stepSize()+' µs';
 }
 if(document.getElementById('settingsDraft')){
  const ids=['center','right','left','steerRate','accelTime','decelTime'];
  if(LastInputs.bind('vehicle-manual-draft',ids)){
   dirty=true;editVersion++;
   document.getElementById('settingsDraft').textContent='已恢復上次輸入草稿；尚未套用到車上。';
  }
  // Micro-adjust buttons and restore update the form programmatically. Capture their final result.
  const originalFill=fill;
  fill=function(c){originalFill(c);LastInputs.capture('vehicle-manual-draft',ids);};
  LastInputs.bind('vehicle-manual-options',['recordOnStart','settingsCount']);
 }
})();
