/* 04 live-PID helper: shows WHY "開始本輪實車PID" is disabled and adds a short
   how-to. Reads the same status/document APIs as pid-live.js; it never starts,
   stops or changes the vehicle and never relaxes pid-live.js's own checks
   (the start button state is still decided only by pid-live.js). */
(()=>{
 const q=id=>document.getElementById(id),h=v=>String(v??'').replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
 const NAME={'camera-pair-v2-junction-flag':'正式起點（平地）起跑（同整趟控制）','camera-pair-v2-junction-crest-full':'坡頂起跑（同整趟控制）','camera-pair-v2-junction-uphill':'上坡起跑測試（同起點版控制）','camera-pair-v2-junction-crest':'坡頂起跑測試（左車道）','camera-pair-v2-junction':'新視角雙邊界 r2＋路口通道切換','camera-pair-v2':'新視角雙邊界 r2','legacy9':'原9列雙線中心'};
 let busy=false;
 async function json(path){const r=await fetch(path);if(!r.ok)throw Error(r.status);return r.json();}
 function guide(){
  if(q('pidHowTo'))return;const card=q('livePIDStart')?.closest('.card');if(!card)return;
  card.insertAdjacentHTML('beforebegin',`<section class="card tint" id="pidHowTo"><h2>實車 PID 怎麼跑（讓車子動的只有下方「開始本輪實車PID（會動）」）</h2>
   <ol class="steps"><li>把車擺在要測的直線起點，車道置中、車頭對正。</li>
   <li>路徑選「新視角雙邊界 r2＋路口通道切換」；取樣列有改過就先按「停止辨識與試跑」，再按「載入模型／預覽（不動車）」。</li>
   <li>只改了 Kp／Ki／Kd 等其他參數時，按「嘗試恢復連線（不動車）」讓頁面讀到新版本。</li>
   <li>看下方「開始前檢查」全部打勾，勾選現場確認框，再按「開始本輪實車PID（會動）」；緊急時按「立即停止PID」或實體急停。</li>
   <li>跑完到本頁最下面「直線測試」按「同步最新紀錄」比較結果。</li></ol>
   <p class="note">本說明與檢查清單只讀取狀態，不會讓車子動、不會改參數。「建立一輪（參數快照與時間線）」只建立筆電紀錄，也不會讓車子動。</p></section>`);
 }
 function render(rows,fix){
  const btn=q('livePIDStart');if(!btn)return;let box=q('pidStartCheck');
  if(!box){(btn.closest('.actions')||btn).insertAdjacentHTML('afterend','<div id="pidStartCheck" class="ribbon" style="margin-top:8px"></div>');box=q('pidStartCheck');}
  const bad=rows.filter(r=>!r.ok);
  box.innerHTML=`<b>開始前檢查</b>（${bad.length?`還有 ${bad.length} 項未通過，所以開始按鈕是灰的`:'全部通過；若按鈕仍是灰的，請按「嘗試恢復連線（不動車）」'}）<ul style="margin:6px 0 0 0">${rows.map(r=>`<li>${r.ok?'✅':'❌'} ${h(r.label)}${r.ok?'':` — <span>${r.fix}</span>`}</li>`).join('')}</ul>`;
  box.querySelectorAll('[data-use-path]').forEach(b=>b.onclick=()=>{const sel=q('livePIDPath');sel.value=b.dataset.usePath;sel.dispatchEvent(new Event('change'));fix&&fix();});
  box.querySelectorAll('[data-click]').forEach(b=>b.onclick=()=>q(b.dataset.click)?.click());
 }
 async function check(){
  if(busy||location.hash!=='#drive'||!q('livePIDStart'))return;busy=true;
  try{
   guide();
   const [s,d]=await Promise.all([json('/api/pid/live/status'),json('/api/document/visual-pid')]);
   const p=d.data?.parameters||{},path=q('livePIDPath')?.value||'legacy9',cap=(s.path_algorithms||[]).find(a=>a.id===path);
   const reusable=window.PIDSession?.reusable?.(s,d,q('livePIDModel')?.value||'v4-default',path);
   const btn=(id,label)=>`<button class="secondary" data-click="${id}" style="padding:2px 10px">${label}</button>`;
   const rows=[
    {ok:!!s.state&&s.state!=='idle'&&s.state!=='failed',label:'Orin 預覽已載入（目前：'+(s.state||'未知')+'）',fix:'按 '+btn('livePIDPrepare','載入模型／預覽（不動車）')},
    {ok:(s.path_algorithm||'legacy9')===path,label:`路徑選單與 Orin 預覽相同（選單：${NAME[path]||path}；Orin：${NAME[s.path_algorithm]||s.path_algorithm||'無'}）`,fix:s.path_algorithm?`<button class="secondary" data-use-path="${h(s.path_algorithm)}" style="padding:2px 10px">選單改成 ${h(NAME[s.path_algorithm]||s.path_algorithm)}</button>`:'先載入預覽'},
    {ok:!!cap?.control,label:'此路徑算法允許實車PID',fix:'改選可實車的算法（例如路口通道切換）'},
    {ok:s.row_ratio===p.row_ratio,label:`預覽取樣列與保存參數相同（預覽 ${s.row_ratio??'—'}／保存 ${p.row_ratio}）`,fix:'取樣列改過：按 '+btn('livePIDPreviewStop','停止辨識與試跑')+' 再按 '+btn('livePIDPrepare','載入模型／預覽（不動車）')},
    {ok:s.state==='ready'&&(s.ready_frames||0)>=3&&!!s.latest,label:`連續有效路徑至少3格（目前 ${s.ready_frames??0}）`,fix:'把車擺到兩條車道線都清楚的位置，等畫面出現路徑'},
    {ok:!window.PIDField?.isDirty?.(),label:'參數草稿沒有未保存的修改',fix:'到上方「下一輪參數」按保存，或按恢復保存值'},
    {ok:!!reusable,label:`頁面已核對保存參數 v${d.rev}`,fix:'按 '+btn('livePIDRecover','嘗試恢復連線（不動車）')},
    {ok:s.autonomous_pid_version===1,label:'Orin 支援限時自主PID',fix:'車端版本不支援，需另行部署'},
    {ok:s.run?.state!=='running',label:'目前沒有其他PID正在執行',fix:'等待目前這輪結束或按立即停止PID'},
   ];
   render(rows);
  }catch(e){const box=q('pidStartCheck');if(box)box.textContent='開始前檢查：暫時無法讀取Orin狀態（'+e.message+'），不代表車況。';}
  finally{busy=false;}
 }
 window.addEventListener('hashchange',()=>setTimeout(check,800));setInterval(check,2000);check();
})();
