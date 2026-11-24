// Local offline review only. Human choices remain separate from algorithm output.
async function mountThreeZoneReview(){
 if(route!=='models'||document.getElementById('threeZoneReview'))return;
 const host=document.getElementById('researchModels');if(!host)return;
 const box=document.createElement('section');box.className='card';box.id='threeZoneReview';document.querySelector('main').prepend(box);
 try{
  const base='/experiment/three-zone-20261030/';const report=await api(base+'report.json');
  const key='three-zone-human-20261030';let doc=await api('/api/document/'+key);let current=0;
  box.innerHTML=`<h2>近中遠連續選線 r1 · 離線判讀</h2><p>六輪既有遮罩重播；仍有入口折線跟錯的疑慮，尚未通過。橘線不是標準答案，未部署、未修改 PID。</p>
  ${report.human_analysis?`<div class="ribbon"><b>已讀取 ${report.human_analysis.answers} 題人工判讀（v${report.human_analysis.document_revision}）</b><p>${esc(report.human_analysis.conclusion)}</p><p>下一步：${esc(report.human_analysis.next_step)}</p></div>`:""}
  <p>每題先看左側原圖，再判斷右側路徑。綠＝該輪原路徑；橘＝候選；紫＝實際遮罩；白色數字＝觀測線段。藍色橫線分近／中／遠。前後半秒原圖在下方。</p>
  <label>判讀畫面 <select id="tzQuestion">${report.questions.map((q,i)=>`<option value="${i}">${q.id} · ${q.run} · ${q.t.toFixed(2)} 秒</option>`).join('')}</select></label>
  <p id="tzDetail"></p><img id="tzImage" style="width:100%;height:auto" alt="原圖與現行候選路徑比較"><details><summary>看前後半秒原圖</summary><img id="tzContext" style="width:100%;height:auto" alt="前後半秒原圖"></details>
  <div class="formgrid"><label>這是什麼路段？<select id="tzScene"><option value="">尚未判讀</option><option value="junction_straight">路口，應直行</option><option value="normal_bend">正常彎道，應沿彎走</option><option value="straight">一般直線</option><option value="uncertain">看不出來</option></select></label>
  <label>哪條路徑比較符合應走方向？<select id="tzVerdict"><option value="">尚未判讀</option><option value="candidate">橘線較對</option><option value="recorded">綠線較對</option><option value="both">兩條都可以</option><option value="neither">兩條都不對</option><option value="uncertain">無法判斷</option></select></label></div>
  <label>補充方向／線段編號（可空白）<input id="tzNote" maxlength="500" placeholder="例如：應朝左上隧道口；應接線段 1→3"></label><button id="tzSave">保存本題判讀</button><span id="tzSaved"></span>
  <details><summary>六輪指標與編碼器對照</summary><p>固定 0.6 列沒目標不等於 PID 沒有可用取樣列。不同輪使用的歷史算法不同；表格是同輪記錄與候選比較，不當作算法因果排名。輪速用有效正反向淨計數／秒，未換算 m/s；時間為 Orin 收到回報時刻，不能量測舵機延遲。</p>
  <table><thead><tr><th>輪次</th><th>影格</th><th>0.6 列無目標 原→候選</th><th>跳動&gt;48px 原→候選</th><th>相同控制器無效 原→候選</th><th>輪速 A／B 中位數</th></tr></thead><tbody>${Object.entries(report.runs).map(([id,r])=>`<tr><td>${id}</td><td>${r.frames}</td><td>${r.recorded.no_target}→${r.candidate.no_target}</td><td>${r.recorded.jumps_over_48px}→${r.candidate.jumps_over_48px}</td><td>${r.recorded_path_v3_controller_invalid}→${r.candidate_pid_invalid}</td><td>${r.encoder_median_counts_s.map(Math.round).join('／')}</td></tr>`).join('')}</tbody></table>
  <label>訊號圖 <select id="tzRun">${Object.keys(report.runs).map(id=>`<option>${id}</option>`).join('')}</select></label><img id="tzSignals" style="width:100%;height:auto" alt="目標輪速與舵機指令時間圖"></details>
  <p><a href="${base}report.json">完整資料與限制</a> · <a href="${base}acceptance.json">連接規則驗證</a>。人工判讀只保存筆電資料庫，保留版本歷史，不自動作訓練標籤。</p>`;
  const el=id=>document.getElementById(id);
  const show=()=>{const q=report.questions[current];const saved=doc.data?.answers?.[q.id]||{};el('tzImage').src=base+q.image;el('tzContext').src=base+q.context;el('tzDetail').textContent=`${q.id} · seq ${q.seq} · 原圖相對遮罩 ${q.video_delta_ms.toFixed(0)} ms · ${q.status==='OBSERVED_ZONE_CHAIN'?'本格使用跨區連接':'本格退回原選線，不能算新法成功'}`;el('tzScene').value=saved.scene||'';el('tzVerdict').value=saved.verdict||'';el('tzNote').value=saved.note||'';el('tzSaved').textContent=saved.saved_at?'已保存；可修改並另存新版本':'尚未判讀';};
  el('tzQuestion').onchange=()=>{current=Number(el('tzQuestion').value);show();};
  el('tzSave').onclick=async()=>{const button=el('tzSave');if(!el('tzScene').value||!el('tzVerdict').value){el('tzSaved').textContent='請選路段及路徑判斷，也可選無法判斷。';return;}button.disabled=true;try{const q=report.questions[current];const answer={scene:el('tzScene').value,verdict:el('tzVerdict').value,note:el('tzNote').value,run:q.run,seq:q.seq,t:q.t,source:'human_review',training_truth:false,saved_at:new Date().toISOString()};doc=await api('/api/document',{key,rev:doc.rev,data:{revision:report.revision,answers:{...(doc.data?.answers||{}),[q.id]:answer}}});el('tzSaved').textContent='已保存本題 v'+doc.rev+'；未套用車端';}catch(e){el('tzSaved').textContent='保存失敗，請保留畫面：'+e.message;}finally{button.disabled=false;}};
  el('tzRun').onchange=()=>{el('tzSignals').src=base+el('tzRun').value+'-signals.png';};el('tzRun').onchange();show();
 }catch(e){box.textContent='近中遠判讀讀取失敗：'+e.message;}
}
async function threeZoneWhenReady(){for(let i=0;i<120&&route==='models';i++){if(document.getElementById('researchModels')){await mountThreeZoneReview();return;}await new Promise(r=>setTimeout(r,100));}}
const beforeThreeZoneModels=modelsPage;
modelsPage=async()=>{await Promise.all([beforeThreeZoneModels(),threeZoneWhenReady()]);};
if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',()=>threeZoneWhenReady(),{once:true});else threeZoneWhenReady();
