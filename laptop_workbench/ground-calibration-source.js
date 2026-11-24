/* Saved candidate report. Reads local measurements; no hardware output. */
(()=>{
let loading=false;
async function mount(){
 if(location.hash!=='#config'||loading||document.getElementById('groundCandidate')||!document.getElementById('cameraMeasure'))return;
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
