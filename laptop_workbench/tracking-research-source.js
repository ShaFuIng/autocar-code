/* Research and proposal editor only. No vehicle/control endpoint is called. */
const trackingSources=[
 ['Nav2：Regulated Pure Pursuit 原始實作說明','https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/README.md'],
 ['Macenski 等：Regulated Pure Pursuit 論文','https://arxiv.org/abs/2305.20026'],
 ['Stanford：Stanley 原始論文（§9.2）','https://robots.stanford.edu/papers/thrun.stanley05.pdf'],
 ['Autoware：MPC Lateral Controller','https://autowarefoundation.github.io/autoware_universe/main/control/autoware_mpc_lateral_controller/'],
 ['Optuna：參數搜尋取樣器','https://optuna.readthedocs.io/en/stable/reference/samplers/index.html']
];
const trackingFields=[
 ['pwm','測試PWM上限 %（提案）',15,0,25,1],
 ['steer_rate','轉向指令速度 µs/s（提案）',300,50,800,1],
 ['lookahead_min','最小前視距離 m（後軸基準，暫擬）',0.8,0.1,5,0.01],
 ['lookahead_max','最大前視距離 m（暫擬）',1.2,0.1,5,0.01],
 ['lookahead_time','速度前視時間 s（暫擬）',1,0,5,0.1],
 ['curve_scale','彎道PWM比例（提案，非實測速度比例）',0.8,0,1,0.05],
 ['ramp_s','PWM上升時間 s（0代表直接指定）',0,0,1,0.05],
 ['image_timeout_ms','影像逾時 ms（提案，未驗證）',300,50,1000,10],
 ['latency_ms','實測全鏈延遲 ms（未知留空）',null,0,5000,1],
 ['near_cm','鏡頭至最近地面水平距離 cm',39,0,200,0.1],
 ['camera_ahead_cm','鏡頭在前軸前方 cm（基準待確認）',10,-100,100,0.1],
 ['wheelbase_cm','軸距 cm',23,1,100,0.1],
 ['front_diameter_cm','前輪外徑 cm（未實量；留空）',null,1,50,0.1],
 ['backlash_left_us','向左反轉補償 µs（未知、未啟用）',null,0,300,1],
 ['backlash_right_us','向右反轉補償 µs（未知、未啟用）',null,0,300,1]
];
function trackingProposalDefaults(){return Object.fromEntries(trackingFields.map(f=>[f[0],f[2]]));}
function validateTrackingProposal(d){
 for(const [key,label,,min,max] of trackingFields){const n=d[key];if(n===null){if(!['latency_ms','front_diameter_cm','backlash_left_us','backlash_right_us'].includes(key))throw Error(label+'不可空白');continue;}if(typeof n!=='number'||!Number.isFinite(n)||n<min||n>max)throw Error(label+'超出草稿範圍');}
 if(d.lookahead_min>d.lookahead_max)throw Error('最小前視不可大於最大前視');
 return d;
}
function trackingGeometry(d){const front=d.near_cm+d.camera_ahead_cm;return {front_axle_cm:front,rear_axle_cm:front+d.wheelbase_cm,front_edge_cm:d.front_diameter_cm===null?null:front-d.front_diameter_cm/2};}
function trackingResearchPanel(){return `<section class="card" id="trackingResearch">
<span class="pill">研究比較／參數提案 · 未部署、無車控輸出</span><h2>適合這台車的路徑追蹤與調參方案</h2>
<p>建議先以 <b>可調前視 Pure Pursuit＋彎道降速＋左右獨立轉向映射</b>作為低速試驗基線。這是根據今日資料與文獻作的工程判斷，不是已跑完的演算法成績，也不是換一個白線辨識模型。資料與文獻查核日：2026-10-18。</p>
<div class="ribbon">白線辨識 → 地面座標／可見範圍與不確定度 → 可行局部路徑 → 追蹤器計算曲率與速度 → 左右校準／轉向速率限制 → 車控授權與停止保護。每一層要能單獨回放和更換。</div>
<details open><summary>四種方法比較（建議與限制）</summary><div style="max-height:340px;overflow:auto" tabindex="0"><table style="min-width:780px"><thead><tr><th>方法</th><th>作用／優點</th><th>本車問題</th><th>採用順序</th></tr></thead><tbody>
<tr><td>視覺偏差 P／PD</td><td>對固定影像位置的偏差修正，易建立診斷基線。</td><td>像素偏差受距離與相機姿態影響；固定增益難跨PWM，近處不可見，積分可能在間隙中累積。</td><td>可作對照，不作追求速度的主方案；先不加入積分。</td></tr>
<tr><td>Adaptive／Regulated Pure Pursuit [1,2]</td><td>以車體座標的前視點求曲率；前視隨速度調整，彎道降低速度。</td><td>需地面尺度、可行路徑與轉向映射；前視過長可能切彎，過短可能擺動。視野盲區限制可用前視點。</td><td><b>第一候選</b>：借用方法，不要求先移植整套ROS。此車不能原地旋轉，不啟用rotate-to-heading。</td></tr>
<tr><td>Stanley [3]</td><td>以航向與前軸橫向偏差回授。</td><td>基本式含速度分母，接近零速需正則化；前軸附近不可見，偏差需靠短期歷史估計，不能從最下排像素當前軸誤差。</td><td>第二對照候選；先補姿態與近處路徑估計，再比較。</td></tr>
<tr><td>帶限制的運動學 MPC [4]</td><td>預測多步行為，納入轉向速率、延遲、速度與路徑約束。</td><td>需可信輪角／速度／延遲模型；目前間隙未知，錯誤模型會產生過度樂觀預測，調參成本高。</td><td>後續升級；左右映射與遲滯需自行建模，MPC不會自動消除鬆動。</td></tr>
</tbody></table></div></details>
<details open><summary>相機盲區與跑道餘裕：先統一距離基準</summary>
<p>2026-10-19 相機下俯後：軸距23cm、鏡頭前伸10cm（按前軸量，但註記仍需確認）、鏡頭到最近可見地面32cm（暫按水平距離）。因此最近可見地面約在<b>前軸前42cm、後軸前65cm</b>。前輪直徑未量；若暫借7.5cm作示例，輪胎最前緣到可見地面約38.25cm，這不是實量。</p>
<p>Pure Pursuit若採後軸中心為原點，直接可見的前視點須在可見區域內；不能把鏡頭近界距離直接當後軸前視距離。彎道上的點還要考慮橫向位置與視野邊界。過去看過而現已進入盲區的路段，可用短時里程／姿態傳遞，但誤差會累積；兩輪里程不能在打滑時提供可靠姿態保證。無有效路徑或影像逾時時應停止，不能長時間照上一個轉向盲走。</p>
<p>淨寬46cm、輪胎外廓32cm：置中直線每側僅<b>7cm</b>名義餘裕，轉彎需檢查整個輪胎掃掠範圍。不是允許橫向誤差7cm，還要扣除辨識、幾何、控制與機構不確定度。路徑要求比已知能力更緊時，降速本身不保證幾何上轉得過去。</p>
<p>延遲行進距離＝速度×全鏈延遲；全鏈包括曝光／取像、推論、通訊、控制與轉向響應。以0.213m/s、假設100ms為例，已前進約2.1cm。300µs/s從1300到1900需2秒，若維持0.213m/s會前進約42.6cm：這是理想指令時間示例，非實測輪角動態。增加轉向指令速度可減少此項等待，卻不等於消除機構延遲。</p></details>
<details><summary>左右不對稱與間隙的處理方式</summary><p>20%下已量左940外輪半徑82.5±2.5cm、右1900為52.5±2.5cm；同樣106.5cm外輪半徑，左需1030、右需1750。採左右獨立單調查表，保留誤差與接近方向；1450記為未量得，不填零曲率。外輪半徑不能直接當後軸中心曲率的倒數：需確認前／後輪量測基準與幾何，或另量後軸中心軌跡後再建立曲率映射。</p><p>預留「前次轉向方向／反轉量／補償上限／信心」欄位。現在不把約50µs觀察硬寫成補償，不自動跳動吃間隙；先用命令限速、適量平滑並記錄反向次數。濾波太強會多出延遲。真正回差補償要先做同一指令由兩側接近的成對量測；機械鬆動過大時應先改善機構。</p></details>
<details><summary>明天試跑的分階段驗收（尚未實作自動循跡）</summary><ol>
<li>靜止／手動錄影：核對左右符號、地面尺度、最近可見範圍、白線雙側與單側狀態、影像時間戳與全鏈延遲。</li>
<li>影子回放：同一影片比PP／Stanley輸出、限幅與方向反轉；顯示目標路徑、前視點、曲率、預計µs與PWM。固定影片不會隨候選控制改變車位，所以不能據此宣稱閉環能跑完。</li>
<li>離線閉環模擬：把量測誤差、左右映射、未知延遲／間隙作範圍擾動；未知範圍是情境假設，不是實測。</li>
<li>具備實際控制轉接與停止驗證後，另行確認一輪低速短路段；保持人工接管、限時、掉線／掉路徑停止。不直接連跑整圈，不自動提高PWM。15%只是今日已量直行檔，非循跡驗證檔；20%轉圈表跨用到15%仍需確認。</li>
<li>先調前視與轉向響應，再固定它們比較PWM檔位；每輪保存路段、配置版本、偏差、壓線／介入、掉線、延遲、轉向反轉／飽和、左右輪速與停止狀況。是否通過以輪胎外緣與淨跑道判斷。</li></ol><p>本次完成研究與提案保存，不會讓現有手動按鈕變成自動循跡。明天能否試跑取決於上述通路與短路段驗收，現在不能保證全跑道完成。</p></details>
<details><summary>自動調參可以做，但先離線、再一輪一輪驗證</summary><p>可以。先以小範圍網格／隨機搜尋找出穩定區域，再用Optuna的TPE等方法縮小候選 [5]。先搜前視距離／時間、轉向指令速率、濾波與彎道動力策略；機構映射和誤差範圍需有版本，不能靠最佳化掩蓋錯誤標定。</p><p>評分先淘汰壓線、失聯、無路徑、頻繁飽和與人工接管的候選，再比較最大／95百分位偏差、方向反轉及完成時間；未完成不能靠短時間得高分。用未參與調參的影片與路段驗證，直線／左彎／右彎分開評分。離線回放只能篩感知與命令特徵；閉環成績須模擬或實跑取得。</p><p>實車階段採「程式提出下一組 → 人工確認現場並授權單輪 → 保存結果 → 提出下一組」。自動搜尋不可改寫硬體安全上限、急停、心跳或車上模型，也不在無人場地自行加速。高PWM需另建速度資料及轉彎／停止驗證，不能把今天15%直行和20%轉圈外推成高速能力。</p></details>
<details><summary>未來可替換的介面與每輪紀錄</summary><pre style="max-height:240px;overflow:auto">PerceptionFrame: capture_time, image_mode, ground_points_m, visibility_polygon, confidence, calibration_rev
VehicleState: time, rear_axle_pose, speed_mps, uncertainty, last_steer_command_us, last_direction
TrackerProposal: curvature_1pm, target_speed_mps, lookahead_m, path_valid, reason
ActuatorMap: left/right calibration_rev, approach_direction, pwm_band, hysteresis_state
CommandProposal: steer_us, steer_rate_us_s, right/left_pwm_pct, expires_at, saturation_reason
TrialRecord: config_rev, route_segment, video_id, timestamps, commands, telemetry, observations, interventions</pre><p>保留固定PWM與將來輪速閉環兩種動力介面；PWM不是實際速度。命令必須經單一安全閘門才可出車控，提案、保存與影子回放都不能啟動。預留飽和回報、掉線停止及回退配置；現在僅保存契約草案，尚未接成執行API。</p></details>
<details><summary>參數提案編輯（只保存筆電，不套用實車）</summary><p>下面是可修改、保存、恢復與匯出的研究草稿。初值是待驗證提案，不是已採用參數；不會變更02實量、04手動設定或車上模型。未知值保留空白，回差補償始終未啟用。</p><form id="trackingProposalForm"><div class="formgrid">${trackingFields.map(([k,l,v,min,max,step])=>`<label>${esc(l)}<input id="tp-${k}" type="number" min="${min}" max="${max}" step="${step}" value="${v??''}"></label>`).join('')}</div><p id="trackingGeometryPreview" class="ribbon"></p><div class="actions"><button type="button" id="trackingSave">保存提案至筆電</button><button type="button" id="trackingRestore">恢復最近保存提案</button><button type="button" id="trackingExport">匯出提案 JSON</button></div><p id="trackingDraftState" role="status">載入提案…</p></form></details>
<details><summary>原始來源（外部連結；正文可離線閱讀）</summary><ol>${trackingSources.map(([t,u])=>`<li><a href="${u}" target="_blank" rel="noopener">${t}</a></li>`).join('')}</ol><p>方法介紹依原始論文／專案文件；本車優先順序、盲區計算、測試流程與介面是依本次資料提出的設計判斷。未移植來源專案、未安裝套件。</p></details></section>`;}
async function bindTrackingProposal(){
 const key='tracking-proposal',draftKey='wb-tracking-proposal-draft';let stored={rev:0,data:{}},lastValid=null;
 try{stored=await api('/api/document/'+key);}catch(e){if(!$('trackingDraftState'))return;$('trackingDraftState').textContent='資料庫讀取失敗，可編輯瀏覽器草稿：'+e.message;}
 if(!$('trackingProposalForm'))return;
 let current={...trackingProposalDefaults(),...stored.data.parameters};
 let local=null;try{local=JSON.parse(localStorage.getItem(draftKey)||'null');}catch{}
 if(local?.parameters)current={...current,...local.parameters};
 function fill(p){for(const [k] of trackingFields)$('tp-'+k).value=p[k]??'';}
 function read(){return validateTrackingProposal(Object.fromEntries(trackingFields.map(([k])=>[k,$('tp-'+k).value===''?null:Number($('tp-'+k).value)])));}
 function preview(){try{const p=read(),g=trackingGeometry(p);lastValid=p;$('trackingGeometryPreview').textContent=`幾何草稿：前軸前 ${g.front_axle_cm.toFixed(1)} cm；後軸前 ${g.rear_axle_cm.toFixed(1)} cm；前輪最前緣前 ${g.front_edge_cm===null?'未知（前輪直徑未量）':g.front_edge_cm.toFixed(1)+' cm'}。`+(p.lookahead_min*100<g.rear_axle_cm?'最小前視短於直線直接可見起點；不能直接當作看得到。':'前視仍須確認落在實際可見區域。')+` 彎道PWM提案 ${(p.pwm*p.curve_scale).toFixed(1)}%，可能低於負載下持續轉動門檻；不代表速度比例。`;return true;}catch(e){$('trackingGeometryPreview').textContent=e.message;return false;}}
 fill(current);preview();$('trackingDraftState').textContent=local?'已恢復瀏覽器草稿；未套用實車':`筆電提案 v${stored.rev}；未套用實車`;
 $('trackingProposalForm').onsubmit=e=>e.preventDefault();
 $('trackingProposalForm').oninput=()=>{if(preview()){localStorage.setItem(draftKey,JSON.stringify({rev:local?.rev??stored.rev,parameters:lastValid}));$('trackingDraftState').textContent='瀏覽器草稿已保存，尚未保存至筆電資料庫；不控制實車';}};
 $('trackingSave').onclick=async()=>{try{const parameters=read();const result=await api('/api/document',{key,rev:local?.rev??stored.rev,data:{schema_version:1,mode:'proposal_only',hardware_output:false,controller_candidate:'adaptive_pure_pursuit_regulated_speed',parameters,backlash_compensation_enabled:false,calibration_reference:'20261018',sources:trackingSources}});stored=result;local=null;localStorage.removeItem(draftKey);$('trackingDraftState').textContent=`已保存至筆電 v${stored.rev}；未套用實車`;}catch(e){if(!$('trackingDraftState'))return;$('trackingDraftState').textContent='未保存：'+e.message;}};
 $('trackingRestore').onclick=async()=>{try{stored=await api('/api/document/'+key);local=null;localStorage.removeItem(draftKey);fill({...trackingProposalDefaults(),...stored.data.parameters});preview();$('trackingDraftState').textContent=`已恢復筆電 v${stored.rev}；未套用實車`;}catch(e){$('trackingDraftState').textContent=e.message;}};
 $('trackingExport').onclick=()=>{try{const proposal={schema_version:1,mode:'proposal_only',hardware_output:false,parameters:read(),backlash_compensation_enabled:false};const url=URL.createObjectURL(new Blob([JSON.stringify(proposal,null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download='tracking-proposal.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}catch(e){$('trackingDraftState').textContent=e.message;}};
}
const trackingResearchModelsBase=modelsPage;
function mountTrackingResearch(){if(route!=='models'||$('trackingResearch')||!$('editor'))return;$('main').insertAdjacentHTML('afterbegin',trackingResearchPanel());bindTrackingProposal();}
modelsPage=async()=>{await trackingResearchModelsBase();mountTrackingResearch();};
// The first async route can start before deferred extensions load. Mount again
// when that route replaces main, without invoking any analysis or vehicle API.
new MutationObserver(mountTrackingResearch).observe($('main'),{childList:true});
mountTrackingResearch();
