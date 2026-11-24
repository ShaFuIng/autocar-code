import json,shutil,hashlib
from pathlib import Path
root=Path('Orin_Interface_v1');out=root/'laptop_workbench/data/experiments/k-branch-temporal';r=json.loads((out/'report.json').read_text(encoding='utf8'))
for label,folder,names in [('baseline',root/'web',['junction_approach_candidate.py','k_parallel_path.py']),('baseline',root/'web/k_lane_runtime',['junction_k_lane.py','road_segments_v32.py']),('candidate',root/'tools',['junction_branch_candidate.py','compare_branch_temporal.py','verify_branch_temporal.py'])]:
 dest=out/label;dest.mkdir(exist_ok=True)
 for n in names:shutil.copy2(folder/n,dest/n)
rows=json.loads((out/'latest.json').read_text());r['control_steps']={}
for key in ['baseline','candidate']:
 steps=[]
 for a,b in zip(rows,rows[1:]):
  if 450<=b['seq']<=490 and a[key]['pid']['valid'] and b[key]['pid']['valid']:
   step=abs(b[key]['pid']['steer_us']-a[key]['pid']['steer_us']);assert step<=300*(b['t']-a['t'])+1e-5;steps.append(step)
 r['control_steps'][key]={'max_steer_step_us':max(steps)}
r['result']='路口漏切改善，負例維持原結果；第二入口接管後藍線跳幅仍待改善，不標示全面通過'
r['limitations'].append('第二入口seq450～490固定影像列的最大路徑跳幅12.27→26.48px、P95 10.26→13.28px；接管當格僅4.29px，但後續K參考仍有跳動，未修改K幾何或加大400ms。')
r['baseline_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (out/'baseline').glob('*.py')}
(out/'report.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf8')
f=root/'laptop_workbench/integration.js';shutil.copy2(f,out/'integration-before.js');s=f.read_text(encoding='utf8').replace('workbench-i2c-retry-deployed-20261024.55','workbench-branch-offline-20261024.56')
for key,value in [('status','第二入口分支時序候選已完成離線比較，未部署。新版第460格接管；原兩路口接回時間不變，三個負例無新增誤切。K接管後參考跳幅仍待改善。'),('acceptance','最新輪588筆（587控制＋1停止診斷）與原長片1760格重播。第二入口約41.4秒接管，接管跳幅4.29px／提案2.91µs；後續最大路徑跳幅26.48px，比原版增加，未標示全面通過。400ms不變，未部署或動車。')]:
 a=s.index(' '+key+':',s.index('const WORKBENCH_RELEASE='));b=s.index('\n',a);s=s[:a]+' '+key+':'+json.dumps(value,ensure_ascii=False)+','+s[b:]
s=s.replace(' pending:[',' pending:[\n "分支時序候選已解決本輪第二入口漏切；K後續參考跳動需處理，僅離線，Orin維持原版。",',1)
s=s.replace('<section class="card" id="recentAcceptance"><h2>最近驗收</h2>','<section class="card" id="recentAcceptance"><h2>最近驗收</h2><p><b>第二入口分支時序候選 · 僅離線，未部署</b> · <a href="#models">第六頁查看同影格比較</a> · <a href="/experiment/k-branch-temporal/report.json">比較報告與剩餘問題</a></p>',1)
s+='''
const branchTemporalModelsBase=modelsPage;
modelsPage=async()=>{await branchTemporalModelsBase();if(route!=='models'||$('branchTemporalReport'))return;
 const r=await api('/experiment/k-branch-temporal/report.json');if(route!=='models')return;
 $('main').insertAdjacentHTML('afterbegin',`<section class="card" id="branchTemporalReport"><h2>第二入口 · 分支時序核對（離線候選，未部署）</h2><p>允許不連通、但與入口兩側及前後影格一致的分支；短暫雙線缺失保留接近證據，K仍需當格入口核對。歷史對齊失效時，單線只保留觀測支持段。</p><table><tr><th>比較區間</th><th>結果</th></tr><tr><td>剛剛第二入口</td><td>原版未切K → 第460格、約41.4秒接管；第553格開始接回，第557格回A。</td></tr><tr><td>原路口1／2</td><td>第一段提早8格接管；第二段相同。兩段接回A時點不變，沒有新增模式往返。</td></tr><tr><td>23.4／97.3／142秒負例</td><td>路徑與模式逐格相同，無新增誤切。</td></tr></table><p><b>仍有問題：</b>接管當格跳幅4.29px、PID提案2.91µs；第二入口後續最大路徑跳幅由12.27增至26.48px（P95由10.26增至13.28px）。所以漏切改善，但K參考穩定性尚未全面通過。未改400ms或車端程式。</p><p>左：保留原版；右：候選。原始錄影不會隨候選轉向改變，因此不能推定實車已走對出口。</p><select id="branchTemporalFrame">${r.cards.map(x=>`<option value="${x.seq}">${x.seconds}秒 · 第${x.seq}格 · ${x.baseline} → ${x.candidate}</option>`).join('')}</select><p id="branchTemporalReason"></p><img id="branchTemporalImage" style="max-width:100%"><p><a href="/experiment/k-branch-temporal/report.json">完整離線報告</a></p></section>`);
 const draw=()=>{const c=r.cards.find(x=>x.seq==$('branchTemporalFrame').value);$('branchTemporalImage').src='/experiment/k-branch-temporal/'+c.image;$('branchTemporalReason').textContent=c.reason;};$('branchTemporalFrame').onchange=draw;$('branchTemporalFrame').value='460';draw();
};
'''
f.write_text(s,encoding='utf8')
f=root/'laptop_workbench/README.md';f.write_text(f.read_text(encoding='utf8')+'\n\n## .56 第二入口分支時序候選（僅離線）\n工具junction_branch_candidate.py，資料data/experiments/k-branch-temporal；保留baseline及候選原碼。原長片1760格＋最新588筆凍結幾何重播，含1筆停止診斷。第460格接管、553/557接回；負例逐格相同，原兩路口接回不變。接管4.29px，但後續最大跳幅12.27→26.48px，不能宣稱全面改善；需另處理K參考穩定性。僅工具與工作台報告更新，未修改web算法、未部署、未動車、400ms不變。\n',encoding='utf8')
print(json.dumps(r['control_steps']))
