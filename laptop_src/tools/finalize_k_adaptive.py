from pathlib import Path
import json,hashlib,shutil
root=Path(__file__).resolve().parents[1];out=root/'laptop_workbench/data/experiments/k-adaptive'
report=json.loads((out/'report.json').read_text(encoding='utf-8'));comparisons=json.loads((out/'comparison.json').read_text())
for case in ('intersection1','intersection2'):
 assert all(all(r['baseline'].get(k)==r['candidate'].get(k) for k in ['valid','reason','error','steer_us','desired_us','p','i','d','pwm','backlash_offset_us']) for r in comparisons[case])
report['intersection_pid_proposals_exactly_equal']=True
report['tail']=json.loads((out/'tail-replay.json').read_text())
report['deployment']=json.loads((out/'deployment.json').read_text(encoding='utf-8'))
report['post_status']=json.loads((out/'post-status.json').read_text(encoding='utf-8-sig'))
preview=root/'laptop_workbench/data/experiments/k-parallel/preview-junction-k-lane-v3.json'
shutil.copy2(preview,out/'stationary-preview.json')
report['stationary_preview']={'samples':10,'run':None,'note':'sampling active; current camera view still mostly invalid, not a road-following acceptance'}
report['status']='deployed_verified_no_motor_start'
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
# Keep laptop-only existing features; apply just the same sampling changes.
f=root/'web/live_pid.py';(out/'before/live_pid-local.py').write_bytes(f.read_bytes());s=f.read_text(encoding='utf-8')
patch=(root/'tools/prepare_k_adaptive.py').read_text(encoding='utf-8');patch=patch[patch.index("s=s.replace("):patch.index("assert s.count")];ctx={'s':s};exec(patch,ctx);s=ctx['s'];compile(s,'live_pid.py','exec');f.write_text(s,encoding='utf-8')
f=root/'laptop_workbench/integration.js';s=f.read_text(encoding='utf-8').replace('workbench-i2c-timing-deployed-20261024.49','workbench-k-adaptive-deployed-20261024.50')
for key,value in [('status','完整K動態PID取樣r1已部署，400ms不變。第374格改取61.85%，兩路口372格轉向提案與基準一致。靜態入口已驗證、目前idle/LOCKED/雙輪0；隧道仍有漏辨識，未宣稱已通過實車。'),('acceptance','712格已存路徑重播：隧道第374格不再因取樣越界失效，路口轉向提案逐格一致。補算停止後4格中3格可用、最後空路徑仍停止。無馬達啟動；保留診斷v2。')]:
 start=s.index(' '+key+':',s.index('const WORKBENCH_RELEASE='));end=s.index('\n',start);s=s[:start]+' '+key+':'+json.dumps(value,ensure_ascii=False)+','+s[end:]
s=s.replace(' pending:[',' pending:[\n "K動態取樣已部署；使用者自行試跑。隧道完全空路徑仍停止，訓練資料與光流改善另案處理。",',1)
s+='''
const kAdaptiveModelsBase=modelsPage;
modelsPage=async()=>{await kAdaptiveModelsBase();if(route!=='models'||$('kAdaptiveReport'))return;$('main').insertAdjacentHTML('afterbegin',`<section class="card" id="kAdaptiveReport"><h2>完整K · 動態PID取樣 r1（已部署Orin）</h2><p>第374格：固定60%無法取樣 → 61.85%落在有效連續路徑內，轉向提案變動16.2 µs。只改取樣，不改路徑與K切換；400ms不變。</p><table><tr><th>比較</th><th>結果</th></tr><tr><td>本輪340格既有路徑</td><td>取樣失效1→0；第374格前提案一致</td></tr><tr><td>原兩路口181＋191格</td><td>轉向提案逐格一致；未新增失效</td></tr><tr><td>停止後4格重新辨識</td><td>3格能取樣，最後1格空路徑仍停止</td></tr></table><p>靜態預覽已確認動態取樣生效；所在位置仍常無有效路徑，不代表隧道已可通過。原PID與完整K基準保留，沒有啟動馬達。</p><a href="/experiment/k-adaptive/report.json">比較與部署紀錄</a><details><summary>停止前後離線補算圖（黃：原取樣；綠：新取樣）</summary><p>後4格重新以v4辨識；前段原圖使用最近錄影影格，不是逐位元相同即時重播。</p><img src="/experiment/k-adaptive/tail-review.jpg" style="max-width:100%"></details></section>`)};
'''
f.write_text(s,encoding='utf-8')
with (root/'laptop_workbench/README.md').open('a',encoding='utf-8') as f:f.write('\n\n## .50 K動態PID取樣已部署\n新增web/k_adaptive_pid.py，僅完整K控制／預覽接入，未改ROAD_ALGORITHMS避免影響模型worker路徑。採既有影像路徑連續界限，微分在新取樣位置重算，保留轉向速率及400ms。原版遠端備份 '+report['deployment']['backup']+'，本機before/live_pid-local.py。712格既有路徑比較及停止後4格補算，最後空路徑仍失效。必要服務重啟後MainPID17413，靜態預覽完成並停止，無PID run、LOCKED、雙輪0、診斷v2/I²C0。結果data/experiments/k-adaptive/report.json；未測閉環動態改善。\n')
print('Report, workbench overview/models, and local source updated')
