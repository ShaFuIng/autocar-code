import json,shutil,hashlib
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'laptop_workbench/data/experiments/k-approach'
r=json.loads((out/'report.json').read_text(encoding='utf8'))
datasets={k:json.loads((out/(k+'.json')).read_text()) for k in ['latest','fullclip']}
entries=[]
for case,rows in datasets.items():
 for prev,x in zip(rows,rows[1:]):
  p,q=prev['candidate']['pid'],x['candidate']['pid']
  if p['valid'] and q['valid']:assert abs(q['steer_us']-p['steer_us'])<=300*(x['captured']-prev['captured'])+1e-6
  if x['candidate']['reason']=='APPROACH_SEQUENCE_AND_CURRENT_ENTRANCE':
   g=x['candidate']['gate'];assert g['reference_compatible'] and g['entry_check_passed']
   entries.append(dict(case=case,seq=x['seq'],steer_step_us=q['steer_us']-p['steer_us'],max_entry_error_px=max(abs(z) for z in g['entrance_errors'].values()),approach_progress_px=g['approach_progress_px']))
for name in ['curve23','adjacent97','curve142']:
 s=r['summary'][name];assert s['path_changes']==0 and not any('CROSS' in x[1] for x in s['candidate_modes'])
r.update(total_replayed_frames=sum(len(x) for x in datasets.values()),parallel_validation=json.loads((out/'parallel-validation.json').read_text()),handoffs=entries,
 checks=['No new false K handoffs in full original 1760-frame replay','Three target negative intervals retain exact paths/modes','Latest 102-frame parallel candidate matches sequential path/mode exactly','Entry mismatch, expired latch, static branches, single line and missing reference do not trigger alternative handoff','Existing steering slew limit respected'],
 parameters={'approach_window_s':1.5,'minimum_supported_observations':5,'opening_progress_px_at_640x360':12,'retention_after_last_evidence_s':1,'note':'Offline heuristic candidate values, not validated safety thresholds; original near-entry route and exit logic retained'},
 limits=['Latest recording ends inside intersection; no new-run exit footage to validate return A','Replay does not predict the different camera trajectory a changed real steering command would cause','Some latest original images are nearest AVI frames, not bit-identical control images','No retraining, deployment, live PID start or hardware action; 400ms unchanged'])
r['baseline_sha256']={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (out/'baseline').glob('*.py')}
(out/'report.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf8')
f=root/'laptop_workbench/integration.js';backup=out/'integration-before.js'
if not backup.exists():shutil.copy2(f,backup)
s=f.read_text(encoding='utf8').replace('workbench-tunnel-reference-offline-20261024.51','workbench-approach-offline-20261024.52')
for key,value in [('status','路口接近時序候選已完成離線比較，未部署；Orin仍是原完整K動態取樣r1。最新輪第77格接管，原第139格不再空路徑。'),('acceptance','最新102格＋原長片1760格比較；23.4／97.3／142秒前後無誤切且路徑一致。原兩路口提早3／11格接管，接回A時點不變。平行候選102格與順序計算一致；400ms未改。')]:
 start=s.index(' '+key+':',s.index('const WORKBENCH_RELEASE='));end=s.index('\n',start);s=s[:start]+' '+key+':'+json.dumps(value,ensure_ascii=False)+','+s[end:]
s=s.replace(' pending:[',' pending:[\n "路口接近時序候選僅離線；最新輪錄影止於路口內，該輪接回A與改變行車軌跡後效果尚無實車資料。等待部署指示。",',1)
s+='''
const approachModelsBase=modelsPage;
modelsPage=async()=>{await approachModelsBase();if(route!=='models'||$('approachReport'))return;$('main').insertAdjacentHTML('afterbegin',`<section class="card" id="approachReport"><h2>完整K · 路口接近時序候選（離線，未部署）</h2><p>保留已確認的接近過程：開口往近處移動＋連續本車道支援，不再硬等近處三段白線。K仍須當前影格入口核對與連續確認；單線不作新增接管依據。逾時的接近記憶會失效。</p><table><tr><th>區間</th><th>離線比較</th></tr><tr><td>最新一輪102格</td><td>第77格接管K；第139格原空路徑恢復。接管提案變動0.9 µs</td></tr><tr><td>原路口1／2</td><td>提早3／11格接管；原接回A時點不變，沒有新增反覆切換</td></tr><tr><td>23.4／97.3／142秒前後</td><td>無誤切K，路徑與基準相同</td></tr></table><p>原長片1760格完整重播，保留前段歷史；最新輪102格平行候選與順序計算結果一致。只改切換，未加入隧道候選；原版保留、400ms不變。</p><p>限制：最新輪停在路口內，不能確認該輪完整通過及接回A；離線畫面不會隨新轉向改變，不能當成實車成功。</p><details><summary>最新輪同影格比較（左原版／右候選；紫色遮罩／青色路徑）</summary><img src="/experiment/k-approach/comparison.jpg" style="max-width:100%"></details><a href="/experiment/k-approach/report.json">完整比較與切換時間紀錄</a></section>`)};
'''
f.write_text(s,encoding='utf8')
with (root/'laptop_workbench/README.md').open('a',encoding='utf8') as f:f.write('\n\n## .52 路口接近時序候選（僅離線）\n資料data/experiments/k-approach，程式tools/junction_approach_candidate.py，平行入口候選candidate-runtime/k_parallel_path.py。車端web原碼與Orin均未修改。最新102格第77格接管，空路徑1→0；原1760格重播兩路口提早3/11格、接回時間不變，三個負例區間無誤切。平行實際重播102格路徑／模式一致。接近記憶有界，不繞過入口核對；既有400ms與PID設定不變。總覽與模型頁已同步。\n')
print('Published offline approach report; no vehicle deployment')
