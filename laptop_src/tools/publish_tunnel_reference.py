from pathlib import Path
import json,shutil,hashlib
root=Path(__file__).resolve().parents[1];out=root/'laptop_workbench/data/experiments/k-tunnel-reference'
r=json.loads((out/'report.json').read_text(encoding='utf8'));d=json.loads((out/'comparison.json').read_text(encoding='utf8'))
last=d['tunnel'][-1];previous=d['tunnel'][-2]
r.update(diagnosis={'mask':'Warm tape missing in original class-1 model output before path association; daylight tape remains. Lighting correlation, not proof of missing training labels.',
 'flow':'TOO_FEW_FEATURES returns tracks=0 before LK; not a count of all current-image features. Retained anchor is retried after failure. Nearest-AVI replay anchor becomes 4.025s old; live exact anchor not recorded.',
 'current_image_features':'420-430 nearest AVI frames all reach 250 features using original settings; AVI compression differs from live source.',
 'candidate':'Current-mask far pair + prior width profile + current image contrast edges + three consecutive confirmations. No unaligned history positions, no mask mutation or history refresh with assisted edges.'},
 handoff={'previous_row':previous['candidate']['pid']['sample_row_ratio'],'current_row':last['candidate']['pid']['sample_row_ratio'],
 'steer_step_us':last['candidate']['pid']['steer_us']-previous['candidate']['pid']['steer_us'],'derivative':last['candidate']['pid']['d'],'physical_response_verified':False},
 tests='Flat image, reversed contrast, empty mask, consecutive confirmation, intersection exact PID equality, existing steering slew bound passed',
 warmup_note='One invalid PID result in each replay is initial WARMUP, not loss of path',
 remaining=['History alignment remains unavailable; separate adjacent-frame recovery needed, never blindly warp stale anchor',
 'Near-single-line path still shrinks before candidate confirms; final row changes 93% to 67%',
 'Extra pixel checks increase laptop processing cost; Orin latency not measured',
 'No footage after this stop demonstrating sustained passage; not closed-loop validation'])
r['baseline_sha256']={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (out/'baseline').glob('*.py')}
(out/'report.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf8')
f=root/'laptop_workbench/integration.js';backup=out/'integration-before.js'
if not backup.exists():shutil.copy2(f,backup)
s=f.read_text(encoding='utf8');s=s.replace('workbench-k-adaptive-deployed-20261024.50','workbench-tunnel-reference-offline-20261024.51')
for key,value in [('status','Orin仍為完整K動態取樣r1；新增隧道遠雙線像素核對離線候選，未部署。這輪第430格空路徑恢復，兩路口372格路徑／切換／PID提案相同。'),('acceptance','離線447格比較：隧道75格空路徑1→0，路口372格逐格相同。接回取樣93%→67%、轉向提案35.5µs；平均筆電計算29→49ms。歷史對齊仍待處理，非實車通過證明。')]:
 start=s.index(' '+key+':',s.index('const WORKBENCH_RELEASE='));end=s.index('\n',start);s=s[:start]+' '+key+':'+json.dumps(value,ensure_ascii=False)+','+s[end:]
s=s.replace(' pending:[',' pending:[\n "隧道像素核對候選僅離線：歷史參考恢復、接回取樣範圍變化與額外計算成本仍待改善；Orin未更新。",',1)
s+='''
const tunnelReferenceModelsBase=modelsPage;
modelsPage=async()=>{await tunnelReferenceModelsBase();if(route!=='models'||$('tunnelReferenceReport'))return;$('main').insertAdjacentHTML('afterbegin',`<section class="card" id="tunnelReferenceReport"><h2>隧道暖光 · 遠雙線核對候選（僅離線、未部署）</h2><p>原遮罩保留。紫色：模型遮罩；橘色：當前影像亮帶邊緣核對（不是模型辨識或人工真值）；青色：使用的中心。</p><p>最新一輪75格空路徑1→0，第430格經連續3格確認後接回；原兩路口372格路徑、切換與PID提案相同。初始WARMUP另計。</p><p>限制：歷史對齊仍不可用；接回取樣93%→67%，提案變動35.5 µs受原速率限制。筆電平均計算29→49 ms，未測Orin控制延遲。沒有重新訓練、部署或動車，400ms不變。</p><details><summary>停止前後同影格比較</summary><img src="/experiment/k-tunnel-reference/comparison.jpg" style="max-width:100%"><p>原圖使用最近錄影影格，部分與控制影格相差最多約60ms；遮罩保留該控制影格原紀錄。</p></details><a href="/experiment/k-tunnel-reference/report.json">完整診斷與離線結果</a></section>`)};
'''
f.write_text(s,encoding='utf8')
with (root/'laptop_workbench/README.md').open('a',encoding='utf8') as f:f.write('\n\n## .51 隧道遠雙線核對候選（僅離線）\n工具 tools/tunnel_reference_candidate.py；未修改或部署web車端算法。原版保存在data/experiments/k-tunnel-reference/baseline。原始模型遮罩不動，遠雙線需要當前影像亮帶支持、歷史寬度與連續確認；不把核對結果更新成模型歷史。75格隧道空路徑1→0，兩路口372格路徑／模式／PID一致。接回範圍跳變、歷史對齊失敗與額外約19ms筆電開銷仍需處理。總覽版本與模型頁已同步；Orin仍為.50所述部署版本。\n')
print('Offline report published; no deployment')
