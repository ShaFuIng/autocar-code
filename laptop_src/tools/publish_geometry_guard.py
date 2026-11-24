import json,shutil,hashlib
from pathlib import Path
root=Path('Orin_Interface_v1');out=root/'laptop_workbench/data/experiments/k-geometry-guard';r=json.loads((out/'report.json').read_text(encoding='utf8'));r['version']='geometry-guard-r3';r['checks']={};r['focus']=[]
for name in ['latest','fullclip']:
 rows=json.loads((out/(name+'.json')).read_text());assert all(x['baseline']['mode']==x['candidate']['mode'] for x in rows)
 deltas=[abs((x['baseline']['pid']['steer_us'] or 0)-(x['candidate']['pid']['steer_us'] or 0)) for x in rows]
 r['checks'][name]={'same_modes_all_frames':True,'control_proposal_changed_frames':sum(d>.001 for d in deltas),'max_variant_steer_difference_us':max(deltas),'additional_empty':sum(not x['candidate']['valid'] for x in rows)-sum(not x['baseline']['valid'] for x in rows),'additional_invalid_pid':sum(not x['candidate']['pid']['valid'] for x in rows)-sum(not x['baseline']['pid']['valid'] for x in rows)}
 if name=='latest':
  for x in rows:
   if 438<=x['seq']<=442:
    def jump(key):
     p=x[key]['path'];return max([abs(b[0]-a[0]) for a,b in zip(p,p[1:]) if 0<b[1]-a[1]<=16] or [0])
    r['focus'].append({'seq':x['seq'],'cutoff_y':x['guard']['cutoff_y'] if x['guard'] else None,'before_spatial_step_px':jump('baseline'),'after_spatial_step_px':jump('candidate'),'same_pid_proposal':x['baseline']['pid']==x['candidate']['pid']})
 else:
  for label,lo,hi in [('intersection1',640,820),('intersection2',1010,1200),('curve23',210,260),('adjacent97',940,1000),('curve142',1390,1450)]:
   xs=[x for x in rows if lo<=x['seq']<=hi];r['checks'][label]={'max_variant_steer_difference_us':max(abs((x['baseline']['pid']['steer_us'] or 0)-(x['candidate']['pid']['steer_us'] or 0)) for x in xs)}
r['scope']='本輪在前版分支時序候選之上加輸出幾何核對。保留前版及已部署版；新候選不更新A歷史，不以截短結果增加切換證據。'
r['limitations']=['最新輪588筆包含587控制樣本及1筆停止診斷，錄影使用最近時間配對。','切斷錯誤接點而非重建缺失的遠方道路；綠線為保留邊界，藍線為輸出中心。','K幾何維持原樣，前版接管後26.48px時序跳幅未由本輪修復。','完整長片另有95格PID提案與前版不同，最大版本間差207.75µs，並非單步跳幅；新取樣範圍會影響PID及後續狀態，尚未全面確認控制改善。','此為凍結幾何後處理與逐格狀態重播，不代表實車閉環行車成功，未部署、未動車、400ms不變。']
r['heuristics']={'boundary_angle_deg':55,'boundary_residual_px':12,'join_delta_px':20,'join_max_slope':4,'local_match_rows_px':24,'note':'640×360座標中的候選初值；邊界急彎需同區域中心來源接點不連續佐證，避免只因急彎截線。'}
for label,folder,files in [('baseline',root/'tools',['junction_branch_candidate.py']),('candidate',root/'tools',['junction_geometry_guard.py','compare_geometry_guard.py'])]:
 d=out/label;d.mkdir(exist_ok=True)
 for f in files:shutil.copy2(folder/f,d/f)
(out/'report.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf8')
f=root/'laptop_workbench/integration.js';shutil.copy2(f,out/'integration-before.js');s=f.read_text(encoding='utf8').replace('workbench-branch-offline-20261024.56','workbench-geometry-offline-20261024.57')
for key,val in [('status','邊界分段／拼接核對候選已完成離線比較，未部署。第439～442格錯誤接點截除，路口切換不變；整段其他取樣變化仍待確認。'),('acceptance','最新588筆＋原長片1760格重播；438～442格PID提案相同、無新增空路徑或誤切。第440格約76px錯誤接點已截除；整段其他區段最大版本間控制提案差207.75µs，未標示全面通過。')]:
 a=s.index(' '+key+':',s.index('const WORKBENCH_RELEASE='));b=s.index('\n',a);s=s[:a]+' '+key+':'+json.dumps(val,ensure_ascii=False)+','+s[b:]
s=s.replace(' pending:[',' pending:[\n "幾何候選只離線：核心尖角已截除，整段其他取樣變化與K既有時序跳幅仍待確認；不部署。",',1)
s=s.replace('<section class="card" id="recentAcceptance"><h2>最近驗收</h2>','<section class="card" id="recentAcceptance"><h2>最近驗收</h2><p><b>邊界分段與拼接核對 · 僅離線</b> · <a href="#models">06查看第438～442格</a> · <a href="/experiment/k-geometry-guard/report.json">完整比較與限制</a></p>',1)
s+='''
const geometryGuardModelsBase=modelsPage;
modelsPage=async()=>{await geometryGuardModelsBase();if(route!=='models'||$('geometryGuardReport'))return;const r=await api('/experiment/k-geometry-guard/report.json');if(route!=='models')return;
$('main').insertAdjacentHTML('afterbegin',`<section class="card" id="geometryGuardReport"><h2>最新輪第438～442格 · 邊界分段與拼接核對（離線，未部署）</h2><p>左：前版分支時序候選；右：新增幾何核對。同一輪0d77c3ca、同影格。綠色為邊界，藍色為中心。只截除錯誤接點及不可信遠段，不用平滑補造道路；截短結果不作新增路口證據。</p><p>第438格保留；439～442格錯誤接點截除，五格PID提案不變。第二入口仍460格接管、553／557接回；原兩路口切換時點不變。23.4／97.3／142秒無新增誤切，未新增空路徑或失效取樣。</p><p><b>限制：</b>整段其他區段有95格PID提案不同，最大版本間差207.75µs（不是單步跳幅）；尚不能宣稱整段控制已改善。K原有時序跳幅未修。本輪不部署、不動車、400ms不變。</p><select id="geometryGuardFrame">${r.cards.map(c=>`<option value="${c.seq}">${c.seconds}秒 · 第${c.seq}格</option>`).join('')}</select><p id="geometryGuardReason"></p><img id="geometryGuardImage" style="max-width:100%"><p><a href="/experiment/k-geometry-guard/report.json">完整比較紀錄</a></p></section>`);
const draw=()=>{const c=r.cards.find(c=>c.seq==$('geometryGuardFrame').value);$('geometryGuardImage').src='/experiment/k-geometry-guard/'+c.image;$('geometryGuardReason').textContent=c.guard?'截除不可信遠段，保留 y≥'+c.guard.cutoff_y+' 的連續部分（640×360座標）':'此格不需截除，保留前版結果';};$('geometryGuardFrame').onchange=draw;$('geometryGuardFrame').value='440';draw();};
'''
f.write_text(s,encoding='utf8')
f=root/'laptop_workbench/README.md';f.write_text(f.read_text(encoding='utf8')+'\n\n## .57 邊界分段與拼接核對（僅離線）\n候選tools/junction_geometry_guard.py及compare_geometry_guard.py；資料data/experiments/k-geometry-guard。前版baseline保留。第439～442格錯誤拼接截除，438格不動。最新輪控制提案完全一致；原長片模式一致、無新增空路徑/失效PID，兩路口與負例切換不變。但整段95格提案改變最大207.75µs，非單步跳幅，需後續核對；K時序跳幅未處理。未變更web算法、未部署、未動車、400ms不變。\n',encoding='utf8')
print(json.dumps(r['focus']))

