import sys,json,copy,shutil,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'laptop_workbench/runtimes/opencv-4.11'),str(ROOT/'web'),str(ROOT/'web/k_lane_runtime')]
import cv2,numpy as np
from k_adaptive_pid import KAdaptivePID
from junction_k import center
from junction_branch_candidate import guard_single
OUT=ROOT/'laptop_workbench/data/experiments/k-branch-temporal'
run=ROOT.parent/'firmware_builds/i2c_retry_v3/run-0d77c3ca/0d77c3ca05284af9b8e5ad4d0ef81d5c/pid.jsonl';logs=[json.loads(x) for x in run.read_text().splitlines()];params=logs[0]['parameters'];assert params['max_gap']==.4
results={n:json.loads((OUT/(n+'.json')).read_text()) for n in ['latest','fullclip']};samples={x['vision']['seq']:x for x in logs if x.get('kind')=='sample'}
checks={};handoffs=[]
for name,rows in results.items():
 pids=[KAdaptivePID(params),KAdaptivePID(params)];last={};steps={'baseline':[],'candidate':[]}
 for row in rows:
  for key,pid in zip(['baseline','candidate'],pids):
   r=row[key];r['pid']=pid.step_path(row['t'],r['path'] if r['valid'] else [])
   if key in last:
    prev=last[key];dx=[abs(center((np.array(r['path'])/2).tolist(),y)-center((np.array(prev['path'])/2).tolist(),y))*2 for y in [210,260,300] if r['path'] and prev['path'] and center((np.array(r['path'])/2).tolist(),y) is not None and center((np.array(prev['path'])/2).tolist(),y) is not None]
    if name=='latest' and 450<=row['seq']<=490 and dx:steps[key].append(max(dx))
    if r['mode']=='KL_CROSS' and prev['mode']!='KL_CROSS':
     g=r['k']['mask_gate'];assert g['reference_compatible'] and g['entry_check_passed']
     handoffs.append(dict(dataset=name,variant=key,seq=row['seq'],path_step_px=max(dx) if dx else None,steer_step_us=abs(r['pid']['steer_us']-prev['pid']['steer_us']),branch_votes=g.get('branch_votes'),branch_progress_px=g.get('branch_progress_px')))
   last[key]=r
 checks[name]={'frames':len(rows),'single_guard_frames':sum(bool(x['guard']) for x in rows),'second_entry_path_steps':{k:dict(max=max(v),p95=float(np.percentile(v,95))) if v else None for k,v in steps.items()}}
 (OUT/(name+'.json')).write_text(json.dumps(rows),encoding='utf8')
# Exact preservation of requested negative windows and existing intersection 2 paths.
full=results['fullclip']
for n,lo,hi in [('curve23',210,260),('adjacent97',940,1000),('curve142',1390,1450)]:
 rs=[x for x in full if lo<=x['seq']<=hi];assert all(x['baseline']['path']==x['candidate']['path'] and x['baseline']['mode']==x['candidate']['mode'] for x in rs);checks[n]={'exact_path_and_mode_match':True,'frames':len(rs)}
# Narrow guard behavior, with valid geometry unchanged.
fake={'path':[[1,100],[2,200],[3,300]],'motion':{'valid':False},'road_segments':[{'source':'single','points':[[2,200],[3,300]]}]};gate=type('G',(),{'state':'SUSPECT'})();assert guard_single(fake,gate)['path']==[[2,200],[3,300]]
clear=copy.deepcopy(fake);clear['motion']['valid']=True;assert guard_single(clear,gate)==clear
checks['single_guard_contract']='unsupported extension removed only with unresolved crossing and unavailable alignment'
# Render selected frames as standalone comparison and inspect them before publishing.
images=[];cards=[]
for seq in [440,460,470,480,500,520]:
 row=next(x for x in results['latest'] if x['seq']==seq);im=cv2.imdecode(np.fromfile(str(OUT/f'latest-{seq}.jpg'),np.uint8),1);panels=[]
 for label,key in [('BASELINE','baseline'),('CANDIDATE','candidate')]:
  vis=im.copy();r=row[key]
  if r['path']:cv2.polylines(vis,[np.int32(np.array(r['path'])/2)],False,(255,130,0),2)
  cv2.putText(vis,f'{label} seq{seq} {r["mode"]}',(8,22),0,.6,(0,255,255),2);panels.append(vis)
 pair=np.hstack(panels);cv2.imencode('.jpg',pair)[1].tofile(str(OUT/f'compare-{seq}.jpg'));images.append(pair)
 cards.append({'seq':seq,'seconds':round(samples[seq]['wall']-logs[0]['started'],2),'baseline':row['baseline']['mode'],'candidate':row['candidate']['mode'],'reason':row['candidate']['reason'],'image':f'compare-{seq}.jpg'})
cv2.imencode('.jpg',np.vstack(images))[1].tofile(str(OUT/'comparison.jpg'))
report={'status':'offline_only_not_deployed','version':'k-branch-temporal-r2','summary':{n:json.loads((OUT/(n+'-summary.json')).read_text()) for n in results},'checks':checks,'handoffs':handoffs,'cards':cards,'max_gap':.4,'opencv':cv2.__version__,'limitations':['最新輪以錄影最近影格與實際控制遮罩配對，非逐像素相同控制影像；包含587筆控制樣本及1筆停止診斷。','此為凍結幾何逐格重播、候選切換及輸出保護；沒有重訓或改動車端、PID設定。','改變轉向後車輛與相機位置將不同，離線切換成功不等於實車已走對出口。','單線保護只保留當前單線支持範圍，不以未對齊歷史補出中心；不是道路真值。']}
(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({'checks':checks,'handoffs':handoffs},ensure_ascii=False))

