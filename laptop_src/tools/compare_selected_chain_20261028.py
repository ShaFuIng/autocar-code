"""10/24 recordings, file-only comparison. No network or hardware calls."""
import sys,os,json,time,hashlib,copy,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];WB=ROOT/'laptop_workbench';PREV=WB/'data/experiments/new-junctions-20261028';OUT=WB/'data/experiments/selected-chain-20261028';OUT.mkdir(exist_ok=True)
sys.path[:0]=[str(WB/'runtimes/opencv-4.11'),str(ROOT/'web'),str(ROOT/'web/k_lane_runtime'),str(ROOT.parent/'yolo26_sem_v5_focus70/.deps')]
os.environ['YOLO_CONFIG_DIR']=str(OUT/'yolo-config')
import cv2,numpy as np

from junction_k_lane import KLanePath,lane_evidence
from junction_approach_candidate import ApproachGate
from junction_segment_candidate import SegmentCandidate
from k_adaptive_pid import KAdaptivePID
cv2.setNumThreads(2)
params=json.loads((WB/'data/experiments/visual-center-18/parameters-after.json').read_text(encoding='utf8'));assert params['rev']==79
p=params['data']['parameters'];assert p['visual_center_offset_px_640']==18 and p['max_gap']==.4
(OUT/'parameters.json').write_text(json.dumps(params,ensure_ascii=False,indent=2),encoding='utf8')
weight=ROOT.parent/'yolo26_sem_v4_trial/runs/trial01/weights/best.pt'
archive=json.loads((WB/'data/experiments/field-recordings-20261024/acceptance.json').read_text(encoding='utf8'))
report=dict(status='running',parameter_revision=79,visual_center_offset_px_640=18,max_gap=.4,weights_sha256=hashlib.sha256(weight.read_bytes()).hexdigest(),baseline='k-branch-geometry-live-r1',candidate='selected-chain-exit-extent-r1',opencv=cv2.__version__,clips=[])
for name,prefix,title in [('entry2','1274e47f','新第二入口'),('entry1','4edf0adc','新第一入口'),('slope','60aee9fc','斜坡與相鄰跑道')]:
 folder=OUT/name;folder.mkdir(exist_ok=True);(folder/'mask').mkdir(exist_ok=True)
 item=next(x for x in archive['items'] if x['id'].startswith(prefix));src=Path(item['folder']);stamps=json.loads((src/'preview/frames.json').read_text());base=KLanePath();base.gate=ApproachGate();cur=SegmentCandidate();shared=KLanePath();pid=KAdaptivePID(p);rows=[];caps={};old=json.loads((PREV/name/'rows.json').read_text())
 for i,stamp in enumerate(stamps):
  video=stamp['source']
  if video not in caps:caps[video]=cv2.VideoCapture(str(src/video))
  ok,im=caps[video].read();assert ok,(name,i);im=cv2.resize(im,(640,360));t=stamp['capture_s']
  maskfile=PREV/name/'mask'/f'{i:06d}.png'
  mask=cv2.imdecode(np.fromfile(str(maskfile),np.uint8),0)>0
  a=shared.base.step(im,mask,t);v=shared.visible.step(im);d=lane_evidence(mask,a,v);d['capture_s']=t
  result=cur.combine(copy.deepcopy(a),v,mask,copy.deepcopy(d))
  row=dict(index=i,t=t,source_frame=stamp['source_frame'],source=video,legacy=old[i]['baseline'],baseline=old[i]['candidate']);panels=[]
  pr=pid.step_path(t,result['path'] if result['valid'] else [])
  row['candidate']={k:result[k] for k in ['path','valid','mode','reason','k']}
  row['candidate'].update(pid=pr,guard=result.get('geometry_guard'),motion=a.get('motion'))
  for key in ['legacy','baseline','candidate']:
   r=row[key];vis=im.copy();vis[mask]=(vis[mask]*.65+np.array([220,40,220])*.35).astype(np.uint8)
   if r['path']:cv2.polylines(vis,[np.int32(np.array(r['path'])/2)],False,(255,130,0),2)
   cv2.putText(vis,f'{key} {i} {t:.2f}s {r["mode"]}',(8,20),0,.5,(0,255,255),1);panels.append(vis)
  cv2.imencode('.jpg',np.hstack(panels),[cv2.IMWRITE_JPEG_QUALITY,85])[1].tofile(str(folder/f'{i:06d}.jpg'));rows.append(row)
  if i%100==0:print(name,i,'/',len(stamps),flush=True)
 for cap in caps.values():cap.release()
 (folder/'rows.json').write_text(json.dumps(rows),encoding='utf8')
 summary=dict(name=name,title=title,id=item['id'],frames=len(rows),duration=rows[-1]['t'],source_folder=str(src),groups={})
 for key in ['baseline','candidate']:
  changes=[];last=None
  for r in rows:
   if r[key]['mode']!=last:changes.append(dict(index=r['index'],t=r['t'],mode=r[key]['mode'],reason=r[key]['reason']));last=r[key]['mode']
  summary['groups'][key]=dict(transitions=changes,empty=sum(not r[key]['valid'] for r in rows),pid_invalid=sum(not r[key]['pid']['valid'] for r in rows),guard_frames=sum(bool(r[key]['guard']) for r in rows))
 report['clips'].append(summary);(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(summary,ensure_ascii=False),flush=True)
report['status']='complete';(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
