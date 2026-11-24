"""Local source clip and offline comparison only; no remote/device access."""
from pathlib import Path
import sys,os,json,datetime,time,uuid,copy
ROOT=Path(__file__).resolve().parents[1];W=ROOT/'laptop_workbench'
sys.path[:0]=[str(W/'runtimes/opencv-4.11'),str(W)]
import cv2,numpy as np
from analysis_jobs import read,write,alive
from shadow_worker import run,sha
for f in (W/'data/analysis').glob('*/job.json'):
 j=read(f);s=read(f.with_name('status.json')) if f.with_name('status.json').exists() else j
 if s.get('state') in ('queued','running') and alive(j.get('pid')):raise RuntimeError('Existing offline analysis running')
source=ROOT.parent/'完整跑道錄影_20261016_155847_3e961ecb';clip=ROOT/'shadow_workbench/runs/k-video003-000-256'
if clip.exists():raise RuntimeError('Preserve existing clip; output already exists')
clip.mkdir(parents=True)
records=[json.loads(x) for x in (source/'frames_003.jsonl').read_text(encoding='utf-8').splitlines()]
origin=datetime.datetime.fromisoformat(records[0]['time']).timestamp()
cap=cv2.VideoCapture(str(source/'video_003.avi'));fps=cap.get(cv2.CAP_PROP_FPS);assert fps==10 and int(cap.get(cv2.CAP_PROP_FRAME_COUNT))==len(records)
writer=cv2.VideoWriter(str(clip/'video_003_000-256.avi'),cv2.VideoWriter_fourcc(*'MJPG'),fps,(1280,720));assert writer.isOpened()
rows=[];inputs=[]
for i in range(int(176*fps)):
 ok,im=cap.read()
 if not ok:raise RuntimeError('Missing video frame')
 t=datetime.datetime.fromisoformat(records[i]['time']).timestamp()-origin
 path=clip/f'{i:06d}.jpg';cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,95])[1].tofile(str(path));writer.write(im)
 rows.append(dict(frame=i,capture_s=t,source_frame=i,player_s=i/fps));inputs.append(dict(image=str(path),capture_s=t,frame=i))
cap.release();writer.release();write(clip/'frames.json',rows)
report=W/'data/experiments/k-live-prep';report.mkdir(exist_ok=True,parents=True)
write(clip/'provenance.json',dict(source=str(source/'video_003.avi'),source_sha256=sha(source/'video_003.avi'),timestamp_sha256=sha(source/'frames_003.jsonl'),player_range_s=[0,176],range_end_exclusive=True,frames=len(rows),fps=fps,clip_sha256=sha(clip/'video_003_000-256.avi'),encoding='derived MJPEG AVI; analysis JPEG quality95; source unchanged',ground_truth=False))
uid=uuid.uuid4().hex;folder=W/'data/analysis'/uid;folder.mkdir()
spec=copy.deepcopy(read(W/'data/analysis/e57c06186694457ea00c57edfe1d03fa/job.json')['spec']);base=spec['groups'][1];spec.update(start=0,end=len(rows)-1,stride=1,groups=[])
for letter,alg,label in [('A','road-memory-v3.1','原一般道路 A · v3.1'),('B','junction-k-v1','新版 K · 接回穩定化')]:
 g=copy.deepcopy(base);g.update(id=letter,label=label,algorithm=alg,pid_enabled=False);spec['groups'].append(g)
job=dict(id=uid,kind='shadow',source='shadow-'+clip.name,title='新測試片段｜video_003 0:00～2:56｜A／新版K｜PID關閉',models=['v4-640'],spec=spec,created=time.time(),state='running',pid=os.getpid(),total=len(rows)*3,execution='筆電離線；OpenCV4.11；無車控輸出')
write(folder/'job.json',job);write(folder/'input.json',inputs)
write(report/'clip-job.json',dict(job=uid,source=job['source'],clip=str(clip),frames=len(rows),hardware_output=False))
print('CLIP_READY',uid,len(rows),flush=True)
try:
 result=run(folder);write(folder/'exit.json',dict(returncode=0));print('ANALYSIS_COMPLETE',uid,result['cache'],flush=True)
except Exception:
 import traceback
 write(folder/'status.json',dict(state='failed',error=traceback.format_exc()));write(folder/'exit.json',dict(returncode=1));raise
