"""Local full-frame 55–75s comparison. No remote or vehicle imports."""
import copy,datetime,json,os,sys,time,uuid
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'laptop_workbench'))
from analysis_jobs import read,write,alive
from shadow_worker import run,sha

def main():
    data=ROOT/'laptop_workbench/data';jobs=data/'analysis'
    for p in jobs.glob('*/job.json'):
        j=read(p);s=read(p.with_name('status.json')) if p.with_name('status.json').exists() else j
        if s.get('state') in ('running','queued') and alive(j.get('pid')):raise RuntimeError('已有分析執行中')
    source=ROOT.parent/'完整跑道錄影_20261016_155847_3e961ecb'
    clip=ROOT/'shadow_workbench/runs/fulltrack-55-75-continuous';clip.mkdir(exist_ok=True)
    records=[json.loads(x) for x in (source/'frames_003.jsonl').read_text().splitlines()]
    origin=datetime.datetime.fromisoformat(records[0]['time']).timestamp()
    cap=cv2.VideoCapture(str(source/'video_003.avi'))
    if int(cap.get(cv2.CAP_PROP_FRAME_COUNT))!=len(records):raise RuntimeError('影格與時間戳數量不符')
    inputs=[];rows=[]
    for i,record in enumerate(records):
        t=datetime.datetime.fromisoformat(record['time']).timestamp()-origin
        ok,im=cap.read()
        if not ok:raise RuntimeError('影片解碼失敗')
        if t>75:break
        if t<55:continue
        k=len(inputs);path=clip/f'{k:06d}.jpg'
        cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,95])[1].tofile(str(path))
        inputs.append(dict(image=str(path),capture_s=t,frame=k))
        rows.append(dict(frame=k,capture_s=t,source_frame=i))
    cap.release();write(clip/'frames.json',rows)
    gaps=np.diff([x['capture_s'] for x in rows]);print('FRAMES',len(rows),'RANGE',rows[0]['capture_s'],rows[-1]['capture_s'],'GAP',float(np.median(gaps)),float(max(gaps)),flush=True)
    uid=uuid.uuid4().hex;folder=jobs/uid;folder.mkdir()
    spec=copy.deepcopy(read(jobs/'e57c06186694457ea00c57edfe1d03fa/job.json')['spec'])
    base=spec['groups'][1];spec.update(start=0,end=len(rows)-1,stride=1,groups=[])
    for letter,alg in [('A','road-memory-v3'),('B','road-memory-v3.1')]:
        g=copy.deepcopy(base);g.update(id=letter,label=alg,algorithm=alg,pid_enabled=False);spec['groups'].append(g)
    job=dict(id=uid,kind='shadow',source='shadow-'+clip.name,title='完整原片55～75秒連續影格｜歷史v3／v3.1修補比較｜PID關閉',models=['v4-640'],spec=spec,created=time.time(),state='running',pid=os.getpid(),total=len(rows)*3,execution='筆電離線；共用V4；無車控輸出')
    write(folder/'job.json',job);write(folder/'input.json',inputs)
    report=data/'experiments/road-v31';report.mkdir(exist_ok=True)
    provenance=dict(job=uid,frames=len(rows),start_s=rows[0]['capture_s'],end_s=rows[-1]['capture_s'],median_gap_s=float(np.median(gaps)),max_gap_s=float(max(gaps)),source_video=str(source/'video_003.avi'),source_sha256=sha(source/'video_003.avi'),timestamps_sha256=sha(source/'frames_003.jsonl'),source_frame_range=[rows[0]['source_frame'],rows[-1]['source_frame']],pid_enabled=False,hardware_output=False)
    write(report/'report.json',provenance)
    result=run(folder)
    metrics={}
    for k,values in result['models'].items():
        rr=[x['A'] for x in values]
        metrics[k]=dict(path_valid=sum(x['valid'] for x in rr),history_frames=sum(x.get('history_points',0)>0 for x in rr),alignment_valid=sum(x.get('motion',{}).get('valid',False) for x in rr),reacquire_frames=sum(bool(x.get('candidates')) for x in rr),mean_span_px=float(np.mean([x['coverage']['visible_span_px'] for x in rr])))
    provenance.update(metrics=metrics,cache=result['cache'],new_tests_passed=3)
    write(report/'report.json',provenance);write(folder/'exit.json',dict(returncode=0));print(json.dumps(provenance,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
