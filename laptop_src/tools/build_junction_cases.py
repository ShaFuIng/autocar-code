"""Prepare two reproducible offline junction cases; never access the car."""
import copy, datetime, json, os, sys, time, uuid
from pathlib import Path
import cv2
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'laptop_workbench'))
from analysis_jobs import read, write, alive
from shadow_worker import run, sha

def main():
    data=ROOT/'laptop_workbench/data'; jobs=data/'analysis'
    out=data/'experiments/junction-cases'; out.mkdir(exist_ok=True)
    first='3c92628ecb2446f482e70e3e79c10c49'
    original=read(jobs/first/'result.json')
    source=ROOT.parent/'完整跑道錄影_20261016_155847_3e961ecb'
    video=source/'video_003.avi'; records=read_lines(source/'frames_003.jsonl')
    cap=cv2.VideoCapture(str(video)); fps=cap.get(cv2.CAP_PROP_FPS)
    assert fps==10 and int(cap.get(cv2.CAP_PROP_FRAME_COUNT))==len(records)
    report=read(data/'experiments/junction-straight/report.json')
    assert sha(video)==report['source_sha256']
    for name, digest in report['implementation_sha256'].items():
        assert sha(ROOT/'laptop_workbench'/name)==digest, '基準算法已變更：'+name
    saved=out/'second-job.json'
    if saved.exists():
        uid=read(saved)['job']; result=read(jobs/uid/'result.json')
        assert read(jobs/uid/'status.json')['state']=='completed'
    else:
        for p in jobs.glob('*/job.json'):
            j=read(p); s=read(p.with_name('status.json')) if p.with_name('status.json').exists() else j
            if s.get('state') in ('running','queued') and alive(j.get('pid')):raise RuntimeError('已有分析執行中')
        clip=ROOT/'shadow_workbench/runs/junction-player102-120-continuous'; clip.mkdir(exist_ok=True)
        origin=datetime.datetime.fromisoformat(records[0]['time']).timestamp()
        inputs=[]; rows=[]
        for i in range(1201):
            ok,im=cap.read()
            if not ok:raise RuntimeError('影片解碼失敗')
            if i<1020:continue
            k=len(inputs); p=clip/f'{k:06d}.jpg'
            cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,95])[1].tofile(str(p))
            t=datetime.datetime.fromisoformat(records[i]['time']).timestamp()-origin
            inputs.append(dict(image=str(p),frame=k,capture_s=t))
            rows.append(dict(frame=k,capture_s=t,source_frame=i,player_s=i/fps))
        write(clip/'frames.json',rows)
        uid=uuid.uuid4().hex; folder=jobs/uid; folder.mkdir()
        spec=copy.deepcopy(original['spec']); spec.update(start=0,end=len(rows)-1,stride=1)
        assert all(not g['pid_enabled'] for g in spec['groups'])
        job=dict(id=uid,kind='shadow',source='shadow-'+clip.name,title='十字路口第二次｜播放器1:42～2:00｜v3.1／直行選路v1｜PID關閉',models=['v4-640'],spec=spec,created=time.time(),state='running',pid=os.getpid(),total=len(rows)*3,execution='筆電離線；共用V4；無車控輸出')
        write(folder/'job.json',job); write(folder/'input.json',inputs)
        print('ANALYSIS_JOB',uid,flush=True)
        try:
            result=run(folder)
            for f,r in zip(result['frames'],rows):f.update(player_s=r['player_s'],source_video_frame=r['source_frame'])
            write(folder/'result.json',result);write(folder/'exit.json',dict(returncode=0))
            write(saved,dict(job=uid))
        except Exception as e:
            write(folder/'status.json',dict(state='failed',error=str(e)));raise
    cap.release()
    assert original['recognition']['v4-640']['weight_sha256']==result['recognition']['v4-640']['weight_sha256']
    cases=[]
    for key,title,job,r,start,end in [('first','第一次通過',first,original,65,82),('second','第二次通過',uid,result,102,120)]:
        reasons={}
        for row in r['models']['B']:
            reason=row['A']['junction']['reason'];reasons[reason]=reasons.get(reason,0)+1
        cases.append(dict(id=key,title=title,job=job,player_range_s=[start,end],frames=len(r['frames']),source_frame_range=[int(start*fps),int(end*fps)],capture_range_s=[r['frames'][0]['capture_s'],r['frames'][-1]['capture_s']],reasons=reasons,summaries=r['summaries'],intent='straight',annotation_status='尚未人工標註'))
    write(out/'cases.json',dict(version=1,cases=cases,source_video=str(video),source_sha256=report['source_sha256'],timestamps_sha256=sha(source/'frames_003.jsonl'),fps=fps,recognition=result['recognition'],algorithms=report['implementation_sha256'],pid_enabled=False,hardware_output=False,second_cache=result['cache']))
    print(json.dumps(cases,ensure_ascii=False),flush=True)

def read_lines(p):return [json.loads(x) for x in p.read_text(encoding='utf8').splitlines()]
if __name__=='__main__':main()
