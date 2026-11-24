"""Explicit local offline batch; cached masks permit repeatable geometry experiments."""
import sys,json,hashlib,os
from pathlib import Path
from datetime import datetime
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent.parent
sys.path.insert(0,str(PROJECT/'yolo26_sem_v5_focus70/.deps'))
sys.path.insert(0,str(ROOT.parent/'web'))
import cv2,numpy as np,torch
from live_model_worker import V4,corridor
from curve_replay import BoundaryTracker,GEOMETRY,pursuit,pixel

RUNS=['3443492eb7e14fe2a3f466d725b52783','45e345daa12245a58abb073046b7efa6',
      '4f7d497d310042029552587ad902d4e4']
def main():
    out=ROOT/'data/experiments/curve-v1';out.mkdir(parents=True,exist_ok=True)
    weights=PROJECT/'yolo26_sem_v4_trial/runs/trial01/weights/best.pt'
    torch.set_num_threads(4);cv2.setNumThreads(2);model=None;reports=[]
    for run in RUNS:
        base=ROOT/'data/orin/records/manual'/run
        current=json.loads((base/'current.json').read_text());folder=base/current['version']
        events=[json.loads(s) for s in (folder/'pid.jsonl').read_text().splitlines()]
        start=events[0];p=start['parameters'];tracker=BoundaryTracker()
        dest=out/run;dest.mkdir(exist_ok=True);rows=[];count=0
        for video in sorted(folder.glob('video_*.avi')):
            meta=[json.loads(s) for s in (folder/video.name.replace('video_','frames_').replace('.avi','.jsonl')).read_text().splitlines()]
            cap=cv2.VideoCapture(str(video));i=0
            while True:
                ok,im=cap.read()
                if not ok:break
                m=meta[i] if i<len(meta) else {}
                stamp=m.get('time') or m.get('timestamp')
                # Must use capture metadata, never nominal AVI FPS for wall duration.
                if not stamp:raise ValueError('Frame metadata lacks time: '+str(m)[:300])
                t=datetime.fromisoformat(stamp).timestamp()-start['started']
                maskfile=dest/f'{count:06d}-mask.png'
                if maskfile.exists():mask=cv2.imdecode(np.fromfile(str(maskfile),np.uint8),0)>0
                else:
                    if model is None:model=V4(weights)
                    mask=model.predict(im)
                    cv2.imencode('.png',mask.astype(np.uint8)*255)[1].tofile(str(maskfile))
                old,q=corridor(mask,p['row_ratio']);new=tracker.step(mask,t)
                vis=cv2.resize(im,(640,360));h,w=mask.shape
                small=cv2.resize(mask.astype(np.uint8),(640,360),interpolation=cv2.INTER_NEAREST)>0
                vis[small]=(vis[small]*.5+np.array([220,40,220])*.5).astype(np.uint8)
                for r in new['rows']:
                    y=int(r['y']*360/h)
                    for key,seen in [('left',r['mode']!='right_only'),('right',r['mode']!='left_only')]:
                        if not seen:
                            uv=pixel(*r['inferred_boundary'],640,360) if r.get('inferred_boundary') else None
                            if uv:cv2.circle(vis,tuple(int(v) for v in uv),2,(0,170,255),1)
                            continue
                        x=int(r[key]*640/w)
                        cv2.circle(vis,(x,y),2,(255,220,0),-1)
                for x,z in new['path']:
                    uv=pixel(x,z,640,360)
                    if uv:cv2.circle(vis,tuple(int(v) for v in uv),2,(0,255,0),-1)
                for x,y in old:cv2.circle(vis,(int(x/2),int(y/2)),2,(0,255,255),-1)
                cv2.imencode('.jpg',vis)[1].tofile(str(dest/f'{count:06d}.jpg'))
                actual=min((e for e in events if e.get('kind')=='sample'),key=lambda e:abs(e['wall']-start['started']-t),default={})
                delta=abs(actual.get('wall',-999)-start['started']-t)
                rows.append(dict(frame=count,t=t,old_valid=q>=.66,old_quality=q,
                    new=new,pp=pursuit(new['path']),actual_us=actual.get('proposal',{}).get('steer_us') if delta<.25 else None,
                    actual_match_error_s=delta if delta<.25 else None))
                i+=1;count+=1
            cap.release()
        report=dict(id=run,source_version=current['version'],source_manifest_sha256=hashlib.sha256((folder/'sync_manifest.json').read_bytes()).hexdigest(),
                    parameters=p,parameter_revision=start.get('parameter_revision'),geometry=GEOMETRY,
                    stop=events[-1],frames=rows,frame_count=len(rows),
                    old_valid=sum(r['old_valid'] for r in rows),new_valid=sum(r['new']['valid'] for r in rows))
        (dest/'result.json').write_text(json.dumps(report,ensure_ascii=False,allow_nan=False),encoding='utf-8')
        reports.append({k:v for k,v in report.items() if k!='frames'})
        print(run,count,report['old_valid'],report['new_valid'],flush=True)
    manifest=dict(version=1,hardware_output=False,weights_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(),
                  geometry=GEOMETRY,runs=reports,limitations=[
                  '來源影片重解碼，與原即時JPEG不完全相同；不能復原原停止瞬間的影像年齡。',
                  '有效路徑比例不是辨識準確率或實車通過率；無人工真值。',
                  '地面投影未標定；單側身分由雙側歷史短暫追蹤，再沿估計地面法向偏移25.25cm，急彎仍需驗證。',
                  '換胎前轉向映射僅供情境估算；沒有模擬車輛運動後的新畫面。'])
    tmp=out/'manifest.tmp';tmp.write_text(json.dumps(manifest,ensure_ascii=False,allow_nan=False),encoding='utf-8');tmp.replace(out/'manifest.json')
if __name__=='__main__':main()
