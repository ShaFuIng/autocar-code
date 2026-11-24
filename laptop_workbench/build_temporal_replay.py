"""Re-run cached segmentation through temporal tracking; never accesses Orin."""
import json, hashlib
from pathlib import Path
import cv2
import numpy as np
from temporal_lane import TemporalLane
from curve_replay import pixel, pursuit

ROOT=Path(__file__).resolve().parent
def main():
    oldroot=ROOT/'data/experiments/curve-v1'
    out=ROOT/'data/experiments/curve-v2';out.mkdir(exist_ok=True)
    manifest=json.loads((oldroot/'manifest.json').read_text(encoding='utf-8'))
    reports=[]
    for run in manifest['runs']:
        rid=run['id'];old=json.loads((oldroot/rid/'result.json').read_text(encoding='utf-8'))
        source=ROOT/'data/orin/records/manual'/rid/old['source_version']
        dest=out/rid;dest.mkdir(exist_ok=True)
        tracker=TemporalLane();frames=[];index=0
        for video in sorted(source.glob('video_*.avi')):
            cap=cv2.VideoCapture(str(video))
            while True:
                ok,image=cap.read()
                if not ok:break
                f=old['frames'][index]
                mask=cv2.imdecode(np.fromfile(str(oldroot/rid/f'{index:06d}-mask.png'),np.uint8),0)>0
                new=tracker.step(image,mask,f['t'])
                vis=cv2.resize(image,(640,360))
                small=cv2.resize(mask.astype(np.uint8),(640,360),interpolation=cv2.INTER_NEAREST)>0
                vis[small]=(vis[small]*.5+np.array([220,40,220])*.5).astype(np.uint8)
                for side,points in new['memory'].items():
                    for x,y,observed in points:
                        if f['t']-observed>1e-6:
                            cv2.circle(vis,(round(x),round(y)),2,(0,170,255),1)
                for points in new['boundaries'].values():
                    for x,y in points:cv2.circle(vis,(round(x),round(y)),2,(255,220,0),-1)
                for x,z in f['new']['path']:
                    uv=pixel(x,z,640,360)
                    if uv:cv2.circle(vis,tuple(round(v) for v in uv),1,(0,255,255),-1)
                for x,y in new['image_path']:
                    cv2.circle(vis,(round(x),round(y)),2,(0,255,0),-1)
                cv2.imencode('.jpg',vis)[1].tofile(str(dest/f'{index:06d}.jpg'))
                frames.append(dict(f,new=new,pp=pursuit(new['path']),v1_valid=f['new']['valid']))
                index+=1
            cap.release()
        assert index==old['frame_count']
        report=dict(old,frames=frames,new_valid=sum(f['new']['valid'] for f in frames),
                    algorithm_version=2,motion_valid=sum(f['new']['motion']['valid'] for f in frames),
                    memory_frames=sum(f['new']['single_rows']>0 for f in frames),
                    baseline_result_sha256=hashlib.sha256((oldroot/rid/'result.json').read_bytes()).hexdigest())
        (dest/'result.json').write_text(json.dumps(report,ensure_ascii=False,allow_nan=False),encoding='utf-8')
        reports.append({k:v for k,v in report.items() if k!='frames'})
        print(rid,index,report['new_valid'],report['motion_valid'],report['memory_frames'],flush=True)
    manifest.update(version=2,runs=reports,limitations=[
        '紫色為同一批快取V4遮罩；青色為本影格骨架邊界，綠色為雙側配對中心；黃色為v1候選。',
        '橘色空心點為經光流對齊的歷史實際觀測，不是憑空補另一條線；最長0.65秒，對齊失敗立即捨棄。',
        '使用下半部特徵的LK光流與RANSAC平面單應性，尚非編碼器／IMU里程計或完整地圖。',
        '尚未標定地面與鏡頭畸變；綠線為影像中點，不等同精準公分尺度的地面中心。單側首次出現不推造中心。',
        '無人工真值；輸出比例不是準確率。沒有模擬新命令造成的車輛運動，未部署實車。'])
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,allow_nan=False),encoding='utf-8')
if __name__=='__main__':
    cv2.setNumThreads(2)
    main()
