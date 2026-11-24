"""Offline full-track bend comparisons; existing weights and videos unchanged."""
import json,sys,hashlib,os
from pathlib import Path
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent.parent
sys.path.insert(0,str(PROJECT/'yolo26_sem_v5_focus70/.deps'))
sys.path.insert(0,str(ROOT.parent/'web'))
os.environ['YOLO_CONFIG_DIR']=str(ROOT/'data/experiments/night-20261018/ultralytics_config')
import cv2,numpy as np,torch
from live_model_worker import V4,corridor
from curve_replay import BoundaryTracker,GEOMETRY,ground,pixel
from temporal_lane import TemporalLane
from boundary_follow import offset_boundary,image_curve,PrioritySelector,anchored_identity

def main():
    cv2.setNumThreads(2);torch.set_num_threads(4)
    source=PROJECT/'完整跑道錄影_20261016_155847_3e961ecb'
    oldroot=ROOT/'data/experiments/night-20261018'
    out=ROOT/'data/experiments/bend-cases';out.mkdir(exist_ok=True)
    rows=json.loads((oldroot/'v4-640-fulltrack-frames.json').read_text(encoding='utf8'))
    # Historical footage predates the Oct19 camera tilt. Never apply current pose.
    geometry=dict(GEOMETRY,pitch_deg=0.,status='HISTORICAL_CAMERA_SCENARIO_NOT_CALIBRATED')
    cases=[];model=None
    for cid,label,start,end in [('large','起始大彎',0,43),('small','第二個小彎',56,85)]:
        folder=out/cid;folder.mkdir(exist_ok=True)
        selected=[r for r in rows if start<=r['capture_s']<=end]
        tracker=TemporalLane();old=BoundaryTracker();selector=PrioritySelector();results=[]
        cap=cv2.VideoCapture(str(source/'video_003.avi'))
        cap.set(cv2.CAP_PROP_POS_FRAMES,selected[0]['source_frame'])
        for i,r in enumerate(selected):
            ok,im=cap.read();assert ok
            maskpath=folder/f'{i:06d}-cache.png'
            if maskpath.exists():mask=cv2.imdecode(np.fromfile(str(maskpath),np.uint8),0)>0
            else:
                if model is None:model=V4(PROJECT/'yolo26_sem_v4_trial/runs/trial01/weights/best.pt')
                mask=model.predict(im);cv2.imencode('.png',mask.astype(np.uint8)*255)[1].tofile(str(maskpath))
            n=tracker.step(im,mask,r['capture_s']);v1=old.step(mask,r['capture_s'])
            legacy,q=corridor(mask,.6)
            anchored={s:anchored_identity(n['boundaries'][s],s) for s in ('left','right')}
            left=offset_boundary(n['boundaries']['left'],'left',geometry) if anchored['left'] else []
            right=offset_boundary(n['boundaries']['right'],'right',geometry) if anchored['right'] else []
            double=[p for x,y in n['image_path'] if (p:=ground(x,y,640,360,geometry))] if all(anchored.values()) else []
            chosen=selector.update(dict(double=double,left=left,right=right))
            paths=dict(legacy=[[x/2,y/2] for x,y in legacy],
                v1=[p for x,z in v1['path'] if (p:=pixel(x,z,640,360))],
                v2=n['image_path'],left=image_curve(left,geometry),right=image_curve(right,geometry),
                priority=image_curve(chosen['path'],geometry),
                A=[[x/2,y/2] for x,y in r['A'].get('path',[])],
                B=[[x/2,y/2] for x,y in r['B'].get('path',[])],
                C=[[x/2,y/2] for x,y in r['C'].get('path',[])])
            cv2.imencode('.jpg',cv2.resize(im,(640,360)),[cv2.IMWRITE_JPEG_QUALITY,85])[1].tofile(str(folder/f'{i:06d}-raw.jpg'))
            small=cv2.resize(mask.astype(np.uint8),(640,360),interpolation=cv2.INTER_NEAREST)
            rgba=np.zeros((360,640,4),np.uint8);rgba[:,:,3]=small*255
            cv2.imencode('.png',rgba)[1].tofile(str(folder/f'{i:06d}-mask.png'))
            results.append(dict(frame=i,source_frame=r['source_frame'],t=r['capture_s'],paths=paths,
                mode=chosen['mode'],pending=chosen['pending'],side_anchors=anchored,boundaries=n['boundaries'],
                legacy_quality=q,motion=n['motion'],hardware_output=False))
            if i%50==0:print(cid,i,len(selected),flush=True)
        cap.release()
        report=dict(id=cid,label=label,start=results[0]['t'],end=results[-1]['t'],
            frames=results,geometry=geometry,source='video_003.avi',hardware_output=False,
            availability={k:sum(bool(f['paths'][k]) for f in results) for k in results[0]['paths']})
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,allow_nan=False),encoding='utf8')
        cases.append({k:v for k,v in report.items() if k!='frames'}|dict(frame_count=len(results)))
    manifest=dict(cases=cases,model='V4-640',hardware_output=False,geometry=geometry,
        video_sha256=hashlib.sha256((source/'video_003.avi').read_bytes()).hexdigest(),
        weights_sha256=hashlib.sha256((PROJECT/'yolo26_sem_v4_trial/runs/trial01/weights/best.pt').read_bytes()).hexdigest(),
        limitations=['原片含訓練來源場景，只做回歸比較，不是獨立準確率測試。',
        '拍攝早於10/19相機調整；單線偏移使用舊相機0度情境，並非已標定姿態，25.25cm僅為幾何假設。',
        'v1保留先前4.7度投影情境作失敗對照；歷史A/B/C保留原夜間實驗結果，不代表相同幾何。',
        '單線身分增加近處／畫面側緣錨點檢查；只有遠處曲線時不靠畫面左右猜身分。仍未完成完整身分追蹤與統一地面優化。',
        '優先策略僅3影格確認，尚未加入平順交接與可信里程計；不沿用未對齊的舊位置。',
        '沒有車輛閉環模擬或硬體輸出；綠線／路徑可用不代表能實車過彎。'])
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf8')
if __name__=='__main__':main()
