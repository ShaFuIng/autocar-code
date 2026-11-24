"""Build reviewable candidate from saved measurements; no hardware/network."""
import json,hashlib,shutil
from pathlib import Path
import cv2,numpy as np
from ground_calibration import calculate,project
ROOT=Path(__file__).resolve().parent
def main():
    inputs=json.loads((ROOT/'data/ground-calibration-input.json').read_text(encoding='utf-8-sig'))
    doc=inputs['camera-ground-measure'];v=inputs['camera-ground-validation-thin-stick']['data']
    report=calculate(doc['data']['lines'],require_confirmed=False)
    report['all_lines_confirmed']=all(l['confirmed'] for l in doc['data']['lines'])
    report['confirmation_note']='本版勾選未確認；僅作草稿比較，不代表已驗收' if not report['all_lines_confirmed'] else '三條線已勾選確認'
    folder=ROOT/'data/experiments/ground-calibration'/f"measurement-v{doc['rev']}"
    folder.mkdir(parents=True,exist_ok=True)
    H=np.asarray(report['matrix']);xy=np.asarray(v['estimated_pixel'],float)
    z=float(project([xy],H)[0,1])
    samples=project([[xy[0],xy[1]+dy] for dy in (-1,0,1)],H)
    report.update(source_revision=doc['rev'],source_snapshot=doc['data']['snapshot'],
        lane_width_cm=46,width_source='沿用使用者提供淨寬，尚未在此次三個截面重新量測',
        coordinate_system='x：跑道中心向右為正；z：前輪輪胎最前緣往前。不是鏡頭／後軸原點。',
        assumptions=['地面平坦；三條線為相同前向距離的左右內緣截面',
        '直線區域內緣淨寬46cm，三個截面的中點共線',
        '保存量測與驗證照片之間相機／車輛未移動（待現場確認）',
        '未校正鏡頭畸變；原始距離估計±3cm，沒有足夠資料給出統計信賴區間'],
        validation=dict(target_cm=100,pixel=xy.tolist(),pixel_uncertainty=1,
            predicted_cm=z,error_cm=z-100,pixel_only_range_cm=[float(samples[:,1].min()),float(samples[:,1].max())],
            used_in_fit=False,placement_confirmed=False),
        status='候選投影：驗證仍有偏差，未套用控制',metric_calibrated=False,hardware_output=False)
    # Camera pixels -> top view. No synthetic fills outside the sampled strip.
    T=np.array([[4,0,240],[0,-4,680],[0,0,1]],float)
    for name,source in [('measurement',ROOT/'data/experiments/camera-measure'/doc['data']['snapshot']['id']/'raw.jpg'),
                        ('validation',ROOT/'data/calibration-thin-stick.jpg')]:
        im=cv2.imdecode(np.fromfile(str(source),np.uint8),1)
        shutil.copyfile(source,folder/f'{name}.jpg')
        warp=cv2.warpPerspective(im,T@H,(480,680))
        support=np.zeros(im.shape[:2],np.uint8)
        cv2.fillConvexPoly(support,cv2.convexHull(np.asarray(report['image_points'],np.float32)).astype(np.int32),255)
        valid=cv2.warpPerspective(support,T@H,(480,680))
        warp[valid==0]=24
        for zz in (50,80,100,120):
            yy=int(680-4*zz);cv2.line(warp,(148,yy),(332,yy),(60,210,240),1)
            cv2.putText(warp,f'{zz} cm',(10,yy),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
        if name=='validation':
            p=project([xy],T@H)[0];cv2.circle(warp,tuple(np.round(p).astype(int)),5,(0,0,255),2)
        cv2.imencode('.png',warp)[1].tofile(str(folder/f'{name}-top.png'))
    report['image_sha256']=hashlib.sha256((folder/'validation.jpg').read_bytes()).hexdigest()
    (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    (folder.parent/'latest.json').write_text(json.dumps({'folder':folder.name}),encoding='utf8')
    print(json.dumps({k:report[k] for k in ('fit','held_out_rows','validation')},ensure_ascii=False))
if __name__=='__main__':main()
