"""Export same-frame selectable overlays without changing any analysis result."""
import sys,json
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'web'))
from live_model_worker import corridor
from curve_replay import pixel
def main():
    root=ROOT/'data/experiments';out=root/'curve-layers';out.mkdir(exist_ok=True)
    for run in json.loads((root/'curve-v2/manifest.json').read_text(encoding='utf8'))['runs']:
        rid=run['id'];dest=out/rid;dest.mkdir(exist_ok=True)
        old=json.loads((root/'curve-v1'/rid/'result.json').read_text(encoding='utf8'))
        new=json.loads((root/'curve-v2'/rid/'result.json').read_text(encoding='utf8'))
        source=ROOT/'data/orin/records/manual'/rid/run['source_version'];rows=[];i=0
        for video in sorted(source.glob('video_*.avi')):
            cap=cv2.VideoCapture(str(video))
            while True:
                ok,im=cap.read()
                if not ok:break
                mask=cv2.imdecode(np.fromfile(str(root/'curve-v1'/rid/f'{i:06d}-mask.png'),np.uint8),0)>0
                legacy,q=corridor(mask,old['parameters']['row_ratio'])
                cv2.imencode('.jpg',cv2.resize(im,(640,360)))[1].tofile(str(dest/f'{i:06d}-raw.jpg'))
                small=cv2.resize(mask.astype(np.uint8),(640,360),interpolation=cv2.INTER_NEAREST)
                rgba=np.zeros((360,640,4),np.uint8);rgba[:,:,3]=small*255
                cv2.imencode('.png',rgba)[1].tofile(str(dest/f'{i:06d}-mask.png'))
                f=new['frames'][i];n=f['new']
                rows.append(dict(frame=i,t=f['t'],legacy=[[x/2,y/2] for x,y in legacy],
                    legacy_quality=q,v1=[p for x,z in old['frames'][i]['new']['path'] if (p:=pixel(x,z,640,360))],
                    v2=n['image_path'],boundaries=n['boundaries'],
                    memory={s:[[x,y] for x,y,t in points if f['t']-t>1e-6] for s,points in n['memory'].items()},
                    diagnostic=dict(valid=n['valid'],reason=n['reason'],left_points=len(n['boundaries']['left']),
                        right_points=len(n['boundaries']['right']),paired_rows=len(n['rows']),motion=n['motion'])))
                i+=1
            cap.release()
        assert i==new['frame_count']
        (dest/'layers.json').write_text(json.dumps(dict(run=rid,source_version=run['source_version'],
            model='V4',frames=rows,legacy_note='同一快取遮罩重算原9列算法，不是原即時JPEG輸出'),ensure_ascii=False),encoding='utf8')
        print(rid,rows[-1]['diagnostic'],flush=True)
if __name__=='__main__':main()
