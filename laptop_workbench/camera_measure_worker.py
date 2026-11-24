"""Segment exactly one saved JPEG using existing V4; no network or hardware."""
import sys,os,json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[1]/'yolo26_sem_v5_focus70/.deps'))
sys.path.insert(0,str(ROOT.parent/'web'))
os.environ['YOLO_CONFIG_DIR']=str(ROOT/'data/experiments/night-20261018/ultralytics_config')
import cv2,numpy as np
from live_model_worker import V4
def main():
    folder=Path(sys.argv[1])
    source=folder/'raw.jpg';image=cv2.imdecode(np.fromfile(str(source),np.uint8),1)
    if image is None:raise ValueError('Invalid camera JPEG')
    weights=ROOT.parents[1]/'yolo26_sem_v4_trial/runs/trial01/weights/best.pt'
    mask=V4(weights).predict(image)
    rgba=np.zeros((*mask.shape,4),np.uint8);rgba[:,:,3]=mask.astype(np.uint8)*255
    cv2.imencode('.png',rgba)[1].tofile(str(folder/'mask.png'))
    data=json.loads((folder/'capture.json').read_text(encoding='utf8'))
    data.update(width=image.shape[1],height=image.shape[0],model='V4-640',
        image_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        weights_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(),
        mask_pixels=int(mask.sum()),metric_calibrated=False)
    (folder/'result.json').write_text(json.dumps(data,ensure_ascii=False),encoding='utf8')
if __name__=='__main__':main()
