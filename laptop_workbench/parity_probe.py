"""One-image read-only inference probe; run inside isolated Orin image or locally."""
import sys,json,hashlib,os,base64
from pathlib import Path
base=Path(sys.argv[1])
if len(sys.argv)>2:
    sys.path.insert(0,sys.argv[2]);os.environ['YOLO_CONFIG_DIR']=str(base/'Orin_Interface_v1/laptop_workbench/data/experiments/night-20261018/ultralytics_config')
import torch,cv2,numpy as np
from ultralytics import YOLO,__version__
folder=base/'yolo26_sem_v4_trial'
p=folder/'dataset/images/train/v003_006.jpg';weights=folder/'runs/trial01/weights/best.pt'
model=YOLO(str(weights));im=cv2.imdecode(np.fromfile(p,np.uint8),1)
mask=model.predict(im,imgsz=640,device=0,verbose=False)[0].semantic_mask.data.cpu().numpy().astype(np.uint8)
result=dict(torch=torch.__version__,cuda=torch.version.cuda,ultralytics=__version__,gpu=torch.cuda.get_device_name(0),image_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),weights_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(),mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest(),foreground_pixels=int((mask==1).sum()),shape=list(mask.shape),imgsz=640)
result['packed_mask']=base64.b64encode(np.packbits(mask==1).tobytes()).decode('ascii')
print('PARITY_JSON:'+json.dumps(result))
