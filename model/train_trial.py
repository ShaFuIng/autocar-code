import json, os, time
from pathlib import Path
os.environ.setdefault('USER', 'jetson')
os.environ.setdefault('LOGNAME', 'jetson')
os.environ.setdefault('TORCHINDUCTOR_CACHE_DIR', '/tmp/torchinductor_jetson')
import torch
from ultralytics import YOLO

root = Path('/work')
os.chdir(root)
torch.set_num_threads(2)
started = time.time()
model = YOLO('/previous/runs/trial01/weights/best.pt')
print('MODEL_READY', model.task, 'CUDA', torch.cuda.is_available(), flush=True)
assert model.task == 'semantic' and torch.cuda.is_available()
result = model.train(data='/work/dataset/data.yaml', epochs=30, imgsz=640, batch=2,
    device=0, workers=0, cache=False, amp=False, optimizer='AdamW', lr0=.001,
    patience=10, seed=42, deterministic=True, project='/work/runs', name='trial01',
    exist_ok=False, plots=True, save=True, val=True, verbose=False,
    degrees=3, translate=.05, scale=.15, fliplr=.5, flipud=0,
    mosaic=0, mixup=0, copy_paste=0)
report = dict(elapsed_s=time.time()-started, save_dir=str(model.trainer.save_dir),
              metrics=result.results_dict if result is not None else {},
              limitation='Same-video blocked development validation only, not an independent track test.')
(root/'training_summary.json').write_text(json.dumps(report,indent=2,default=str))
print('TRAINING_COMPLETE', json.dumps(report,default=str), flush=True)
