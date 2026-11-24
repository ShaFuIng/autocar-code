"""Build a digest-pinned JetPack 6 notebook environment on Orin."""
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent


def run(*args, capture=False):
    return subprocess.run(args, cwd=ROOT, check=True, text=True,
                          stdout=subprocess.PIPE if capture else None).stdout


image = json.loads(run('docker','image','inspect','ultralytics/ultralytics:latest-jetson-jetpack6',capture=True))[0]
digest = image['RepoDigests'][0]
(ROOT/'.env').write_text('BASE_IMAGE='+digest+'\nORIN_HOST=10.130.227.240\nBIND_ADDR=10.130.227.240\n')
print('BASE_PINNED',digest,flush=True)
run('docker','compose','build')
(ROOT/'notebooks/.ultralytics/Ultralytics').mkdir(parents=True, exist_ok=True)
run('docker','compose','up','-d')
verify = '''import json, torch, torchvision, ultralytics, tensorrt
assert torch.cuda.is_available(), 'CUDA not available'
x=torch.randn(256,256,device='cuda'); y=x@x; torch.cuda.synchronize()
boxes=torch.tensor([[0.,0.,10.,10.],[1.,1.,9.,9.]],device='cuda')
scores=torch.tensor([.9,.8],device='cuda')
assert torchvision.ops.nms(boxes,scores,.5).numel()==1
print(json.dumps({'torch':torch.__version__,'cuda':torch.version.cuda,
 'gpu':torch.cuda.get_device_name(0),'torchvision':torchvision.__version__,
 'ultralytics':ultralytics.__version__,'tensorrt':tensorrt.__version__,
 'gpu_matrix':'passed','gpu_nms':'passed'}))
'''
report=run('docker','exec','orin-notebook','python3','-c',verify,capture=True)
(ROOT/'gpu_validation.txt').write_text(report)
print(report,flush=True)
(ROOT/'requirements-installed.txt').write_text(run('docker','exec','orin-notebook','python3','-m','pip','freeze',capture=True))
run('docker','exec','orin-notebook','python3','-m','jupyter','nbconvert',
    '--to','notebook','--execute','--ExecutePreprocessor.timeout=60',
    '--output','01_GPU與相機驗證_已執行.ipynb','/workspace/01_GPU與相機驗證.ipynb')
print('NOTEBOOK_EXECUTED',flush=True)
