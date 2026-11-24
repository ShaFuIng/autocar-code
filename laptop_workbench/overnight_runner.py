"""Offline experiments only. No network, serial, camera, or controller imports."""
from pathlib import Path
import os, sys, json, time, hashlib, traceback, random
from datetime import datetime
ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent/'data/experiments/night-20261018'
OUT.mkdir(parents=True, exist_ok=True)
os.environ['YOLO_CONFIG_DIR'] = str(OUT/'ultralytics_config')
sys.path.insert(0, str(ROOT/'yolo26_sem_v5_focus70/.deps'))
sys.path.insert(0, str(ROOT/'Orin_Interface_v1/shadow_workbench'))
import cv2
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
from ultralytics import YOLO, __version__ as uv
from route_a import RouteTracker
from route_b import MotionTracker
from route_unified import UnifiedTracker

def save(name, data):
    p=OUT/name; p.parent.mkdir(parents=True,exist_ok=True)
    tmp=p.with_suffix(p.suffix+'.tmp')
    tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    os.replace(tmp,p)
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''): h.update(b)
    return h.hexdigest()
def read(p): return cv2.imdecode(np.fromfile(p,np.uint8),cv2.IMREAD_UNCHANGED)
def jpg(p,im):
    p.parent.mkdir(parents=True,exist_ok=True)
    cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,85])[1].tofile(str(p))
STATE=dict(state='running',started=datetime.now().isoformat(),jobs=[],results=[],videos=[],errors=[],
    safety='純離線檔案分析；無車輛／序列埠介面；未更換部署權重。',
    limitations=['驗證集是同一路線的時間分段開發集，不是獨立場地測試。',
    '既有 v4/v5 曾接觸同來源資料；新小模型從零訓練，不能視為等量預訓練的公平架構排名。',
    '原資料保留一張未定位的局部漏標影像；未知區域按背景計算，分數可能受影響。',
    '路徑出現率僅表示演算法有輸出，沒有正確車道真值，不能當準確率或實車安全證據。',
    '未標定地面尺度、輪角、速度；未執行 Pure Pursuit、馬達／舵機或實車試跑。',
    'Fast-SCNN-inspired 是本次簡化實作，非官方原架構／原論文數字。'],
    sources=[dict(name='U-Net 原始方法',url='https://lmb.informatik.uni-freiburg.de/people/ronneber/u-net/'),
    dict(name='Fast-SCNN 原始論文',url='https://arxiv.org/abs/1902.04502'),
    dict(name='Torchvision LR-ASPP 參考（本輪未訓練）',url='https://docs.pytorch.org/vision/main/models/generated/torchvision.models.segmentation.lraspp_mobilenet_v3_large.html')])
if '--resume' in sys.argv and (OUT/'report.json').exists():
    STATE=json.loads((OUT/'report.json').read_text(encoding='utf-8'))
    STATE['state']='running'
    for previous in STATE['jobs']:
        if previous['state']=='running':previous['state']='interrupted'
def status(**kw):
    STATE.update(kw);STATE['updated']=datetime.now().isoformat();save('report.json',STATE)
def job(name,fn):
    row=dict(name=name,state='running',started=time.time());STATE['jobs'].append(row);status(current=name)
    try:
        value=fn();row.update(state='completed',elapsed_s=round(time.time()-row['started'],2));status();return value
    except Exception:
        err=traceback.format_exc();print(err,flush=True);row.update(state='failed',error=err);STATE['errors'].append(dict(job=name,error=err));status();return None

class Conv(nn.Sequential):
    def __init__(self,a,b,stride=1):
        super().__init__(nn.Conv2d(a,b,3,stride,1,bias=False),nn.BatchNorm2d(b),nn.ReLU(inplace=True),
                         nn.Conv2d(b,b,3,1,1,bias=False),nn.BatchNorm2d(b),nn.ReLU(inplace=True))
class TinyUNet(nn.Module):
    def __init__(self):
        super().__init__();self.a=Conv(3,16);self.b=Conv(16,32);self.c=Conv(32,64);self.d=Conv(96,32);self.e=Conv(48,16);self.out=nn.Conv2d(16,2,1)
    def forward(self,x):
        a=self.a(x);b=self.b(F.max_pool2d(a,2));c=self.c(F.max_pool2d(b,2))
        d=self.d(torch.cat([F.interpolate(c,size=b.shape[-2:],mode='bilinear',align_corners=False),b],1))
        return self.out(self.e(torch.cat([F.interpolate(d,size=a.shape[-2:],mode='bilinear',align_corners=False),a],1)))
class DS(nn.Sequential):
    def __init__(self,a,b,s=1):
        super().__init__(nn.Conv2d(a,a,3,s,1,groups=a,bias=False),nn.BatchNorm2d(a),nn.ReLU(),nn.Conv2d(a,b,1,bias=False),nn.BatchNorm2d(b),nn.ReLU())
class FastInspired(nn.Module):
    def __init__(self):
        super().__init__();self.down=nn.Sequential(nn.Conv2d(3,24,3,2,1),nn.ReLU(),DS(24,32,2),DS(32,48,2));self.global_=nn.Sequential(DS(48,64,2),DS(64,96,2),DS(96,96));self.low=nn.Conv2d(96,64,1);self.high=nn.Conv2d(48,64,1);self.out=nn.Sequential(DS(64,64),nn.Conv2d(64,2,1))
    def forward(self,x):
        a=self.down(x);b=self.global_(a);b=b+F.adaptive_avg_pool2d(b,1)
        fused=F.relu(self.high(a)+F.interpolate(self.low(b),size=a.shape[-2:],mode='bilinear',align_corners=False))
        return F.interpolate(self.out(fused),size=x.shape[-2:],mode='bilinear',align_corners=False)

def tensor(im):
    im=cv2.resize(im,(640,360));return torch.from_numpy(im[:,:,::-1].copy().transpose(2,0,1)).float().div_(255)
def pairs(split):
    base=ROOT/'boundary_masks_v4_checked/dataset'
    rows=[]
    for p in sorted((base/'images'/split).glob('*')):
        m=base/'masks'/split/(p.stem+'.png')
        assert m.exists(),m
        mask=read(m);assert set(np.unique(mask)).issubset({0,1}),np.unique(mask)
        rows.append((p,m))
    assert rows,split
    return rows
def audit():
    train,val=pairs('train'),pairs('val');a={sha(p) for p,m in train};b={sha(p) for p,m in val};assert not a&b,'Duplicate image in train/val'
    manifest={s:[dict(image=str(p.relative_to(ROOT)),mask=str(m.relative_to(ROOT)),sha256=sha(p),mask_sha256=sha(m)) for p,m in pairs(s)] for s in ['train','val']}
    save('dataset_manifest.json',manifest)
    status(environment=dict(python=sys.version,torch=torch.__version__,cuda=torch.version.cuda,ultralytics=uv,opencv=cv2.__version__,gpu=torch.cuda.get_device_name(0),orin_torch='2.10.0 (local differs)',functional_v4_reproduction=True),dataset=dict(train=len(train),val=len(val),duplicate_hashes=0,manifest='dataset_manifest.json'))

def fit(name,cls):
    torch.manual_seed(42);np.random.seed(42);random.seed(42)
    model=cls().cuda()
    if '--resume' in sys.argv and (OUT/(name+'-training.json')).exists() and (OUT/(name+'.pt')).exists():
        history=json.loads((OUT/(name+'-training.json')).read_text())['history']
        if len(history)>=30:
            model.load_state_dict(torch.load(OUT/(name+'.pt'),weights_only=True)['model']);return SmallPredictor(model)
    opt=torch.optim.AdamW(model.parameters(),lr=.001,weight_decay=.0001)
    data=[(tensor(read(p)),torch.from_numpy(cv2.resize(read(m),(640,360),interpolation=cv2.INTER_NEAREST)).long()) for p,m in pairs('train')]
    history=[];best=-1;st=time.time()
    for epoch in range(1,31):
        model.train();random.shuffle(data);losses=[]
        for i in range(0,len(data),4):
            batch=data[i:i+4];x=torch.stack([r[0] for r in batch]).cuda();y=torch.stack([r[1] for r in batch]).cuda()
            if random.random()<.5:x=x.flip(-1);y=y.flip(-1)
            pred=model(x);ce=F.cross_entropy(pred,y,weight=torch.tensor([1.,4.],device='cuda'))
            prob=pred.softmax(1)[:,1];gt=(y==1).float();dice=1-(2*(prob*gt).sum()+1)/(prob.sum()+gt.sum()+1)
            loss=ce+dice;opt.zero_grad();loss.backward();opt.step();losses.append(loss.item())
        predictor=SmallPredictor(model);score=evaluate(predictor,save_samples=False)
        history.append(dict(epoch=epoch,loss=float(np.mean(losses)),iou=score['iou']))
        if score['iou']>best:
            best=score['iou'];torch.save(dict(model=model.state_dict(),architecture=name,epoch=epoch,seed=42,input=[360,640]),OUT/(name+'.pt'))
        save(name+'-training.json',dict(history=history,parameters=sum(p.numel() for p in model.parameters()),split='v4 temporal train63/val15',selection='best development IoU',pretrained=False))
        status(current=f'{name} 訓練 {epoch}/30',epoch=epoch)
        print(name,epoch,score['iou'],flush=True)
        if time.time()-st>1500:break
    model.load_state_dict(torch.load(OUT/(name+'.pt'),weights_only=True)['model']);return SmallPredictor(model)
class SmallPredictor:
    def __init__(self,m):self.m=m
    def __call__(self,im):
        self.m.eval()
        with torch.inference_mode():p=self.m(tensor(im)[None].cuda()).argmax(1)[0].cpu().numpy().astype(np.uint8)
        return cv2.resize(p,(im.shape[1],im.shape[0]),interpolation=cv2.INTER_NEAREST)>0
class YoloPredictor:
    def __init__(self,path,size):self.m=YOLO(str(path));self.size=size
    def __call__(self,im):
        return self.m.predict(im,imgsz=self.size,device=0,verbose=False)[0].semantic_mask.data.cpu().numpy()==1
def opencv(im):
    lab=cv2.cvtColor(im,cv2.COLOR_BGR2LAB);l=lab[:,:,0];sat=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)[:,:,1]
    contrast=cv2.subtract(l,cv2.GaussianBlur(l,(0,0),9))
    mask=((l>140)&(sat<90)&(contrast>12)).astype(np.uint8)
    mask[:int(im.shape[0]*.2)]=0
    return cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))>0
def evaluate(pred,save_samples=False,name=''):
    tp=fp=fn=0;lat=[];per=[]
    for p,m in pairs('val'):
        im=read(p);gt=read(m)==1;torch.cuda.synchronize();t=time.perf_counter();mask=pred(im);torch.cuda.synchronize();lat.append((time.perf_counter()-t)*1000)
        a=int((mask&gt).sum());b=int((mask&~gt).sum());c=int((~mask&gt).sum());tp+=a;fp+=b;fn+=c
        per.append(dict(image=p.name,iou=a/max(1,a+b+c)))
        if save_samples:
            view=im.copy();view[mask]=(view[mask]*.5+np.array([255,0,255])*.5).astype(np.uint8);jpg(OUT/'validation'/name/(p.stem+'.jpg'),view)
    return dict(iou=tp/max(1,tp+fp+fn),dice=2*tp/max(1,2*tp+fp+fn),precision=tp/max(1,tp+fp),recall=tp/max(1,tp+fn),latency_median_ms=float(np.median(lat[1:])),latency_p95_ms=float(np.percentile(lat[1:],95)),tp=tp,fp=fp,fn=fn,per_image=per)

def videos(pred,name):
    sources=[ROOT/'完整跑道錄影_20261016_155847_3e961ecb',ROOT/'Orin_Interface_v1/field_test/review_latest/08dfcb8c29f74ae78dba9f294060e8e9']
    geom=json.loads((ROOT/'Orin_Interface_v1/shadow_workbench/geometry.json').read_text())
    for source in sources:
        source_id='fulltrack' if source==sources[0] else 'straight'
        if any(v['model']==name and v['source']==source_id for v in STATE['videos']):continue
        a=RouteTracker();b=MotionTracker(np.array(geom['H']));c=UnifiedTracker();rows=[];first=None;counts={'A':0,'B':0,'C':0};modes={};sample=[]
        for video in sorted(source.glob('video_*.avi')):
            stamps=[json.loads(l)['time'] for l in video.with_name(video.stem.replace('video_','frames_')+'.jsonl').read_text().splitlines()]
            cap=cv2.VideoCapture(str(video));local=0
            while True:
                ok,im=cap.read()
                if not ok:break
                dt=datetime.fromisoformat(stamps[local]);first=first or dt;t=(dt-first).total_seconds();mask=pred(im)
                results={'A':a.update(mask,t),'B':b.update(im,mask,t),'C':c.update(im,mask,t)}
                for k,r in results.items():counts[k]+=bool(r.get('path'));modes[k+':'+r['mode']]=modes.get(k+':'+r['mode'],0)+1
                idx=len(rows);row=dict(frame=idx,source=video.name,source_frame=local,capture_s=t,time=stamps[local],**results);rows.append(row)
                if idx%30==0:
                    folder='fulltrack' if source==sources[0] else 'straight';rel=f'frames/{folder}/{idx:06d}.jpg';jpg(OUT/rel,im)
                    view=im.copy();view[mask]=(view[mask]*.6+np.array([255,0,255])*.4).astype(np.uint8)
                    for k,col in [('A',(0,255,255)),('B',(255,170,0)),('C',(0,255,0))]:
                        pts=np.array(results[k].get('path',[]),np.int32)
                        if len(pts)>1:cv2.polylines(view,[pts],False,col,3)
                    overlay=f'overlays/{name}/{folder}/{idx:06d}.jpg';jpg(OUT/overlay,view)
                    sample.append(dict(frame=idx,capture_s=t,raw=rel,overlay=overlay,modes={k:r['mode'] for k,r in results.items()}))
                local+=1
                if idx%100==0:status(current=f'{name} {source.name} 影格 {idx}',processed_frame=idx)
            cap.release();assert local==len(stamps),(video,local,len(stamps))
        assert rows,'No decoded frames'
        folder='fulltrack' if source==sources[0] else 'straight';file=f'{name}-{folder}-frames.json';save(file,rows)
        result=dict(model=name,source=folder,frames=len(rows),duration_s=rows[-1]['capture_s'],path_output_count=counts,modes=modes,samples=sample,rows=file)
        STATE['videos'].append(result);status()

def main():
    torch.set_num_threads(4);cv2.setNumThreads(2);assert torch.cuda.is_available()
    job('環境與資料拆分稽核',audit)
    if not STATE.get('dataset'):raise RuntimeError('dataset audit failed')
    configs=[('v4-640','yolo26_sem_v4_trial/runs/trial01/weights/best.pt',640),('v4-960','yolo26_sem_v4_trial/runs/trial01/weights/best.pt',960),('v3-640','yolo26_sem_v3_trial/runs/trial01/weights/best.pt',640),('v5-focus02','yolo26_sem_v5_focus70/runs/focus02/weights/best.pt',640),('v5-focus03','yolo26_sem_v5_focus70/runs/focus03/weights/best.pt',960)]
    def run(name,pred,weights=None):
        if not any(r['model']==name for r in STATE['results']):
            result=evaluate(pred,True,name);result.update(model=name,weights_sha256=sha(weights) if weights else None);STATE['results'].append(result);status()
        job(name+' 全影片 + A/B/C 路徑',lambda:videos(pred,name))
    for name,path,size in configs:
        def existing(name=name,path=path,size=size):
            pred=YoloPredictor(ROOT/path,size);run(name,pred,ROOT/path);del pred;torch.cuda.empty_cache()
        job(name+' 驗證與回放',existing)
    job('OpenCV 局部對比基線',lambda:run('opencv-local',opencv))
    for name,cls in [('tiny-unet',TinyUNet),('fast-scnn-inspired',FastInspired)]:
        pred=job(name+' 從零訓練',lambda name=name,cls=cls:fit(name,cls))
        if pred is not None:job(name+' 驗證與回放',lambda:run(name,pred,OUT/(name+'.pt')))
        del pred;torch.cuda.empty_cache()
    status(state='completed_with_errors' if STATE['errors'] else 'completed',current='完成；候選尚未部署',finished=datetime.now().isoformat())
if __name__=='__main__':
    try:main()
    except Exception:
        status(state='failed',fatal=traceback.format_exc());raise
