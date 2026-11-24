"""Static exposure comparison at one track spot. usage: python expo_spot.py <label>
Sets camera controls over ssh (camset.py on Orin /tmp), takes workbench snapshots, restores auto at the end."""
import sys,json,time,subprocess,urllib.request,numpy as np,cv2
from pathlib import Path
HERE=Path(__file__).resolve().parent;CM=HERE.parent/'camera-measure';lab=sys.argv[1]
K=['-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local']
H={'Content-Type':'application/json','X-Workbench':'1'};U='http://127.0.0.1:8093/'
RESTORE='gamma=100 bright=0 contrast=1 sharp=2 backlight=1 auto=1 exp=166 auto=3'
CFG=[('auto',RESTORE),('m30 g300','auto=1 exp=300 gamma=300')]
def ssh(c):return subprocess.run(['ssh']+K+['jetson@192.168.0.244',c],capture_output=True,text=True,timeout=30).stdout.strip()
def get(p):return json.load(urllib.request.urlopen(urllib.request.Request(U+p,headers=H),timeout=10))
def snap():
    urllib.request.urlopen(urllib.request.Request(U+'api/camera-measure/capture',data=b'{}',headers=H,method='POST'),timeout=15)
    for _ in range(60):
        s=get('api/camera-measure/status')
        if s.get('state')!='running':break
        time.sleep(1)
    return s['id']
subprocess.run(['scp','-q']+K+[str(HERE/'camset.py'),'jetson@192.168.0.244:/tmp/camset.py'],check=True)
res=[];tiles=[];m0=None
try:
    for n,c in CFG:
        ssh('python3 /tmp/camset.py '+c);time.sleep(3);fps=get('vehicle/api/status')['capture_fps'];i=snap()
        raw=cv2.imdecode(np.fromfile(str(CM/i/'raw.jpg'),np.uint8),1);mk=cv2.imdecode(np.fromfile(str(CM/i/'mask.png'),np.uint8),cv2.IMREAD_UNCHANGED)[...,3]>0
        if m0 is None:m0=mk
        g=cv2.cvtColor(raw,cv2.COLOR_BGR2GRAY);mm=cv2.resize(m0.astype(np.uint8),(g.shape[1],g.shape[0]))>0;lo=np.zeros_like(mm);lo[g.shape[0]//2:]=True
        near=np.zeros_like(mk);near[int(mk.shape[0]*.42):]=True   # below ~row 150/360: the rows control uses
        r=dict(spot=lab,cfg=n,id=i,fps=fps,mean=round(float(g.mean()),1),line=round(float(g[mm].mean()),1) if mm.any() else None,floor=round(float(g[~mm&lo].mean()),1),
               mask_px=int(mk.sum()),iou=round(float((mk&m0).sum()/max(1,(mk|m0).sum())),3),near_iou=round(float((mk&m0&near).sum()/max(1,((mk|m0)&near).sum())),3),
               near_extra=int((mk&~m0&near).sum()),near_missing=int((m0&~mk&near).sum()))
        res.append(r);print(r,flush=True)
        t=cv2.resize(raw,(480,270));v=np.zeros((270,480,3),np.uint8);v[cv2.resize(mk.astype(np.uint8),(480,270))>0]=255;cv2.putText(t,f'{lab} {n} {fps}fps',(8,24),0,.6,(0,255,255),2);tiles.append(np.vstack([t,v]))
finally:
    print('restored',ssh('python3 /tmp/camset.py '+RESTORE+'; rm /tmp/camset.py'))
cv2.imencode('.jpg',np.hstack(tiles))[1].tofile(str(HERE/f'expo_{lab}.jpg'))
with open(HERE/'expo_spots.jsonl','a',encoding='utf8') as f:
    for r in res:f.write(json.dumps(r,ensure_ascii=False)+'\n')
