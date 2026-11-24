import sys,json,base64,glob,numpy as np,cv2
from pathlib import Path
sys.argv=['x'];src=open('camera-calib-20261109/check_camera.py',encoding='utf8').read().split("def main")[0].replace("Path(__file__).resolve().parent","Path('camera-calib-20261109').resolve()")
exec(src)
def an_mask(m):
    L=[];R=[];W=[]
    for y in range(150,271,5):
        l,r=edges(m,y)
        if l is not None:L.append((y,l))
        if r is not None:R.append((y,r))
        if l is not None and r is not None:
            p=project(np.array([[l,y],[r,y]],float),Hm);W.append(float(p[1][0]-p[0][0]))
    vp=None
    if len(L)>=8 and len(R)>=8:
        a1,b1=np.polyfit([y for y,_ in L],[x for _,x in L],1);a2,b2=np.polyfit([y for y,_ in R],[x for _,x in R],1)
        r1=np.sqrt(np.mean((np.polyval([a1,b1],[y for y,_ in L])-[x for _,x in L])**2));r2=np.sqrt(np.mean((np.polyval([a2,b2],[y for y,_ in R])-[x for _,x in R])**2))
        if abs(a1-a2)>1e-6 and max(r1,r2)<2.5:
            y=(b2-b1)/(a1-a2);vp=(a1*y+b1,y)
    return vp,W,len(L),len(R)
vp0,w0=analyse(Path('camera-calib-20261109/baseline'));print('baseline width %.1f vp y %.1f x %.1f'%(np.median([x[1] for x in w0]),vp0[1],vp0[0]))
if __name__=='__main__':
  for d in sorted([d for d in glob.glob('run-*-20261117') if __import__('os').path.exists(d+'/pid.jsonl')]):
    f=open(d+'/pid.jsonl',encoding='utf8',errors='replace');h=json.loads(f.readline());t0=h['started'];res=[]
    for l in f:
        if not l.startswith('{"kind": "sample"'):continue
        r=json.loads(l);t=float(r['wall'])-t0
        if t>0.7:break
        mp=r['vision'].get('mask_png')
        if not mp:continue
        m=cv2.imdecode(np.frombuffer(base64.b64decode(mp),np.uint8),cv2.IMREAD_UNCHANGED)
        if m.ndim==3:m=m[...,-1]
        m=cv2.resize(m,(640,360),interpolation=cv2.INTER_NEAREST)>0
        vp,W,nl,nr=an_mask(m)
        if vp and len(W)>=10:res.append((np.median(W),vp[1],vp[0]))
    if res:
        a=np.array(res);print('%s n=%d width %.1f cm  horizon y %.1f  vp x %.1f'%(d[4:17],len(res),np.median(a[:,0]),np.median(a[:,1]),np.median(a[:,2])))
    else:print(d[4:17],'no usable start frame')
