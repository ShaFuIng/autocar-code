import json,base64,sys,numpy as np,cv2,collections
def load(r,tmax=None):
    Q=[json.loads(l) for l in open(r+'/pid.jsonl',encoding='utf8')];Q=[d for d in Q if d.get('kind')=='sample' and 't' in d['proposal']]
    if tmax:Q=[d for d in Q if d['proposal']['t']-Q[0]['proposal']['t']<tmax]
    return Q
def stats(Q):
    out=collections.defaultdict(lambda:collections.defaultdict(list));last=None;pe=None;ps=None
    for d in Q:
        p=d['proposal'];v=d['vision'];s=(p.get('lap_pos') or {}).get('seg') or last;last=s
        b=v.get('mask_png')
        if not b:continue
        m=cv2.imdecode(np.frombuffer(base64.b64decode(b.split(',')[-1]),np.uint8),cv2.IMREAD_UNCHANGED)
        m=(m[...,3] if m.ndim==3 and m.shape[2]==4 else (m if m.ndim==2 else m[...,0]))>0
        n,lab,st,_=cv2.connectedComponentsWithStats(m.astype(np.uint8),8)
        a=st[1:,4] if n>1 else np.array([])
        h=m.shape[0];o=out[s]
        o['px'].append(int(m.sum()));o['specks'].append(int((a<0.0004*m.size).sum()));o['blobs'].append(int((a>=0.0004*m.size).sum()))
        e=p.get('image_error');src=(v.get('path_diagnostic') or {}).get('selected_source')
        if pe is not None and e is not None:o['jump'].append(abs(e-pe)>0.25)
        if ps is not None:o['sw'].append(src!=ps)
        pe=e;ps=src
    return out
runs=[('auto 7f36','run-7f36-20261117',None),('manual f764','run-f764-20261117',None),('auto d4e1 lap1','run-d4e1-20261116',80)]
R={n:stats(load(r,t)) for n,r,t in runs}
for s in ['curve1','diagonal_v','curve2','circle_straight','left_bend','curve3','ramp','top','dashed_turn','upper_straight','downhill','curve4','main_straight']:
    print(s)
    for n in R:
        o=R[n].get(s)
        if not o or not o['px']:continue
        print('   %-15s n %3d  mask px med %6d  small specks/frame %.2f  blobs/frame %.2f  err jumps>0.25 %4.1f%%  source switches %4.1f%%'%(n,len(o['px']),np.median(o['px']),np.mean(o['specks']),np.mean(o['blobs']),100*np.mean(o['jump']) if o['jump'] else 0,100*np.mean(o['sw']) if o['sw'] else 0))
