"""descents: per-run list of slope passes with reconstructed absolute IMU yaw and distance since the landing."""
from lib import *
import bisect, statistics as S
M=0.000222
def yaw_abs(p):
    hh=p.get('heading_hold') or {}
    if hh.get('yaw_deg') is not None: return float(hh['yaw_deg']),'hh'
    b=g(p,'lap_ff4','band') or {}
    if b.get('ref') is not None and b.get('turned_deg') is not None: return float(b['ref'])-float(b['turned_deg']),'band'
    return None,None
def gz_of(p):
    for k in (('inputs','gz'),('downhill_gz','gz'),('down_straight','gz'),('bottom_owner','gz'),('bottom_guide','gz'),('yaw_inner','gz')):
        v=g(p,*k)
        if v is not None: return float(v)
    return None
def descents(Q):
    out=[];i=0;n=len(Q)
    while i<n:
        pt=pitch(Q[i])
        if pt is not None and pt<-8 and g(Q[i],'_v','dash_route','route') in ('DONE','OPEN',None,'WAIT_CLIMB'):
            # mouth = first frame with pitch < -5 going back
            j=i
            while j>0 and (pitch(Q[j-1]) or 0)<-5: j-=1
            k=i
            while k<n and not ((pitch(Q[k]) or -99)>=-8 and k>i+15): k+=1
            if k-i<20: i=k+1;continue
            out.append(dict(i_mouth=j,i_steep=i,i_land=k if k<n else None))
            i=k+120
        else: i+=1
    return out
def build(Q,d):
    """adds per-frame arrays for frames [i_mouth-90, i_land+260]: t, x (m since landing, enc), yaw (abs, reconstructed), src"""
    a=max(0,d['i_mouth']-90); il=d['i_land']; b=min(len(Q),(il if il is not None else d['i_steep']+60)+430)
    F=Q[a:b]; t=[g(p,'_v','consumed_at') for p in F]
    # yaw: forward integrate gz from the last hh anchor, re-anchored when band appears
    yaw=[None]*len(F); src=[None]*len(F); last=None; mism=None
    for n_,p in enumerate(F):
        y,s=yaw_abs(p)
        if y is not None:
            if s=='band' and last is not None and src[n_-1]=='int' and mism is None:
                gz=gz_of(p) or 0.; pred=last[1]+gz*(t[n_]-last[0]); mism=y-pred
            yaw[n_]=y; src[n_]=s; last=(t[n_],y)
        elif last is not None:
            gz=gz_of(p)
            if gz is None or abs(gz)>200: gz=d.get('_gzprev',0.)
            d['_gzprev']=gz
            y=last[1]+gz*(t[n_]-last[0]); yaw[n_]=y; src[n_]='int'; last=(t[n_],y)
    d.update(a=a,b=b,F=F,t=t,yaw=yaw,src=src,mism=mism)
    # encoder distance since landing with interpolation: enc value + time first seen
    enc=[];
    for n_,p in enumerate(F):
        e=g(p,'inputs','enc_b')
        if e is not None and (not enc or enc[-1][1]!=e): enc.append((t[n_],e))
    d['enc']=enc
    def dist(tt):
        k=bisect.bisect_right([e[0] for e in enc],tt)-1
        if k<0: return None
        if k+1<len(enc):
            (t0,e0),(t1,e1)=enc[k],enc[k+1]; return (e0+(e1-e0)*(tt-t0)/(t1-t0))*M
        return enc[k][1]*M
    d['dist']=dist
    if il is not None:
        tl=t[il-a]; d['t_land']=tl; d0=dist(tl); d['x']=[None if dist(tt) is None else dist(tt)-d0 for tt in t]
    return d
def all_desc():
    R=[]
    for r in RUNS:
        st,Q=load(r)
        for d in descents(Q):
            d['run']=r; d['lap']=Q[d['i_steep']]['_lap']; build(Q,d); R.append(d)
    return R
if __name__=='__main__':
    for d in all_desc():
        print(d['run'],d['lap'],d['i_mouth'],d['i_steep'],d['i_land'],'mism',None if d['mism'] is None else round(d['mism'],1), 'srcs',{s:d['src'].count(s) for s in set(d['src'])})
