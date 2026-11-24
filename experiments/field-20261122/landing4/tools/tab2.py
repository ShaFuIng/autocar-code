from desc import *
import statistics as S, pickle
R=[d for d in all_desc()]
XS=[-2.4,-2.0,-1.6,-1.2,-0.8,-0.6,-0.4,-0.2,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,1.5,1.75,2.0,2.25,2.5,2.75,3.0,3.5,4.0,4.5,5.0]
def interp_at(d,x):
    xs=d['x'];ys=d['yawrel']
    for n in range(1,len(xs)):
        if xs[n-1] is not None and xs[n] is not None and ys[n-1] is not None and ys[n] is not None and xs[n-1]<=x<=xs[n] and xs[n]>xs[n-1]:
            return ys[n-1]+(ys[n]-ys[n-1])*(x-xs[n-1])/(xs[n]-xs[n-1])
    return None
L=[]
for d in R:
    F=d['F'];a=d['a'];im=d['i_mouth']-a;tm=d['t'][im]
    up=[float(g(p,'heading_hold','yaw_deg')) for n,p in enumerate(F[:im]) if g(p,'heading_hold','yaw_deg') is not None and tm-d['t'][n]<=1.2]
    d['ref']=sum(up)/len(up); d['yawrel']=[None if y is None else y-d['ref'] for y in d['yaw']]
    if d['i_land'] is None: continue
    il=d['i_land']-a
    # distribute the integration mismatch: correct integrated frames in the last 0.35 s before the first band frame? keep raw
    co=[];ci=[];ym=[];emax=0;stmax=0;gzmin=0;gzmaxpos=0
    for n in range(il,len(F)):
        x=d['x'][n]
        if x is None or x>3.5: continue
        al=F[n].get('arc_lat') or {}
        if al.get('ok'):
            if al.get('clr_out_cm') is not None and x<=3.0: co.append((al['clr_out_cm'],x))
            if al.get('clr_in_cm') is not None: ci.append((al['clr_in_cm'],x))
        e=F[n].get('image_error')
        if e is not None and x<=1.5: emax=max(emax,e)
        gz=g(F[n],'inputs','gz')
        if gz is not None and abs(gz)<200:
            gzmin=min(gzmin,gz)
            if 0.8<=x<=3.0: gzmaxpos=max(gzmaxpos,gz)
    d['clr_out']=min(co) if co else (None,None); d['clr_in']=min(ci) if ci else (None,None); d['emax']=emax; d['gzmin']=gzmin; d['gzpos']=gzmaxpos
    es=[F[n].get('image_error') for n in range(max(0,il-15),il) if F[n].get('image_error') is not None]; d['e_slope']=sum(es)/len(es) if es else None
    d['lost_pre']=sum(1 for n in range(il-15,il) if F[n].get('reason')!='PROPOSAL_ONLY')
    d['prof']=[interp_at(d,x) for x in XS]
    L.append(d)
print('run lap  clr_out(x)  clr_in(x)  emax gzmin gzpos e_slope lost_pre | yawrel at -0.4 0 0.5 1.0 1.5 2.0 3.0')
for d in L:
    pr=dict(zip(XS,d['prof']))
    print(d['run'],d['lap'],d['clr_out'],d['clr_in'],'%.2f %.0f %.0f'%(d['emax'],d['gzmin'],d['gzpos']),None if d['e_slope'] is None else round(d['e_slope'],2),d['lost_pre'],'|',' '.join('%6.1f'%pr[x] if pr[x] is not None else ' None' for x in (-0.4,0,0.5,1.0,1.5,2.0,3.0)))
def med(ds,k): 
    v=[d['prof'][k] for d in ds if d['prof'][k] is not None]; return (S.median(v),S.pstdev(v),len(v)) if len(v)>2 else (float('nan'),0,len(v))
good=[d for d in L if d['clr_out'][0] is not None and d['clr_out'][0]>=-4 and d['clr_in'][0] is not None and d['clr_in'][0]>=-9 and not (d['run']=='46c9')]
outb=[d for d in L if d['clr_out'][0] is not None and d['clr_out'][0]<-4]
inb=[d for d in L if d['clr_in'][0] is not None and d['clr_in'][0]<-9]
print('good',[(d['run'],d['lap']) for d in good]);print('out-bad',[(d['run'],d['lap']) for d in outb]);print('in-bad',[(d['run'],d['lap']) for d in inb])
print('   x    all(med sd n)      good            out-bad         in-bad')
for k,x in enumerate(XS):
    print('%5.2f'%x,'  '.join('%7.1f %4.1f %2d'%med(ds,k) for ds in (L,good,outb,inb)))
