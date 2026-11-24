from desc import *
import statistics as S
R=all_desc()
def at_x(d,x):
    il=d['i_land']-d['a']
    for n in range(il,len(d['F'])):
        if d['x'][n] is not None and d['x'][n]>=x: return n
    return None
rows=[]
print('run lap | refs rel up_mean: hh_ref usD band_ref | mouth land x.5 1.0 1.5 2.0 3.0 4.5 | main | vland | e_slope')
for d in R:
    F=d['F'];a=d['a'];im=d['i_mouth']-a
    hy=[(n,float(g(p,'heading_hold','yaw_deg'))) for n,p in enumerate(F[:im]) if g(p,'heading_hold','yaw_deg') is not None and g(p,'heading_hold','reason') not in('TURN',)]
    # last 1.2 s of the upper straight
    tm=d['t'][im]
    up=[y for n,y in hy if tm-d['t'][n]<=1.2]
    up_mean=sum(up)/len(up); d['up_mean']=up_mean; d['up_n']=len(up)
    up2=[y for n,y in hy if tm-d['t'][n]<=2.0]; d['up_mean2']=sum(up2)/len(up2)
    hh_ref=[g(p,'heading_hold','ref_deg') for p in F[:im] if g(p,'heading_hold','ref_deg') is not None][-1]
    usD=[g(p,'upper_single','edge','D') for p in F[:im] if g(p,'upper_single','edge','D') is not None]
    usD=usD[-1] if usD else None
    ht=[g(p,'upper_single','h_target_deg') for p in F[:im] if g(p,'upper_single','h_target_deg') is not None]; ht=ht[-1] if ht else None
    d['hh_ref']=hh_ref;d['usD']=usD;d['ht']=ht
    if d['i_land'] is None:
        print(d['run'],d['lap'],'FAIL  hh_ref %.1f usD %s ht %s mouth %.1f'%(hh_ref-up_mean,None if usD is None else round(usD-up_mean,1),None if ht is None else round(ht-up_mean,1),d['yaw'][im]-up_mean)); continue
    il=d['i_land']-a
    bref=None;yl=None
    for n in range(il,len(F)):
        b=g(F[n],'lap_ff4','band')
        if b and b.get('ref') is not None:
            bref=b['ref']; yl=b['ref']-b['turned_deg']+F[n]['lap_ff4']['turned_deg']; break
    d['bref']=bref;d['yaw_land']=yl
    ys=[]
    for x in (0.5,1.0,1.5,2.0,3.0,4.5):
        n=at_x(d,x); ys.append(None if n is None or d['yaw'][n] is None else d['yaw'][n]-up_mean)
    # main straight: mean yaw over x in [10.5,12.5]
    ms=[d['yaw'][n] for n in range(il,len(F)) if d['x'][n] is not None and 10.5<=d['x'][n]<=12.5 and d['yaw'][n] is not None]
    main=sum(ms)/len(ms)-up_mean if ms else None
    d['main']=main
    es=[F[n].get('image_error') for n in range(max(0,il-15),il) if F[n].get('image_error') is not None]
    v=g(F[il+3],'lap_ff4','v')
    d['rel']=dict(mouth=d['yaw'][im]-up_mean,land=yl-up_mean,xs=ys)
    print(d['run'],d['lap'],'| %5.1f %5s %5s %5.1f | %5.1f %5.1f '%(hh_ref-up_mean,None if usD is None else round(usD-up_mean,1),None if ht is None else round(ht-up_mean,1),bref-up_mean,d['yaw'][im]-up_mean,yl-up_mean),' '.join('%6.1f'%y if y is not None else '  None' for y in ys),'| %s'%(None if main is None else round(main,1)),'|',v,'| %.2f'%(sum(es)/len(es) if es else float('nan')))
L=[d for d in R if d['i_land'] is not None]
def ms(v): v=[x for x in v if x is not None]; return '%.1f±%.1f [%.1f..%.1f] n%d'%(S.mean(v),S.pstdev(v),min(v),max(v),len(v))
print('mouth',ms([d['rel']['mouth'] for d in L])); print('land',ms([d['rel']['land'] for d in L]))
for k,x in enumerate((0.5,1.0,1.5,2.0,3.0,4.5)): print('x',x,ms([d['rel']['xs'][k] for d in L]))
print('main - up_mean',ms([d['main'] for d in L])); print('main - up_mean2',ms([d['main']+d['up_mean']-d['up_mean2'] for d in L if d['main'] is not None]))
print('main - hh_ref',ms([d['main']+d['up_mean']-d['hh_ref'] for d in L if d['main'] is not None]))
print('main - bref',ms([d['main']+d['up_mean']-d['bref'] for d in L if d['main'] is not None]))
print('main - usD',ms([d['main']+d['up_mean']-d['usD'] for d in L if d['main'] is not None and d['usD'] is not None]))
print('main - ht',ms([d['main']+d['up_mean']-d['ht'] for d in L if d['main'] is not None and d['ht'] is not None]))
print('main - yaw_land',ms([d['main']+d['up_mean']-d['yaw_land'] for d in L if d['main'] is not None]))
import pickle
