import json,sys
from rp import *
MPC=BASE['m_per_count']
def speed(fr,i,win=0.3):
    j=i
    while j>0 and fr[i]['t']-fr[j]['t']<win: j-=1
    if fr[i]['enc'] is None or fr[j]['enc'] is None or fr[i]['t']==fr[j]['t']: return None
    return (fr[i]['enc']-fr[j]['enc'])*MPC/(fr[i]['t']-fr[j]['t'])
for r in sys.argv[1:]:
    fr=load(r); lp,out=run(fr,BASE)
    c1=[e for e in lp.events if e['name']=='C1s']
    enc0=next(f['enc'] for f in fr if f['enc'] is not None)
    for e in c1[:3]:
        dc=e['d_cross']; print('==',r,'lap',e['lap'],'C1s d',dc)
        fc=None
        for i,(f,o) in enumerate(zip(fr,out)):
            x=o['odo_m']-dc
            if -3.6<=x<=0.6:
                v=speed(fr,i)
                print('%5.2f pwm %5.1f v %s %s/%s tgt %s far %s err %s bz %s %s fc %s lf %s/%s' %(x,f['pwm'] or 0,None if v is None else round(v,2),f['lev'],f['rsn'],f['tgt'],None if f['far'] is None else round(f['far'],2),None if f['err'] is None else round(f['err'],2),f['bzc'],(f['bzz'] or '')[:12],(f['bza'] or {}).get('first_curve'),f['lfc'],f['lfs']))
