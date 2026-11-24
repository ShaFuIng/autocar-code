"""Exit oscillation after each curve: period, amplitude, decay, lags and which control term moves the command.
usage: python exit_osc.py run-.... ..."""
import json,sys,numpy as np,collections
ENC=2.22e-4
def load(r):
    D=[]
    for l in open(r+'/pid.jsonl',encoding='utf8'):
        if '"kind": "sample"' not in l[:40]:continue
        d=json.loads(l)
        if 't' in d['proposal']:D.append(d['proposal'])
    return D
def lagcorr(a,b,dt,maxlag=0.6):
    # lag (s) of b behind a maximising |corr|
    a=a-a.mean();b=b-b.mean();best=(0,0)
    for n in range(0,int(maxlag/dt)+1):
        x=a[:len(a)-n];y=b[n:]
        if len(x)<10 or x.std()==0 or y.std()==0:continue
        c=float(np.corrcoef(x,y)[0,1])
        if abs(c)>abs(best[1]):best=(n*dt,c)
    return best
for r in sys.argv[1:]:
    Q=load(r);T=np.array([q['t'] for q in Q]);hz=len(Q)/(T[-1]-T[0])
    pts=[]
    for q in Q:
        e=(q.get('inputs') or {}).get('enc_b')
        if e is not None and (not pts or e!=pts[-1][1]):pts.append((q['t'],e))
    te=np.array([p[0] for p in pts]);se=np.array([p[1] for p in pts])*ENC
    ev=[];last=None
    for i,q in enumerate(Q):
        a=(q.get('lap_pos') or {}).get('anchor')
        if a!=last and a in('C1e','C2e','C4e'):ev.append((a,q['t']))
        if a:last=a
    out=collections.defaultdict(list)
    for a,t0 in ev:
        m=(T>=t0+0.2)&(T<=t0+4.2)
        if m.sum()<30:continue
        W=[q for q,k in zip(Q,m) if k];t=T[m];dt=0.02;tg=np.arange(t[0],t[-1],dt)
        f=lambda key:np.interp(tg,t,np.array([key(q) for q in W],float))
        gz=f(lambda q:(q.get('inputs') or {}).get('gz') or 0);e=f(lambda q:q.get('image_error') or 0)
        u=f(lambda q:(q.get('play_comp') or {}).get('steer_us',q['steer_us']));raw=f(lambda q:q['steer_us'])
        k=f(lambda q:(q.get('gain_schedule') or {}).get('k') or 1.0)
        P=f(lambda q:q.get('p') or 0)*k;Dm=f(lambda q:q.get('d') or 0)*k
        yi=f(lambda q:(q.get('yaw_inner') or {}).get('term_us') if (q.get('yaw_inner') or {}).get('term_us') is not None else ((q.get('yaw_inner') or {}).get('term_req_us') or 0))
        pc=f(lambda q:(q.get('play_comp') or {}).get('offset_us') or 0)
        v=(np.interp(tg[-1],te,se)-np.interp(tg[0],te,se))/(tg[-1]-tg[0])
        g=gz-np.polyval(np.polyfit(tg,gz,1),tg)
        F=np.abs(np.fft.rfft(g*np.hanning(len(g))));fr=np.fft.rfftfreq(len(g),dt);sel=(fr>=0.4)&(fr<=3.0);fpk=fr[sel][np.argmax(F[sel])]
        # peaks for decay
        s=np.sign(g);zc=np.where(np.diff(s)!=0)[0];pk=[np.abs(g[a_:b_+1]).max() for a_,b_ in zip(zc[:-1],zc[1:]) if b_-a_>=3]
        dec=np.median([pk[i+1]/pk[i] for i in range(len(pk)-1) if pk[i]>3]) if len(pk)>=3 else np.nan
        l_ug=lagcorr(u,-g,dt);l_eu=lagcorr(e,u,dt);l_eg=lagcorr(e,-g,dt,1.0)
        du=np.diff(raw);sg=np.sign(du[du!=0]);rev=(np.diff(sg)!=0).sum()/(tg[-1]-tg[0])
        out[a].append(dict(v=v,p2p=np.ptp(g),sd=g.std(),f=fpk,dec=dec,n_half=len(pk),e_p2p=np.ptp(e),u_sd=u.std(),lag_ug=l_ug[0],c_ug=l_ug[1],lag_eu=l_eu[0],lag_eg=l_eg[0],sdP=P.std(),sdD=Dm.std(),sdY=yi.std(),sdPC=pc.std(),rev=rev,
                           cP=np.corrcoef(P,u-u.mean())[0,1] if P.std()>0 else 0,cD=np.corrcoef(Dm,u-u.mean())[0,1] if Dm.std()>0 else 0,cY=np.corrcoef(yi,u-u.mean())[0,1] if yi.std()>0 else 0))
    print('=====',r[4:8],'%.0f frames/s'%hz)
    for a in ('C1e','C2e','C4e'):
        X=out.get(a,[])
        if not X:continue
        md=lambda k:np.nanmedian([x[k] for x in X])
        print('  after %s (n %d) v %.2f | gz p2p %.0f sd %.1f | period %.2fs | half-cycles %.0f, amplitude ratio per half-cycle %.2f | e p2p %.2f | steer sd %.0f, cmd reversals %.1f/s | lag cmd->yaw %.2fs (corr %.2f), err->cmd %.2fs, err->yaw %.2fs | term sd: P %.0f D %.0f yaw_inner %.0f playcomp %.0f | corr with cmd: P %.2f D %.2f yaw %.2f'%(a,len(X),md('v'),md('p2p'),md('sd'),1/md('f'),md('n_half'),md('dec'),md('e_p2p'),md('u_sd'),md('rev'),md('lag_ug'),md('c_ug'),md('lag_eu'),md('lag_eg'),md('sdP'),md('sdD'),md('sdY'),md('sdPC'),md('cP'),md('cD'),md('cY')))
