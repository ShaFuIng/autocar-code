import json,statistics as S,sys
ENC=2.22e-4
def load(r):
    R=[]
    for l in open('run-%s-20261116/pid.jsonl'%r,encoding='utf8'):
        d=json.loads(l)
        if d.get('kind')=='sample' and 't' in d['proposal']: R.append(d['proposal'])
    return R
names=['curve1','diagonal_v','curve2','circle_straight','left_bend','curve3','ramp','top','dashed_turn','upper_straight','downhill','curve4','main_straight']
ALL={}
for r in sys.argv[1:]:
    Q=load(r); ALL[r]=Q
    pts=[]
    for q in Q:
        e=(q.get('inputs') or {}).get('enc_b')
        if e is not None and (not pts or e!=pts[-1][1]): pts.append((q['t'],e))
    def vavg(t0,t1):
        a=[p for p in pts if t0<=p[0]<=t1]
        return (a[-1][1]-a[0][1])*ENC/(a[-1][0]-a[0][0]) if len(a)>1 and a[-1][0]>a[0][0] else 0
    sg=[]
    for q in Q:
        s=(q.get('lap_pos') or {}).get('seg')
        if s and (not sg or sg[-1][0]!=s): sg.append([s,q['t']])
    sg.append(['END',Q[-1]['t']])
    c1=[t for s,t in sg if s=='curve1']
    print('=====',r,'laps',[round(b-a,1) for a,b in zip(c1,c1[1:])],'end in',sg[-2][0])
    lap=0; row={}
    print('   lap | '+' '.join('%5s'%n[:5] for n in names)+' | c3: gzp2p rev lostF | c2exit gzp2p v | c4exit gzp2p v pwm | bottom e95 sentmax')
    def win(t0,t1): return [q for q in Q if t0<=q['t']<=t1]
    def p2p(W): 
        g=[(q.get('inputs') or {}).get('gz') for q in W]; g=[x for x in g if x is not None]
        return max(g)-min(g) if g else 0
    laps=[]
    cur={}
    for (s,t),(s2,t2) in zip(sg,sg[1:]):
        if s=='curve1' and cur: laps.append(cur); cur={}
        cur.setdefault(s,[]).append((t,t2))
    laps.append(cur)
    for i,L in enumerate(laps):
        cells=[]
        for n in names: cells.append('%5.1f'%sum(b-a for a,b in L.get(n,[])) if n in L else '    -')
        ex=''
        if 'curve3' in L:
            a,b=L['curve3'][0][0],L['curve3'][-1][1]; W=win(a,b)
            st=[(q.get('play_comp') or {}).get('steer_us') or 0 for q in W]; rev=sum(1 for x,y,z in zip(st,st[1:],st[2:]) if (y-x)*(z-y)<0 and abs(z-y)>80 and abs(y-x)>80)
            ex+=' | %3.0f %2d %2d'%(p2p(W),rev,sum(1 for q in W if not q.get('valid')))
        else: ex+=' |   -  -  -'
        if 'curve2' in L:
            b=L['curve2'][-1][1]; W=win(b,b+3.0); ex+=' | %3.0f %.2f'%(p2p(W),vavg(b-0.6,b+0.4))
        else: ex+=' |   -    -'
        if 'curve4' in L:
            a,b=L['curve4'][0][0],L['curve4'][-1][1]; W=win(b-0.5,b+3.0); pw=[q.get('pwm') or 0 for q in win(b-1.0,b)]
            Wb=win(a-0.3,a+2.6); pe=[q.get('image_error') or 0 for q in Wb if q.get('valid')]; se=[(q.get('play_comp') or {}).get('steer_us') or 0 for q in Wb]
            ex+=' | %3.0f %.2f %2.0f | %2d %4.0f'%(p2p(W),vavg(b-0.6,b+0.4),S.mean(pw) if pw else 0,sum(1 for e in pe if e>=0.95),max(se) if se else 0)
        print('   %3d | '%i+' '.join(cells)+ex)
