"""straight swing: python strchk.py run [run ...]"""
import json, sys, numpy as np, collections
for run in sys.argv[1:]:
    S=collections.defaultdict(list)
    for l in open(run+'/pid.jsonl',encoding='utf8'):
        if not l.startswith('{"kind": "sample"'): continue
        p=json.loads(l).get('proposal') or {}; lp=p.get('lap_pos') or {}
        if lp.get('seg') in ('main_straight','diagonal_v','circle_straight') and p.get('image_error') is not None and 2.0<=(lp.get('since_anchor_m') or 0)<=7.0:
            pc=p.get('play_comp') or {}
            S[lp['seg']].append((p['image_error'],(p.get('inputs') or {}).get('gz') or 0,pc.get('offset_us') or 0,p.get('t')))
    out=[]
    for k,v in S.items():
        fl=sum(1 for a,b in zip(v,v[1:]) if a[2]*b[2]<0 and b[3]-a[3]<0.3); dur=sum(b[3]-a[3] for a,b in zip(v,v[1:]) if b[3]-a[3]<0.3)
        out.append('%s 誤差 P10/P90 %+.2f/%+.2f gz標準差 %.1f 換邊 %.1f/s'%(k[:8],np.percentile([x[0] for x in v],10),np.percentile([x[0] for x in v],90),np.std([x[1] for x in v]),fl/max(dur,1e-6)))
    print(run[4:8],' | '.join(out))
