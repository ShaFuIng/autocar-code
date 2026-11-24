import json
def load(run):
    Q=[]
    for l in open(run+'/pid.jsonl',encoding='utf8'):
        if '"kind": "sample"' not in l[:40]:continue
        d=json.loads(l);p=d['proposal']
        if 't' in p:
            v=d.get('vision') or {};p['_v']=v;Q.append(p)
    lap=0;last=None
    for p in Q:
        s=(p.get('lap_pos') or {}).get('seg') or last
        if s=='curve1' and last!='curve1':lap+=1
        p['_lap']=lap;p['_s']=s;last=s
    return Q
