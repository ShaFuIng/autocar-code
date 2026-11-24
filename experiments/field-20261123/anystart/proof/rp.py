import json,sys,copy
SNAP=r'D:/OneDrive/Desktop/自走車/Orin_Interface_v1/laptop_workbench/data/experiments/orin-web-snapshot-20261122c'
sys.path.insert(0,SNAP)
import lap_pos
BASE=lap_pos.load_config(SNAP+'/lap_pos.json')
def load(r): return [json.loads(l) for l in open(r+'.jsonl')]
def run(fr,cfg,i0=0,i1=None):
    lp=lap_pos.LapPos(copy.deepcopy(cfg)); out=[]
    for f in fr[i0:i1]:
        st=lp.update(f['t'],f['gz'],f['pr'],f['enc'])
        out.append(st)
    return lp,out
if __name__=='__main__':
    for r in sys.argv[1:]:
        fr=load(r); lp,out=run(fr,BASE)
        ev=[(e['name'],e.get('lap'),e.get('d_cross'),e.get('s_err')) for e in lp.events]
        # compare with logged
        mism=sum(1 for f,o in zip(fr,out) if f['an']!=o['anchor'] or f['lap']!=o['lap'])
        print(r,len(fr),'mismatch anchor/lap',mism)
        print([e for e in lp.events if e['name'] in ('C1s','SKIPPED','LOST','REACQUIRED')])
