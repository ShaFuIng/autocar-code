import sys,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]/'laptop_workbench';sys.path.insert(0,str(ROOT));from orin_bridge import Bridge
out=ROOT/'data/experiments/field-recordings-20261024';b=Bridge(ROOT/'data');rows=json.loads((out/'catalogue.json').read_text(encoding='utf8'));done=[];errors=[]
for i,r in enumerate(rows):
 print(f"SYNC {i+1}/{len(rows)} {r['kind']} {r['id']}",flush=True)
 try:
  b.sync_one(r);done.append(dict(kind=r['kind'],id=r['id'],folder=str(b.local_folder(r['kind'],r['id']).resolve()),bytes=r['bytes']))
 except Exception as e:errors.append(dict(id=r['id'],error=str(e)));print('ERROR',str(e),flush=True)
 (out/'sync-result.json').write_text(json.dumps(dict(completed=done,errors=errors),ensure_ascii=False,indent=2),encoding='utf8')
print('DONE',len(done),'ERRORS',errors,flush=True)
