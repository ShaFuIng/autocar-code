import sys,json,copy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'web/k_lane_runtime'),str(ROOT/'web')]
from junction_approach_candidate import ApproachGate
OUT=ROOT/'laptop_workbench/data/experiments/k-approach'
def observation(t,y,compatible=True,supported=True):
 return dict(capture_s=t,zones=[dict(name='middle',rows=list(range(y-8,y+1,4)))],a_supported=supported,owned_components=2,
             opening=True,near_opening=False,reference_compatible=compatible)
g=ApproachGate()
for i,y in enumerate([214,218,222,226,230]):
 r=g.step(observation(i*.1,y,False),True,False,True)
assert r['approach_retained'] and g.state=='SUSPECT','Mismatch must block handoff'
for i in range(5,8):
 d=observation(i*.1,230);d['opening']=False;d['zones'][0]['rows']=[]
 r=g.step(d,True,False,True)
assert g.state=='CROSS','Confirmed approach must survive loss of near branches with fresh entrance match'
g=ApproachGate()
for i,y in enumerate([214,218,222,226,230]):g.step(observation(i*.1,y,False),True,False,True)
d=observation(1.6,230);d['opening']=False;d['zones'][0]['rows']=[]
r=g.step(d,True,False,True);assert not r['approach_retained'] and g.state!='CROSS','Expired evidence must not hand off'
for kind in ['single','static','no_reference']:
 g=ApproachGate()
 for i in range(10):
  d=observation(i*.1,214 if kind=='static' else 214+i*4,supported=kind!='single')
  g.step(d,kind!='no_reference',False,True)
 assert g.state!='CROSS',kind
rows=json.loads((OUT/'latest.json').read_text())
for x in rows:
 if x['candidate']['reason']=='APPROACH_SEQUENCE_AND_CURRENT_ENTRANCE':
  assert x['candidate']['gate']['reference_compatible'] and x['candidate']['gate']['entry_check_passed']
 assert not x['candidate_early_allowed'] or x['candidate']['source']=='A'
print('PASS: mismatch blocked, retained approach + current entry succeeds, expiry, single-line/static/no-reference reject, early-A invariant')
