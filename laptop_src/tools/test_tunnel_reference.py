import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from analyze_tunnel_reference import *
from tunnel_reference_candidate import RoadPath,extend_tape

edge=np.array([[100.,float(y)] for y in range(180,225,4)])
flat=np.full((360,640,3),100,np.uint8)
assert len(extend_tape(flat,edge,'left')[1])==0
stripe=flat.copy();stripe[:,:100]=220
assert len(extend_tape(stripe,edge,'left')[1])>4
assert len(extend_tape(stripe,edge,'right')[1])==0
candidate=RoadPath('x');mask=np.zeros((360,640),bool)
for t in [0,.1,.2,.3]:
 r=candidate.step(stripe,mask,t);assert not r['valid'] and not r['far_recovery']['accepted']
data=json.loads((OUT/'comparison.json').read_text(encoding='utf8'))
for case in ['intersection1','intersection2']:
 assert all(x['baseline']['path']==x['candidate']['path'] and x['baseline']['mode']==x['candidate']['mode'] and x['baseline']['pid']==x['candidate']['pid'] for x in data[case])
params=load_tunnel()[1]
for case,rows in data.items():
 for a,b in zip(rows,rows[1:]):
  p,q=a['candidate']['pid'],b['candidate']['pid']
  if p['valid'] and q['valid']:assert abs(q['steer_us']-p['steer_us'])<=params['steer_rate']*(b['captured']-a['captured'])+1e-6
assert data['tunnel'][-1]['candidate']['valid']
assert not data['tunnel'][-1]['baseline']['valid']
assert data['tunnel'][-2]['candidate']['recovery']['votes']==2
assert not data['tunnel'][-2]['candidate']['recovery']['accepted']
print('PASS: flat/wrong-side/empty-mask rejection, consecutive confirmation, two intersection exact PID parity, steering rate bound')

