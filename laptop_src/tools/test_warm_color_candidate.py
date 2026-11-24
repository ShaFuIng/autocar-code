import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"web"))
from warm_color_candidate import WarmColor
warm=np.full((360,640,3),[130,180,230],dtype=np.uint8)
neutral=np.full_like(warm,200)
f=WarmColor();old=np.ones(3)
for t in [0,.1,.2,.3]:
 _,d=f.step(warm,t);assert np.max(abs(np.array(d['gains'])-old))<=.06500001;old=np.array(d['gains'])
_,d=f.step(neutral,.4);assert d['held'] and d['active']
for i in range(5,30):_,d=f.step(neutral,i/10)
assert not d['active']
f.step(warm,3.0);_,d=f.step(neutral,5.0);assert not d['active'] and not d['held']
print('gain rate, short color hold, release and discontinuity reset passed')
