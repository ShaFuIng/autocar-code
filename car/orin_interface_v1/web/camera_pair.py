"""New-pitch image geometry and observed inner-edge pairing; no actuator feedback."""
import json
from pathlib import Path
import numpy as np
from standalone_line_path import tracks, row_runs, _runs_table
from road_state_path import IdentityEstimate
from preview_error import PreviewErrorLine, PreviewErrorPID

GEOMETRY=json.loads(Path(__file__).with_name('camera_pair_geometry.json').read_text(encoding='utf8'))
def width(y):
    l,r=GEOMETRY['fits']['left'],GEOMETRY['fits']['right']
    return max(20.,(r[0]-l[0])*y+r[1]-l[1])

# pathfast-r1: the estimator calls inner_tracks twice per frame on the same mask object (LineEstimate.step, then
# StraightLineEstimate.step). The second call returns fresh copies of the first result (the first result is mutated by its
# caller). Reused only for the very same array object with unchanged content (shape, dtype, values).
_IT_CACHE=[None]
def inner_tracks(mask):
    c=_IT_CACHE[0]
    if c is not None and c[0] is mask and c[1].shape==mask.shape and c[1].dtype==mask.dtype and np.array_equal(c[1],mask):
        return [dict(tr,pts=dict(tr['pts'])) for tr in c[2]]
    output=_inner_tracks(mask)
    _IT_CACHE[0]=(mask,mask.copy(),[dict(tr,pts=dict(tr['pts'])) for tr in output])
    return output

def _inner_tracks(mask):
    table=_runs_table(mask)
    output=tracks(mask,table)
    for tr in output:
        side=tr['side']
        if side=='X':side='L' if tr['slope']<0 else 'R'
        if side=='?':
            x=tr['pts'][tr['ymax']];side='L' if x<250 else 'R' if x>390 else '?'
        if side not in ('L','R'):continue
        for y,x in list(tr['pts'].items()):
            runs=table[y] if y in table else row_runs(mask,y)
            center,w=min(runs,key=lambda v:abs(v[0]-x))
            tr['pts'][y]=center+(1 if side=='L' else -1)*(w-1)/2
    return output

class CameraPairEstimate(IdentityEstimate):
    width=staticmethod(width)
    get_tracks=staticmethod(inner_tracks)

    def _pair(self,trs):
        candidates=[];rejections=[]
        for li,l in enumerate(trs):
            if l['side']!='L':continue
            for ri,r in enumerate(trs):
                if r['side']!='R':continue
                ys=sorted(set(l['pts'])&set(r['pts']))
                if len(ys)<7 or ys[-1]-ys[0]<24:
                    rejections.append(dict(left=li,right=ri,reason='SHORT_SHARED_SUPPORT'));continue
                ratios=np.array([(r['pts'][y]-l['pts'][y])/width(y) for y in ys])
                median=float(np.median(ratios));spread=float(np.percentile(ratios,90)-np.percentile(ratios,10))
                # An experimental shape gate, not a safety threshold. Widening
                # forks cannot pass merely because both lines are visible.
                reason=('CROSSING' if np.any(ratios<=0) else 'WIDTH_PROFILE' if not .65<=median<=1.45 else 'DIVERGING_PROFILE' if spread>.20 else None)
                detail=dict(left=li,right=ri,overlap=[ys[0],ys[-1]],ratio=median,spread=spread)
                if reason:rejections.append(dict(detail,reason=reason));continue
                score=abs(median-1)+spread
                candidates.append((score,l,r,detail))
        self.pair_diagnostic=dict(geometry=GEOMETRY['revision'],edge='inner',rejected=rejections,accepted=len(candidates))
        if candidates:
            candidates.sort(key=lambda c:c[0]);score,l,r,detail=candidates[0]
            # Geometry chooses only when distinguishable. History can break a
            # close tie, but cannot veto a uniquely supported current pair.
            close=[c for c in candidates if c[0]<=score+.05]
            def history(c):
                differences=[abs(tr['pts'][y]-self.last[side][y]) for side,tr in [('L',c[1]),('R',c[2])] if self.last[side] for y in tr['pts'] if y in self.last[side]]
                return float(np.median(differences)) if differences else 1e9
            if len(close)>1:score,l,r,detail=min(close,key=history)
            self.pair_diagnostic.update(selected=detail,reason='CURRENT_PAIR')
            return l,r
        l,r=super()._pair(trs)
        # Do not re-admit a pair just rejected by the shape gate through the
        # legacy broad width check. Keep one observed side as a fallback.
        if l and r:
            if l['ymax']-l['ymin']>=r['ymax']-r['ymin']:r=None
            else:l=None
        self.pair_diagnostic.update(selected=None,reason='SINGLE_SIDE_NO_SUPPORTED_PAIR')
        return l,r

    def step(self,mask,t):
        result=super().step(mask,t)
        result['pairing']=self.pair_diagnostic
        return result

class CameraPairLine(PreviewErrorLine):
    algorithm='camera-pair-v1'
    def set_row(self,row):
        super().set_row(row)
        self.estimate=CameraPairEstimate()

class CameraPairPID(PreviewErrorPID):
    width=staticmethod(width)
    def __init__(self,parameters):
        super().__init__(parameters)
        self.policy.update(revision='camera-pair-r1',geometry=GEOMETRY['revision'])
