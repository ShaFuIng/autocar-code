"""Offline ablation over frozen A evidence; no slope classifier or control entry.

All thresholds are experiment parameters in 640x360 pixels / source seconds.
The single-side branch cannot establish a new center or measured lane width.
"""
import copy
import numpy as np
from road_segments_v32 import split_nodes, render_segments


class GeometryMismatch:
    def __init__(self, threshold=16., confirm=3, stale_s=2., near_y=270):
        self.threshold=threshold; self.confirm=confirm
        self.stale_s=stale_s; self.near_y=near_y; self.votes=0

    def step(self, original):
        a=copy.deepcopy(original); a.pop('k',None)
        samples=original.get('path_samples',[])
        curves=[np.asarray(c,float) for c in original.get('boundaries',[])]
        owned=[np.asarray(e,float) for e in original.get('observed_edges',[])]
        pairs=[]
        # Require one of the pair's inner edges to match the selected entrance
        # boundary. The other edge must actually be visible at the same rows.
        for li,l in enumerate(curves):
            for ri,r in enumerate(curves):
                if li==ri: continue
                ys=np.intersect1d(l[:,0],r[:,0])
                if len(ys)<6: continue
                lx=np.interp(ys,l[:,0],l[:,2]); rx=np.interp(ys,r[:,0],r[:,1])
                if np.any(rx-lx<16): continue
                matches=[]
                for edge in owned:
                    select=(ys>=edge[0,1])&(ys<=edge[-1,1])
                    if select.sum()<6: continue
                    ex=np.interp(ys[select],edge[:,1],edge[:,0])
                    matches.append(min(float(np.median(abs(lx[select]-ex))),float(np.median(abs(rx[select]-ex)))))
                if not matches or min(matches)>6: continue
                # Compare only actual common rows; never extrapolate the pair.
                baseline={n['y']:n['x'] for n in samples}
                shared=[j for j,y in enumerate(ys) if y in baseline]
                if len(shared)<6: continue
                cx=(lx+rx)/2
                if np.mean([lx[j]<baseline[ys[j]]<rx[j] for j in shared])<.8: continue
                residual=float(np.median([abs(cx[j]-baseline[ys[j]]) for j in shared]))
                pairs.append((ys[-1],len(ys),ys,cx,lx,rx,residual))
        pair=max(pairs,key=lambda p:(p[0],p[1])) if pairs else None
        mismatch=bool(pair is not None and pair[-1]>self.threshold)
        self.votes=self.votes+1 if mismatch else 0
        confirmed=self.votes>=self.confirm
        ages=[n['age_s'] for n in samples if n['source']!='double']
        age=max(ages,default=0.)
        unsupported=pair is None and not any(n['source']=='double' for n in samples)
        state='BASELINE'; nodes=copy.deepcopy(samples)
        if confirmed:
            state='CURRENT_PAIR_MISMATCH'
            _,_,ys,cx,lx,rx,_=pair
            nodes=[dict(x=float(x),y=float(y),source='double',confidence=.7,age_s=0.,observed_at=original['capture_s']) for x,y in zip(cx,ys)]
            a['observed_edges']=[np.column_stack([lx,ys]).tolist(),np.column_stack([rx,ys]).tolist()]
        elif unsupported and age>self.stale_s:
            state='SINGLE_WIDTH_UNVERIFIABLE'
            # Keep the old width explicitly provisional, only where a current
            # boundary supports the near field. No synthetic recentering.
            nodes=[n for n in nodes if n['y']>=self.near_y and n['source'] not in ('history','single_extrapolated') and any(e[0,1]<=n['y']<=e[-1,1] for e in owned)]
            for n in nodes: n['confidence']=min(n['confidence'],.25)
        chains=[c for c in split_nodes(nodes) if len(c)>=6]
        chosen=max(chains,key=lambda c:(c[-1]['y'],len(c))) if chains else []
        a.update(path=[[n['x']*2,n['y']*2] for n in chosen],valid=bool(chosen),
                 road_segments=render_segments(nodes),path_samples=nodes,
                 history_segments=[] if state!='BASELINE' else a.get('history_segments',[]),
                 quality=float(np.mean([n['confidence'] for n in chosen])) if chosen else 0.,
                 mode=state,reason=state,hardware_output=False,
                 geometry=dict(state=state,current_pair=pair is not None,residual_px=pair[-1] if pair else None,
                               mismatch_votes=self.votes,width_age_s=age,threshold_px=self.threshold,
                               confirm_frames=self.confirm,stale_s=self.stale_s,near_y=self.near_y,
                               slope_detected=None,scope='frozen_evidence_output_ablation'),
                 coverage=dict(visible_rows=len(nodes),double_rows=sum(n['source']=='double' for n in nodes),components=len(chains),
                               control_span_px=2*(chosen[-1]['y']-chosen[0]['y']) if chosen else 0.,
                               visible_span_px=2*(nodes[-1]['y']-nodes[0]['y']) if nodes else 0.))
        # The baseline branch must be an exact path passthrough.
        if state=='BASELINE':
            for key in ('path','valid','road_segments','path_samples','coverage','quality'):
                a[key]=copy.deepcopy(original[key])
        return a
