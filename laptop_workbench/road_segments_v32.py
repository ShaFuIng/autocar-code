"""Offline road evidence fusion; image coordinates, never a vehicle trajectory.

Memory confidence is heuristic: original age, evidence type and accumulated
alignment residual. Inferred samples never renew their original evidence time.
"""
import math
import cv2
import numpy as np
from entrance_paths_v2 import traces, match_edge, local_offset
from temporal_lane import estimate_motion, transform
from ground_calibration import project

ALGORITHMS=('road-entrance-v3.2',)

def confidence(node,t):
    return float(node['base']*math.exp(-max(0,t-node['observed_at'])/2.-node['alignment_error']/12.))

def split_nodes(nodes):
    out=[]
    for node in sorted(nodes,key=lambda a:a['y']):
        if not out or node['y']-out[-1][-1]['y']>12 or abs(node['x']-out[-1][-1]['x'])>50:
            out.append([])
        out[-1].append(node)
    return out

def render_segments(nodes):
    result=[]
    for chain in split_nodes(nodes):
        group=[]
        for node in chain:
            if group and group[-1]['source']!=node['source']:
                result.append(dict(source=group[-1]['source'],points=[[n['x']*2,n['y']*2] for n in group+[node]],
                    confidence=min(n['confidence'] for n in group),age_s=max(n['age_s'] for n in group)))
                group=[]
            group.append(node)
        if group:result.append(dict(source=group[-1]['source'],points=[[n['x']*2,n['y']*2] for n in group],
            confidence=min(n['confidence'] for n in group),age_s=max(n['age_s'] for n in group)))
    return result

def resample_memory(memory,t):
    """Interpolate warped coordinates, not nearest-row snapping (causes sawteeth)."""
    prior={}
    for chain in split_nodes(memory):
        if len(chain)<2:continue
        for y in range(int(math.ceil((chain[0]['y']-114)/4)*4+114),int(chain[-1]['y'])+1,4):
            for a,b in zip(chain,chain[1:]):
                if a['y']<=y<=b['y'] and b['y']>a['y']:
                    f=(y-a['y'])/(b['y']-a['y'])
                    node=dict(x=a['x']+(b['x']-a['x'])*f,y=float(y),source='history',
                        observed_at=min(a['observed_at'],b['observed_at']),base=min(a['base'],b['base']),
                        alignment_error=max(a['alignment_error'],b['alignment_error']))
                    node.update(confidence=confidence(node,t),age_s=max(0,t-node['observed_at']))
                    key=int(round((y-114)/4))*4+114
                    # Preserve the actual interpolation row; consumers find nearby rows.
                    if key not in prior or node['confidence']>prior[key]['confidence']:prior[key]=node
                    break
    return prior

class RoadPath:
    def __init__(self,algorithm,calibration=None,row=.6):
        self.algorithm='road-memory-v3.1';self.pending_pair=None;self.pending_count=0;self.calibration=calibration;self.row=row
        self.H=np.asarray(calibration['matrix']) if calibration else None
        self.memory_gray=None;self.gray=None;self.t=None;self.edges={};self.width=None;self.width_at=None;self.memory=[];self.mode=None

    def step(self,image,mask,t):
        gray=cv2.cvtColor(cv2.resize(image,(640,360)),cv2.COLOR_BGR2GRAY)
        motion={'valid':False,'reason':'IMAGE_ONLY'};rejections=[];aligned=True;reacquired=[]
        if self.algorithm=='road-memory-v3.1':
            H,motion=estimate_motion(self.memory_gray,gray) if self.t is not None and t>self.t else (None,{'valid':False,'reason':'INITIAL'})
            if H is None:
                if self.memory:rejections.append({'reason':'HISTORY_ALIGNMENT_UNAVAILABLE','points':len(self.memory)})
                aligned=False
            elif self.memory:
                warped=transform([[n['x'],n['y']] for n in self.memory],H)
                for n,(x,y) in zip(self.memory,warped):
                    n.update(x=float(x),y=float(y),alignment_error=n['alignment_error']+float(motion.get('median_error_px',0)))
                self.memory=[n for n in self.memory if np.isfinite([n['x'],n['y']]).all() and -640<n['x']<1280 and -360<n['y']<720 and confidence(n,t)>.01]
        if self.t is not None and t<=self.t:self.edges={};self.width=None;self.memory=[]
        curves=traces(mask);pairs=[]
        # Start at the actual near entrance. Far-only tracks remain diagnostic.
        rejected_pairs=[]
        anchor=270
        def entrance_edge(edge):
            # Stop before a strong corner changes boundary identity. Work upward.
            a=edge.copy()
            for k in range(len(a)-5,4,-1):
                near=(a[k+4,0]-a[k,0])/(a[k+4,1]-a[k,1])
                far=(a[k,0]-a[k-4,0])/(a[k,1]-a[k-4,1])
                if abs(near-far)>2.0:
                    return a[k:]
            return a

        for i,l in enumerate(curves):
            for j,r in enumerate(curves):
                if i==j:continue
                ys=np.intersect1d(l[:,0],r[:,0])
                if len(ys)<6:continue
                lx=np.interp(ys,l[:,0],l[:,2]);rx=np.interp(ys,r[:,0],r[:,1])
                if np.any(rx-lx<=8) or not lx[-1]<319.5<rx[-1]:continue
                if ys[-1]<anchor:
                    rejected_pairs.append(dict(reason='FAR_PAIR_NOT_CONNECTED_TO_ENTRANCE',left=i,right=j,near_y=float(ys[-1])))
                    continue
                # A pair must agree with aligned history before it can replace it.
                prior_check=resample_memory(self.memory,t) if aligned else {}
                deltas=[abs((float(np.interp(y,ys,lx))+float(np.interp(y,ys,rx)))/2-h['x']) for y,h in prior_check.items() if ys[0]<=y<=ys[-1] and y>=230 and h['confidence']>.15]
                if deltas and np.median(deltas)>40:
                    rejected_pairs.append(dict(reason='PAIR_CONFLICT_WITH_TRUSTED_HISTORY',left=i,right=j,delta=float(np.median(deltas))))
                    continue
                pairs.append((ys[-1]/360+np.ptp(ys)/360,i,j,ys,lx,rx))
        edges={};pair=max(pairs,key=lambda p:p[0]) if pairs else None
        if pair:
            _,i,j,ys,lx,rx=pair
            edges={'left':np.column_stack([curves[i][:,2],curves[i][:,0]]),'right':np.column_stack([curves[j][:,1],curves[j][:,0]])}
            profile=np.column_stack([ys,rx-lx])
            # A shrinking overlap does not erase the earlier broad width support.
            if self.width is None or np.ptp(ys)>=.7*np.ptp(self.width[:,0]):self.width=profile;self.width_at=t
        else:
            for side,col in [('left',2),('right',1)]:
                options=[]
                for tr in curves:
                    xy=entrance_edge(np.column_stack([tr[:,col],tr[:,0]]))
                    if len(xy)<6 or xy[-1,1]<anchor:continue
                    old=self.edges.get(side)
                    match=match_edge(xy,[old]) if old is not None else None
                    near_ok=xy[-1,0]<319.5 if side=='left' else xy[-1,0]>319.5
                    if near_ok and match and match[0]<60:options.append((2-match[0]/60+np.ptp(xy[:,1])/360,xy))
                    elif near_ok and self.width is not None and np.ptp(xy[:,1])>=40:
                        # Reacquire from current near-field evidence; no old-coordinate reuse.
                        options.append((np.ptp(xy[:,1])/360,xy))
                        reacquired.append(dict(side=side,match_error_px=match[0] if match else None,reason='CURRENT_ENTRANCE_REACQUIRE'))
                if options:edges[side]=max(options,key=lambda a:a[0])[1]
        rejections.extend(rejected_pairs)
        current={};double_rows=set();metric_rows={}
        if self.H is not None:
            for side,edge in edges.items():
                metric=project(edge,self.H);metric=metric[np.isfinite(metric).all(1)&(metric[:,1]>0)]
                if len(metric)<6:continue
                sections,folds=local_offset(metric,self.calibration['lane_width_cm']/2,side)
                if folds:rejections.append(dict(reason='LOCAL_FOLD_SPLIT',side=side,count=folds))
                spans=[]
                for section in sections:
                    pixels=project(section,np.linalg.inv(self.H));pixels=pixels[np.isfinite(pixels).all(1)]
                    pixels=pixels[np.argsort(pixels[:,1])]
                    if len(pixels)>=2:spans.append(pixels)
                metric_rows[side]=spans
        coeff=np.polyfit(self.width[:,0],self.width[:,1],1) if self.width is not None else None
        def at(edge,y):
            return float(np.interp(y,edge[:,1],edge[:,0])) if edge is not None and edge[0,1]<=y<=edge[-1,1] else None
        for y in range(114,355,4):
            l=at(edges.get('left'),y);r=at(edges.get('right'),y);x=None;source=None;base=0.;origin=t
            if l is not None and r is not None and r-l>8:
                x=(l+r)/2;source='double';base=1.;double_rows.add(y)
            elif l is not None or r is not None:
                side='left' if l is not None else 'right';edge_x=l if l is not None else r
                if self.H is not None:
                    spans=metric_rows.get(side,[])
                    values=[at(span,y) for span in spans];values=[v for v in values if v is not None]
                    if values:x=values[0];source='single_metric';base=.45
                elif coeff is not None:
                    width=float(np.polyval(coeff,y))
                    if 8<width<900:
                        x=edge_x+(1 if side=='left' else -1)*width/2
                        extrap=max(self.width[0,0]-y,0,y-self.width[-1,0])
                        source='single_extrapolated' if extrap else 'single'
                        base=.65*math.exp(-extrap/120);origin=self.width_at
            if x is not None and math.isfinite(x):
                current[y]=dict(x=float(x),y=float(y),source=source,base=base,observed_at=origin,
                    alignment_error=0.,confidence=base,age_s=max(0,t-origin))
        # Smooth only small local steps on the same supported road, never across holes.
        ordered=list(current.values())
        for i in range(1,len(ordered)-1):
            a,b,c=ordered[i-1:i+2]
            if c['y']-a['y']==8 and b['source']!='double' and max(abs(a['x']-b['x']),abs(c['x']-b['x']))<30:
                b['x']=(a['x']+2*b['x']+c['x'])/4
        prior=resample_memory(self.memory,t) if aligned else {}
        nodes=dict(current)
        for y,h in prior.items():
            c=nodes.get(y)
            if c:
                delta=abs(c['x']-h['x'])
                if delta>40:
                    rejections.append(dict(reason='HISTORY_CURRENT_CONFLICT',y=y,delta_px=delta))
                    if y>=230 and h['confidence']>.15:nodes[y]=h
                    continue
                if c['source']!='double' and h['confidence']>.1:
                    # Compatible same-road evidence only; preserve provenance.
                    wc=c['confidence'];wh=h['confidence'];c=dict(c)
                    c.update(x=(c['x']*wc+h['x']*wh)/(wc+wh),source='single_history',
                        history_weight=wh/(wc+wh),age_s=h['age_s'])
                    nodes[y]=c
            elif y>=230 and h['confidence']>.1:nodes[y]=h
        all_nodes=sorted(nodes.values(),key=lambda n:n['y'])
        visible=[n for n in all_nodes if 0<=n['x']<=639.5 and 0<=n['y']<=359.5]
        chains=split_nodes(visible)
        def rank(chain):
            lo,hi=chain[0]['y']*2,chain[-1]['y']*2
            return (lo<=self.row*719<=hi,hi>=self.row*719,len(chain))
        usable=[c for c in chains if len(c)>=6]
        chosen=max(usable,key=rank) if usable else []
        # Keep original timestamps; history/fused results are never relabelled new.
        if self.algorithm=='road-memory-v3.1':
            retained=dict(prior)
            for y,c in current.items():
                old=retained.get(y)
                if old is not None and abs(old['x']-c['x'])>40:
                    continue
                if old is None or c['base']*math.exp(-max(0,t-c['observed_at'])/2)>old['confidence']:
                    retained[y]={k:c[k] for k in ('x','y','source','base','observed_at','alignment_error')}
                elif abs(old['x']-c['x'])>40:retained.pop(y,None)
            # Rebuild an unaligned anchor only after repeated near-entrance pairs.
            proposal={y:c['x'] for y,c in current.items() if c['source']=='double' and y>=230}
            overlap=set(proposal)&set(self.pending_pair or {})
            consistent=len(overlap)>=6 and max(abs(proposal[y]-self.pending_pair[y]) for y in overlap)<20
            self.pending_count=self.pending_count+1 if consistent else (1 if len(proposal)>=6 else 0)
            self.pending_pair=proposal
            rebuild=not aligned and self.pending_count>=3 and bool(pair)
            if rebuild:
                retained={y:{k:c[k] for k in ('x','y','source','base','observed_at','alignment_error')} for y,c in current.items()}
                rejections.append(dict(reason='CONFIRMED_ENTRANCE_ANCHOR_REBUILT',frames=self.pending_count))
            if aligned or not self.memory or rebuild:
                self.memory=list(retained.values());self.memory_gray=gray.copy()
            # Unaligned memory stays anchored to its original image, is invisible,
            # and must be matched to that anchor before it can contribute again.
            self.memory=[n for n in self.memory if confidence(n,t)>.01]
        sources=sorted(set(n['source'] for n in visible));mode='+'.join(sources) or 'no_evidence'
        previous=self.mode;self.mode=mode;self.edges=edges;self.gray=gray;self.t=t
        return dict(path=[[n['x']*2,n['y']*2] for n in chosen],valid=bool(chosen),quality=float(np.mean([n['confidence'] for n in chosen])) if chosen else 0.,
            road_segments=render_segments(all_nodes),history_segments=render_segments(list(prior.values())),
            path_samples=[{k:n[k] for k in ('x','y','source','confidence','age_s','observed_at')} for n in all_nodes],
            mode=mode,transition=f'{previous} → {mode}',reason='NEAR_ENTRANCE_CONSTRAINED_EVIDENCE',
            boundaries=[tr.tolist() for tr in curves],observed_edges=[v.tolist() for v in edges.values()],
            history_path=[],history_points=sum('history' in n['source'] for n in visible),motion=motion,
            rejections=rejections,candidates=reacquired,memory_state=dict(aligned=aligned,stored_points=len(self.memory),usable_points=len(prior),anchor_retained=not aligned and bool(self.memory)),dynamic_sampling=True,hardware_output=False,
            width_support_y=self.width[:,0].tolist() if self.width is not None else [],
            coverage=dict(visible_rows=len(visible),double_rows=len(double_rows),components=len(chains),
                control_span_px=(chosen[-1]['y']-chosen[0]['y'])*2 if chosen else 0.,
                visible_span_px=(visible[-1]['y']-visible[0]['y'])*2 if visible else 0.),
            metric_exploration=self.H is not None,path_semantics='estimated_road_center_not_vehicle_trajectory')
