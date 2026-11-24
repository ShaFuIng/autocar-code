"""Offline geometry guard. Coordinates: edges/samples 640x360, output path 1280x720."""
import copy, math
import numpy as np

def geometry_guard(a):
 r=copy.deepcopy(a);cuts=[]
 # Compare adjacent boundary directions. Keep the connected near-side portion.
 for ei,edge in enumerate(a.get('observed_edges',[])):
  e=np.asarray(edge,float)
  if len(e)<4:continue
  for i in range(1,len(e)-2):
   far=e[i]-e[i-1];near=e[min(i+3,len(e)-1)]-e[i]
   if far[1]<=0 or near[1]<=0:continue
   angle=abs(math.degrees(math.atan2(far[0],far[1])-math.atan2(near[0],near[1])))
   residual=abs(far[0]-near[0]/near[1]*far[1])
   if angle>55 and residual>12:
    cuts.append(dict(reason='BOUNDARY_DIRECTION_BREAK',edge=ei,y=float(e[i,1]),angle_deg=angle,residual_px=residual))
 samples=a.get('path_samples',[])
 for i in range(1,len(samples)):
  p,q=samples[i-1],samples[i];dy=q['y']-p['y'];dx=abs(q['x']-p['x'])
  if 0<=p['y']<q['y']<=334 and p['source']!=q['source'] and 0<dy<=8 and dx>max(20,dy*4):
   cuts.append(dict(reason='CENTER_JOIN_DISCONTINUITY',y=q['y'],dx_px=dx,dy_px=dy,sources=[p['source'],q['source']]))
 joins=[c for c in cuts if c["reason"]=="CENTER_JOIN_DISCONTINUITY"]
 # A sharp boundary alone can be a real bend. Reject it only when the
 # center also has a discontinuous join in the same local region.
 cuts=[c for c in cuts if c["reason"]!="BOUNDARY_DIRECTION_BREAK" or any(abs(c["y"]-j["y"])<=24 for j in joins)]
 if not cuts:return r
 cutoff=max(c['y'] for c in cuts)
 # No interpolation across the removed region, nor substitution of unaligned history.
 r['path']=[p for p in a['path'] if p[1]>=cutoff*2]
 r['path_samples']=[p for p in samples if p['y']>=cutoff]
 r['observed_edges']=[ [p for p in e if p[1]>=cutoff] for e in a.get('observed_edges',[]) ];r['observed_edges']=[e for e in r['observed_edges'] if len(e)>=2]
 for field in ['road_segments','history_segments']:
  r[field]=[]
  for s in a.get(field,[]):
   t=copy.deepcopy(s);t['points']=[p for p in s['points'] if p[1]>=cutoff*2]
   if len(t['points'])>=2:r[field].append(t)
 r['history_path']=[p for p in a.get('history_path',[]) if p[1]>=cutoff*2]
 r['valid']=len(r['path'])>=2
 r['geometry_guard']=dict(cutoff_y=cutoff,rejections=cuts,removed_points=len(a['path'])-len(r['path']))
 r.setdefault('rejections',[]).extend(cuts)
 return r
