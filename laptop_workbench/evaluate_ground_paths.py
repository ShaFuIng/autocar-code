"""Offline single-frame consistency check. Never exports a controller command."""
import json
from pathlib import Path
import cv2,numpy as np
from temporal_lane import thin,graph_path,row_curve
from ground_calibration import project
ROOT=Path(__file__).resolve().parent

def normal_offset(curve,distance,side):
    a=np.asarray(curve,float)
    tangent=np.gradient(a,axis=0)
    length=np.linalg.norm(tangent,axis=1)
    if np.any(length<1e-8):raise ValueError('Degenerate tangent')
    normal=np.column_stack([tangent[:,1],-tangent[:,0]])/length[:,None]
    result=a+normal*distance*(1 if side=='left' else -1)
    if np.any(np.sum(np.diff(result,axis=0)*np.diff(a,axis=0),axis=1)<=0):
        raise ValueError('Offset foldback')
    return result

def main():
    base=ROOT/'data/experiments/ground-calibration'
    folder=base/json.loads((base/'latest.json').read_text())['folder']
    calibration=json.loads((folder/'result.json').read_text(encoding='utf8'))
    snapshot=ROOT/'data/experiments/camera-measure'/calibration['source_snapshot']['id']
    im=cv2.imdecode(np.fromfile(str(snapshot/'raw.jpg'),np.uint8),1)
    mask=cv2.imdecode(np.fromfile(str(snapshot/'mask.png'),np.uint8),-1)[:,:,3]>0
    sk=thin(mask);sk[:100]=0
    H=np.asarray(calibration['matrix']);inverse=np.linalg.inv(H)
    observations={};fits={};diagnostics={}
    # Side graph observations first; locate inner edge in the SAME mask run.
    for side in ('left','right'):
        rows=row_curve(graph_path(sk,side,np.empty((0,2))))
        points=[]
        for y,x in rows.items():
            y=int(y);x=int(round(x))
            if not mask[y,x]:continue
            end=x;step=1 if side=='left' else -1
            while 0<=end+step<640 and mask[y,end+step]:end+=step
            p=project([[end,y]],H)[0]
            if 50<=p[1]<=120:points.append(p)
        a=np.asarray(points)
        if len(a)<8:raise ValueError(side+' insufficient visible inner boundary')
        a=a[np.argsort(a[:,1])]
        # This diagnostic fits a smooth current straight segment; not temporal tracking.
        coefficients=np.polyfit(a[:,1],a[:,0],2)
        residual=np.polyval(coefficients,a[:,1])-a[:,0]
        observations[side]=a.tolist();fits[side]=coefficients
        diagnostics[side]=dict(count=len(a),fit_rms_cm=float(np.sqrt(np.mean(residual**2))),
            observed_z_range=[float(a[:,1].min()),float(a[:,1].max())])
    low=max(diagnostics[s]['observed_z_range'][0] for s in diagnostics)
    high=min(diagnostics[s]['observed_z_range'][1] for s in diagnostics)
    z=np.linspace(low,high,100)
    edges={s:np.column_stack([np.polyval(fits[s],z),z]) for s in fits}
    paths={s:normal_offset(edges[s],23,s) for s in edges}
    paths['double']=(edges['left']+edges['right'])/2
    # Compare only overlapping output z, no extrapolation.
    lo=max(p[:,1].min() for p in paths.values());hi=min(p[:,1].max() for p in paths.values())
    zz=np.linspace(lo,hi,100)
    xx={s:np.interp(zz,p[:,1],p[:,0]) for s,p in paths.items()}
    disagreement=abs(xx['left']-xx['right'])
    width=edges['right'][:,0]-edges['left'][:,0]
    inside={s:bool(np.all((xx[s]>=np.interp(zz,z,edges['left'][:,0]))&
                                    (xx[s]<=np.interp(zz,z,edges['right'][:,0])))) for s in paths}
    metrics=dict(compared_z_cm=[float(lo),float(hi)],single_path_separation_mean_cm=float(disagreement.mean()),
        single_path_separation_max_cm=float(disagreement.max()),
        projected_inner_width_range_cm=[float(width.min()),float(width.max())],
        path_centres_inside_observed_corridor=inside)
    overlay=im.copy();overlay[mask]=(overlay[mask]*.6+np.array([220,40,220])*.4).astype(np.uint8)
    colors={'left':(0,150,255),'right':(255,210,0),'double':(40,240,40)}
    for side,points in paths.items():
        pixels=project(points,inverse).round().astype(np.int32)
        cv2.polylines(overlay,[pixels],False,colors[side],2)
    cv2.imencode('.png',overlay)[1].tofile(str(folder/'paths-overlay.png'))
    top=np.full((640,480,3),24,np.uint8)
    T=np.array([[4,0,240],[0,-4,640],[0,0,1]],float)
    for zzline in (50,80,100,120):
        yy=int(640-4*zzline);cv2.line(top,(40,yy),(440,yy),(70,70,70),1)
        cv2.putText(top,f'{zzline} cm',(5,yy-4),0,.45,(220,220,220),1)
    for side,edge in edges.items():cv2.polylines(top,[project(edge,T).round().astype(np.int32)],False,(220,80,220),2)
    for side,path in paths.items():cv2.polylines(top,[project(path,T).round().astype(np.int32)],False,colors[side],2)
    cv2.imencode('.png',top)[1].tofile(str(folder/'paths-top.png'))
    result=dict(calibration_revision=calibration['source_revision'],snapshot=calibration['source_snapshot']['id'],
        observations=observations,paths={s:p.tolist() for s,p in paths.items()},diagnostics=diagnostics,metrics=metrics,
        hardware_output=False,scope='同一張當前直線快照之幾何一致性，不是獨立準確率／時間穩定性／彎道驗收',
        offset_reference='V4遮罩的白線內緣，向內法線23cm；不是從白線中心偏25.25cm',
        limitations=['投影來源端點與本張畫面相關，結果可能樂觀','兩個單線結果都依賴同一份由雙側量測建立的投影',
        '50～120cm之外不外插；沒有車輪掃掠或控制驗證','仍保留100cm獨立驗證約+3.6cm偏差'])
    (folder/'paths-result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(metrics,ensure_ascii=False))
if __name__=='__main__':main()
