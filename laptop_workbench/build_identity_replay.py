"""Replay cached observations only. No network, controller or model loading."""
import json
from pathlib import Path
import cv2,numpy as np
from boundary_follow import BoundaryIdentity,offset_boundary,image_curve

ROOT=Path(__file__).resolve().parent/'data/experiments/bend-cases'
def main():
    audit={}
    for case in ('large','small'):
        folder=ROOT/case
        source=folder/'result.json'
        backup=folder/'result-before-identity.json'
        if not backup.exists():backup.write_bytes(source.read_bytes())
        report=json.loads(backup.read_text(encoding='utf8'))
        tracker=BoundaryIdentity();reasons={};changes=[]
        for f in report['frames']:
            mask=cv2.imdecode(np.fromfile(str(folder/f"{f['frame']:06d}-cache.png"),np.uint8),0)>0
            boundaries,identity=tracker.step(mask,f['t'])
            diagnostics={}
            for side in ('left','right'):
                diag={}
                path=offset_boundary(boundaries[side],side,report['geometry'],diagnostic=diag)
                if not boundaries[side]:diag['reason']=identity[side]['reason']
                diag['identity']=identity[side]
                diagnostics[side]=diag
                key='tracked_'+side
                f['paths'][key]=image_curve(path,report['geometry'])
                reason=side+':'+diag['reason'];reasons[reason]=reasons.get(reason,0)+1
            f['identity_diagnostics']=diagnostics
            f['tracked_boundaries']=boundaries
            if f['frame'] in (100,119,140):
                changes.append(dict(frame=f['frame'],t=f['t'],diagnostics=diagnostics,
                    old={s:len(f['paths'][s]) for s in ('left','right')},
                    new={s:len(f['paths']['tracked_'+s]) for s in ('left','right')}))
        report['availability']={k:sum(bool(f['paths'][k]) for f in report['frames']) for k in report['frames'][0]['paths']}
        report['identity_version']='bounded-current-observation-v1'
        source.write_text(json.dumps(report,ensure_ascii=False,allow_nan=False),encoding='utf8')
        audit[case]=dict(availability=report['availability'],reasons=reasons,examples=changes)
        print(case,json.dumps(audit[case],ensure_ascii=False),flush=True)
    (ROOT/'identity-audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf8')
if __name__=='__main__':main()
