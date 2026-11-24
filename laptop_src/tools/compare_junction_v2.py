"""Reuse exact frozen images/masks and A/B. C-only offline experiment."""
import copy,json,sys,time,uuid,shutil
from pathlib import Path
import cv2
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'laptop_workbench'))
from analysis_jobs import read,write
from shadow_worker import sha,picture,pid_rows
from junction_tracked import JunctionTracked,PARAMS

def main():
    cv2.setNumThreads(2);data=ROOT/'laptop_workbench/data';exp=data/'experiments/junction-cases'
    manifest=read(exp/'cases.json'); report=dict(parameters=PARAMS,algorithm_sha256=sha(ROOT/'laptop_workbench/junction_tracked.py'),cases=[],inferred=0,hardware_output=False)
    for c in manifest['cases']:
        old=data/'analysis'/c['job']; d=read(old/'result.json');before=sha(old/'result.json')
        uid=uuid.uuid4().hex;folder=data/'analysis'/uid;folder.mkdir();(folder/'raw').mkdir();(folder/'C').mkdir()
        tracker=JunctionTracked();rows=[]
        for i,f in enumerate(d['frames']):
            raw=old/'raw'/f'{i:06d}.jpg'; mask=old/'A'/f'{i:06d}.png'
            assert sha(raw)==f['sha256'];assert sha(mask)==sha(old/'B'/mask.name)
            im=picture(raw);m=picture(mask)[:,:,0]>0;tick=time.perf_counter()
            r=tracker.step(im,m,f['capture_s'],d['models']['A'][i]['A']);r['path_ms']=(time.perf_counter()-tick)*1000;rows.append(r)
            shutil.copyfile(raw,folder/'raw'/raw.name);shutil.copyfile(mask,folder/'C'/mask.name)
        g=copy.deepcopy(d['spec']['groups'][1]);g.update(id='C',label='路口追蹤與接回 v2',algorithm='junction-tracked-v2',pid_enabled=False)
        models,summary=pid_rows(rows,g['parameters'],False)
        result=copy.deepcopy(d);result.update(id=uid,models={'C':models},summaries={'C':summary},baseline_job=c['job'],spec=dict(d['spec'],groups=[g]),cache=dict(inferred=0,mask_reused=len(rows),path_reused=0),candidate_parameters=PARAMS)
        write(folder/'result.json',result);write(folder/'job.json',dict(id=uid,kind='shadow',source=d['source'],title=c['title']+'｜C路口追蹤v2｜原A/B保留',spec=result['spec'],models=['v4-640'],created=time.time(),state='completed',total=len(rows),execution='筆電離線；原遮罩沿用；PID關閉'))
        write(folder/'status.json',dict(state='completed',completed=len(rows),total=len(rows)))
        switches=[dict(index=i,player_s=f['player_s'],state=r['junction']['state'],reason=r['junction']['reason']) for i,(f,r) in enumerate(zip(d['frames'],rows)) if i==0 or r['junction']['state']!=rows[i-1]['junction']['state']]
        c.update(candidate_job=uid,candidate_algorithm='junction-tracked-v2');assert sha(old/'result.json')==before
        report['cases'].append(dict(id=c['id'],job=uid,baseline_job=c['job'],baseline_sha256=before,frames=len(rows),valid=sum(r['valid'] for r in rows),switches=switches,rejected=sum(len(r['junction']['rejected']) for r in rows)))
        print(json.dumps(report['cases'][-1],ensure_ascii=False),flush=True)
    if (exp/'v2-report.json').exists():shutil.copyfile(exp/'v2-report.json',exp/('v2-report-history-'+uuid.uuid4().hex[:8]+'.json'))
    write(exp/'v2-report.json',report);write(exp/'cases.json',manifest)
if __name__=='__main__':main()
