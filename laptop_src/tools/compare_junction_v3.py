"""Reuse exact frozen images/masks and A/B. D-only offline experiment."""
import copy,json,sys,time,uuid,shutil
from pathlib import Path
import cv2
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'laptop_workbench'))
from analysis_jobs import read,write
from shadow_worker import sha,picture,pid_rows
from junction_anchored import JunctionAnchored,PARAMS

def main():
    cv2.setNumThreads(2);data=ROOT/'laptop_workbench/data';exp=data/'experiments/junction-cases'
    manifest=read(exp/'cases.json'); report=dict(parameters=PARAMS,algorithm_sha256=sha(ROOT/'laptop_workbench/junction_anchored.py'),cases=[],inferred=0,hardware_output=False)
    for c in manifest['cases']:
        old=data/'analysis'/c['job']; d=read(old/'result.json');before=sha(old/'result.json')
        uid=uuid.uuid4().hex;folder=data/'analysis'/uid;folder.mkdir();(folder/'raw').mkdir();(folder/'D').mkdir()
        tracker=JunctionAnchored();rows=[]
        for i,f in enumerate(d['frames']):
            raw=old/'raw'/f'{i:06d}.jpg'; mask=old/'A'/f'{i:06d}.png'
            assert sha(raw)==f['sha256'];assert sha(mask)==sha(old/'B'/mask.name)
            im=picture(raw);m=picture(mask)[:,:,0]>0;tick=time.perf_counter()
            r=tracker.step(im,m,f['capture_s'],d['models']['A'][i]['A']);r['path_ms']=(time.perf_counter()-tick)*1000;rows.append(r)
            shutil.copyfile(raw,folder/'raw'/raw.name);shutil.copyfile(mask,folder/'D'/mask.name)
        g=copy.deepcopy(d['spec']['groups'][1]);g.update(id='D',label='邊界出口與參考修正 v3',algorithm='junction-anchored-v3',pid_enabled=False)
        models,summary=pid_rows(rows,g['parameters'],False)
        result=copy.deepcopy(d);result.update(id=uid,models={'D':models},summaries={'D':summary},baseline_job=c['job'],spec=dict(d['spec'],groups=[g]),cache=dict(inferred=0,mask_reused=len(rows),path_reused=0),candidate_parameters=PARAMS)
        write(folder/'result.json',result);write(folder/'job.json',dict(id=uid,kind='shadow',source=d['source'],title=c['title']+'｜D邊界出口v3｜原A/B保留',spec=result['spec'],models=['v4-640'],created=time.time(),state='completed',total=len(rows),execution='筆電離線；原遮罩沿用；PID關閉'))
        write(folder/'status.json',dict(state='completed',completed=len(rows),total=len(rows)))
        switches=[dict(index=i,player_s=f['player_s'],state=r['junction']['state'],reason=r['junction']['reason']) for i,(f,r) in enumerate(zip(d['frames'],rows)) if i==0 or r['junction']['state']!=rows[i-1]['junction']['state']]
        c.update(revision_job=uid,revision_algorithm='junction-anchored-v3');assert sha(old/'result.json')==before
        report['cases'].append(dict(id=c['id'],job=uid,baseline_job=c['job'],baseline_sha256=before,frames=len(rows),valid=sum(r['valid'] for r in rows),switches=switches,rejected=sum(sum(x['reason']!='GENERATION_COUNTS' for x in r['junction']['rejected']) for r in rows)))
        c['revision_summary']=report['cases'][-1]
        print(json.dumps(report['cases'][-1],ensure_ascii=False),flush=True)
    if (exp/'v3-report.json').exists():shutil.copyfile(exp/'v3-report.json',exp/('v3-report-history-'+uuid.uuid4().hex[:8]+'.json'))
    write(exp/'v3-report.json',report);write(exp/'cases.json',manifest)
if __name__=='__main__':main()
