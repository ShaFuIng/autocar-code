"""Reuse exact frozen images/masks and A/B. H-only offline experiment."""
import copy,json,sys,time,uuid,shutil
from pathlib import Path
import cv2
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'laptop_workbench'))
from analysis_jobs import read,write
from shadow_worker import sha,picture,pid_rows
from junction_left import JunctionLeft,PARAMS

def main():
    cv2.setNumThreads(2);data=ROOT/'laptop_workbench/data';exp=data/'experiments/junction-cases'
    manifest=read(exp/'cases.json'); report=dict(parameters=PARAMS,algorithm_sha256=sha(ROOT/'laptop_workbench/junction_left.py'),cases=[],inferred=0,hardware_output=False)
    for c in manifest['cases']:
        if c['id']!='first':continue
        frozen={key:sha(data/'analysis'/c[key]/'result.json') for key in ('job','candidate_job','revision_job','memory_job','corridor_job','fused_job')}
        old=data/'analysis'/c['job']; d=read(old/'result.json');before=sha(old/'result.json')
        uid=uuid.uuid4().hex;folder=data/'analysis'/uid;folder.mkdir();(folder/'raw').mkdir();(folder/'H').mkdir()
        tracker=JunctionLeft();rows=[]
        for i,f in enumerate(d['frames']):
            raw=old/'raw'/f'{i:06d}.jpg'; mask=old/'A'/f'{i:06d}.png'
            assert sha(raw)==f['sha256'];assert sha(mask)==sha(old/'B'/mask.name)
            im=picture(raw);m=picture(mask)[:,:,0]>0;tick=time.perf_counter()
            r=tracker.step(im,m,f['capture_s'],d['models']['A'][i]['A']);r['path_ms']=(time.perf_counter()-tick)*1000;rows.append(r)
            shutil.copyfile(raw,folder/'raw'/raw.name);shutil.copyfile(mask,folder/'H'/mask.name)
        g=copy.deepcopy(d['spec']['groups'][1]);g.update(id='H',label='左邊界與歷史寬度 v7',algorithm='junction-left-v7',pid_enabled=False)
        models,summary=pid_rows(rows,g['parameters'],False)
        result=copy.deepcopy(d);result.update(id=uid,models={'H':models},summaries={'H':summary},baseline_job=c['job'],spec=dict(d['spec'],groups=[g]),cache=dict(inferred=0,mask_reused=len(rows),path_reused=0),candidate_parameters=PARAMS)
        write(folder/'result.json',result);write(folder/'job.json',dict(id=uid,kind='shadow',source=d['source'],title=c['title']+'｜H左邊界v7｜原A/B保留',spec=result['spec'],models=['v4-640'],created=time.time(),state='completed',total=len(rows),execution='筆電離線；原遮罩沿用；PID關閉'))
        write(folder/'status.json',dict(state='completed',completed=len(rows),total=len(rows)))
        switches=[dict(index=i,player_s=f['player_s'],state=r['junction']['state'],reason=r['junction']['reason']) for i,(f,r) in enumerate(zip(d['frames'],rows)) if i==0 or r['junction']['state']!=rows[i-1]['junction']['state']]
        c.update(left_job=uid,left_algorithm='junction-left-v7');assert sha(old/'result.json')==before
        report['cases'].append(dict(id=c['id'],job=uid,baseline_job=c['job'],baseline_sha256=before,frames=len(rows),valid=sum(r['valid'] for r in rows),switches=switches,rejected=sum(sum(x['reason']!='GENERATION_COUNTS' for x in r['junction']['rejected']) for r in rows)))
        c['left_summary']=report['cases'][-1]
        report['cases'][-1]['frozen_sha256']=frozen
        assert all(sha(data/'analysis'/c[key]/'result.json')==value for key,value in frozen.items())
        print(json.dumps(report['cases'][-1],ensure_ascii=False),flush=True)
    if (exp/'v7-report.json').exists():shutil.copyfile(exp/'v7-report.json',exp/('v7-report-history-'+uuid.uuid4().hex[:8]+'.json'))
    write(exp/'v7-report.json',report);write(exp/'cases.json',manifest)
if __name__=='__main__':main()
