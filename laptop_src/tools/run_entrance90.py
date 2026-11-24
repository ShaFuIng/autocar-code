"""Reuse verified saved masks only. Never constructs a predictor or remote client."""
import copy, hashlib, json, sys, time, uuid, shutil, os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]/'laptop_workbench'
sys.path.insert(0,str(ROOT))
from analysis_jobs import read,write,alive
from entrance_paths import EntrancePath,ALGORITHMS
from shadow_worker import picture,pid_rows

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    base=ROOT/'data/analysis/31a8163b457d4ecc8069ceaa68f1fbc2';old=read(base/'result.json')
    for f in base.parent.glob('*/status.json'):
        status=read(f);job=read(f.with_name('job.json'))
        if status.get('state') in ('running','queued') and alive(job.get('pid')):raise RuntimeError('已有分析執行中')
    uid=uuid.uuid4().hex;folder=base.parent/uid;folder.mkdir()
    job=read(base/'job.json');job.update(id=uid,created=time.time(),pid=os.getpid(),state='running',
        title='入口候選新版ABC｜前90秒｜沿用V4｜PID v37動態黃線',execution='筆電CPU／既有V4遮罩重用；無網路或硬體',total=2148)
    for g,algorithm in zip(job['spec']['groups'],ALGORITHMS):
        g.update(algorithm=algorithm,label=algorithm+'（動態黃線）',dynamic_sampling=True)
    job['spec']['version']=2;write(folder/'job.json',job)
    result=copy.deepcopy(old);result.update(id=uid,spec=job['spec'],models={},summaries={},baseline_job=old['id'],
        cache=dict(inferred=0,mask_reused=len(old['frames']),path_reused=0),execution=job['execution'])
    result['limitations']=[s for s in result['limitations'] if not s.startswith('三組首次模擬開始時間不同：')]
    result['limitations'][:0]=['新版為離線探索候選；啟發式分數不是正確率，未接入車控。',
        'C姿態不符，投影範圍外另標記探索；無法據此驗收校正。黃線為動態取樣，v37保存參數未修改。',
        'B只補對齊後短段歷史，不沿用全畫面丟線；本輪未加入速度降級，PWM仍沿用v37。']
    (folder/'raw').mkdir();manifest=[]
    for i,f in enumerate(old['frames']):
        raw=base/'raw'/f'{i:06d}.jpg';mask=base/'v4-640'/f'{i:06d}.png'
        if digest(raw)!=f['sha256']:raise ValueError('來源影格SHA不符')
        shutil.copyfile(raw,folder/'raw'/raw.name)
        manifest.append(dict(index=i,raw_sha256=digest(raw),mask_sha256=digest(mask)))
    write(folder/'reuse_manifest.json',manifest)
    completed=0
    for g in job['spec']['groups']:
        tracker=EntrancePath(g['algorithm'],g['calibration'],g['parameters']['row_ratio']);rows=[]
        target=folder/g['id'];target.mkdir()
        for i,f in enumerate(old['frames']):
            if (folder/'cancel.json').exists():raise RuntimeError('使用者取消')
            im=picture(base/'raw'/f'{i:06d}.jpg');mask=picture(base/'v4-640'/f'{i:06d}.png')[:,:,0]>0
            tick=time.perf_counter();r=tracker.step(im,mask,f['capture_s'])
            r.update(path_ms=(time.perf_counter()-tick)*1000,capture_s=f['capture_s'],frame=f['frame'])
            rows.append(r);shutil.copyfile(base/'v4-640'/f'{i:06d}.png',target/f'{i:06d}.png')
            completed+=1
            if i%60==0:
                write(folder/'status.json',dict(state='running',current=g['algorithm'],completed=completed,total=2148))
                print(g['id'],i,flush=True)
        result['models'][g['id']],result['summaries'][g['id']]=pid_rows(rows,g['parameters'],True,dynamic=True)
        write(folder/'result.json',result)
    result['comparison']={k:dict(baseline_path=old['summaries'][k]['path_valid'],new_path=s['path_valid'],
        baseline_pid=old['summaries'][k]['pid_valid'],new_pid=s['pid_valid'],
        dynamic_rows=sum(r['pid']['sample_row_ratio']>.601 for r in result['models'][k]),
        fixed_row_on_new_path=pid_rows([r['A'] for r in result['models'][k]],job['spec']['groups'][ord(k)-65]['parameters'])[1]['pid_valid'])
        for k,s in result['summaries'].items()}
    result['implementation_sha256']={n:digest(ROOT/n) for n in ['entrance_paths.py','shadow_worker.py']}
    write(folder/'result.json',result);write(folder/'status.json',dict(state='completed',completed=2148,total=2148,finished=time.time()))
    write(folder/'exit.json',dict(returncode=0))
    report=ROOT/'data/experiments/entrance90';report.mkdir(exist_ok=True)
    write(report/'report.json',dict(job=uid,baseline=old['id'],comparison=result['comparison'],limitations=result['limitations'],
        frames=716,mask_reused=716,inferred=0,hardware_output=False,implementation_sha256=result['implementation_sha256']))
    print(json.dumps(dict(job=uid,comparison=result['comparison']),ensure_ascii=False),flush=True)
if __name__=='__main__':main()
