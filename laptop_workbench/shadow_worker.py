"""File-only shared inference, independent temporal paths and read-only PID replay."""
import hashlib
import json
import shutil
import sys
import time
import traceback
from pathlib import Path
# Pin only CLI shadow jobs containing K; other processes keep their environment.
if __name__=='__main__' and len(sys.argv)>1:
    _job=json.loads((Path(sys.argv[1])/'job.json').read_text(encoding='utf-8'))
    if any(g.get('algorithm') in ('junction-k-v1','junction-k-mask-v1','road-entrance-v3.2','junction-k-entrance-v2','junction-k-lane-v3') for g in _job.get('spec',{}).get('groups',[])):
        _cv=Path(__file__).parent/'runtimes/opencv-4.11'
        if not (_cv/'cv2/__init__.py').is_file():raise RuntimeError('K requires isolated OpenCV 4.11 runtime')
        sys.path.insert(0,str(_cv))
import cv2
import numpy as np
from analysis_jobs import ROOT, CONFIGS, read, write

sys.path.insert(0,str(ROOT.parent/'web'))
from calibrated_path import CalibratedPath
from live_model_worker import corridor
from visual_pid import PID
from temporal_lane import TemporalLane

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def key(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()

def picture(path):
    im=cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_COLOR)
    if im is None:raise ValueError('無法讀取影格 '+str(path))
    return im

def default_predictor(weight,size):
    from overnight_runner import YoloPredictor
    return YoloPredictor(weight,size)

def path_rows(group, inputs, mask_folder, calibration_file):
    from entrance_paths import EntrancePath, ALGORITHMS
    entrance=EntrancePath(group['algorithm'], group['calibration'],group['parameters']['row_ratio']) if group['algorithm'] in ALGORITHMS else None
    from entrance_paths_v2 import EntrancePath as EntranceV2, ALGORITHMS as V2
    if group['algorithm'] in V2:entrance=EntranceV2(group['algorithm'],group['calibration'],group['parameters']['row_ratio'])
    from road_segments_v3 import RoadPath, ALGORITHMS as V3
    if group['algorithm'] in V3:entrance=RoadPath(group['algorithm'],group['calibration'],group['parameters']['row_ratio'])
    if group['algorithm']=='road-memory-v3.1':
        from road_segments_v31 import RoadPath as Road31
        entrance=Road31(group['algorithm'],group['calibration'],group['parameters']['row_ratio'])
    if group['algorithm']=='junction-straight-v1':
        from junction_straight import JunctionStraight
        entrance=JunctionStraight(group['algorithm'],None,group['parameters']['row_ratio'])
    if group['algorithm']=='junction-k-v1':
        from junction_k import KPath
        entrance=KPath(group['parameters']['row_ratio'])
    if group['algorithm']=='junction-k-mask-v1':
        from junction_k_mask import KMaskPath
        entrance=KMaskPath(group['parameters']['row_ratio'])
    if group['algorithm']=='road-entrance-v3.2':
        from road_segments_v32 import RoadPath as Road32
        entrance=Road32(group['algorithm'],None,group['parameters']['row_ratio'])
    if group['algorithm']=='junction-k-entrance-v2':
        from junction_k_entrance import KEntrancePath
        entrance=KEntrancePath(group['parameters']['row_ratio'])
    if group['algorithm']=='junction-k-lane-v3':
        from junction_k_lane import KLanePath
        entrance=KLanePath(group['parameters']['row_ratio'])
    tracker=CalibratedPath(calibration_file) if group['algorithm']=='calibrated-v7' else TemporalLane() if group['algorithm']=='temporal-v2' else None
    for i,frame in enumerate(inputs):
        im=picture(frame['image'])
        mask=cv2.imdecode(np.fromfile(str(mask_folder/f'{i:06d}.png'),np.uint8),0)>0
        start=time.perf_counter()
        if entrance:
            r=entrance.step(im,mask,frame['capture_s'])
        elif group['algorithm']=='legacy9':
            path,quality=corridor(mask,group['parameters']['row_ratio'])
            r=dict(path=path,valid=quality>=.66,quality=quality,mode='double',reason='LEGACY9')
        else:
            r=tracker.step(im,mask,frame['capture_s'])
            if group['algorithm']=='temporal-v2':r['path']=[[x*2,y*2] for x,y in r['image_path']]
        r.update(path_ms=(time.perf_counter()-start)*1000,capture_s=frame['capture_s'],frame=frame['frame'])
        # Match the real worker's JSON boundary: numpy scalars become Python
        # numbers before visual_pid's strict numeric contract sees them.
        yield json.loads(json.dumps(r,allow_nan=False))

def pid_rows(rows,parameters,enabled=True,dynamic=False):
    from entrance_paths import AdaptivePID
    adaptive=AdaptivePID(parameters) if dynamic else None
    pid=PID(parameters);out=[];started=False;stop=None;ready=0;events=[];previous=None;last_steer=None
    for i,r in enumerate(rows):
        ready=ready+1 if r['valid'] else 0
        # Continuous diagnosis is separate from the first simulated run.
        p=(adaptive.step(r['capture_s'],r['path'] if r['valid'] else []) if adaptive else pid.step_path(r['capture_s'],r['path'] if r['valid'] else [])) if enabled else None
        if enabled and ready>=3 and p['valid'] and not started and stop is None:started=True
        if enabled and started and stop is None and not p['valid']:
            stop=dict(index=i,frame=r['frame'],capture_s=r['capture_s'],reason=p['reason'])
        mode=r.get('mode','unknown')
        if mode!=previous:events.append(dict(index=i,type='mode',text=f'{previous or "initial"} → {mode}'))
        if not r['valid']:events.append(dict(index=i,type='path_invalid',text=r.get('reason','NO_PATH')))
        if enabled and not p['valid']:events.append(dict(index=i,type='pid_invalid',text=p['reason']))
        delta=None
        if p and p.get('steer_us') is not None:
            delta=None if last_steer is None else p['steer_us']-last_steer
            last_steer=p['steer_us']
        out.append(dict(A=r,pid=p,steer_change_us=delta,simulated_run='stopped' if stop else 'running' if started else 'not_started'))
        previous=mode
    return out,dict(first_stop=stop,started=started,events=events,
        path_valid=sum(r['valid'] for r in rows),frames=len(rows),
        pid_valid=sum(bool(r['pid'] and r['pid']['valid']) for r in out),
        limited=sum(bool(r['pid'] and r['pid'].get('limited')) for r in out))

def run(folder,predictor_factory=default_predictor):
    folder=Path(folder);job=read(folder/'job.json');spec=job['spec'];inputs=read(folder/'input.json')
    cache=folder.parent/'_shadow_cache';cache.mkdir(exist_ok=True)
    cv2.setNumThreads(2)
    sources=[];models={};summaries={};stats=dict(inferred=0,mask_reused=0,path_reused=0,live_mask_reused=0)
    result=dict(runtime=dict(opencv=cv2.__version__,numpy=np.__version__),id=job['id'],kind='shadow',source=job['source'],models=models,frames=sources,
        spec=spec,summaries=summaries,cache=stats,execution=job['execution'],hardware_output=False,
        limitations=['固定影片開環推演，不是車輛閉環模擬；PWM不換算未標定車速。',
          '單輪停止點模擬三張有效準備後的路徑／PID失效；不模擬車板故障、車端處理延遲、實車開始時刻或限時結束。',
          '筆電耗時不是Orin控制延遲；無真值不計準確率。姿態不同或未知時公分推算僅供探索。',
          '區間起點冷啟動；抽幀保留來源時間戳，可能觸發既有PID max_gap。'])
    def cancelled():return (folder/'cancel.json').exists()
    def progress(stage,done):
        write(folder/'status.json',dict(state='running',current=stage,completed=done,total=job['total'],cache=stats))
    (folder/'raw').mkdir(exist_ok=True)
    timestamps=[r['capture_s'] for r in inputs]
    if any(not np.isfinite(t) for t in timestamps) or any(b<=a for a,b in zip(timestamps,timestamps[1:])):
        raise ValueError('來源時間戳必須有限且嚴格遞增；不合成或重排時間')
    for i,r in enumerate(inputs):
        dest=folder/'raw'/f'{i:06d}.jpg';shutil.copyfile(r['image'],dest);r['image']=str(dest)
        sources.append(dict(frame=r['frame'],capture_s=r['capture_s'],sha256=sha(dest)))
    infer_keys={};completed=0;aborted=False
    # Path/PID edits must not invalidate identical inference. Migrate only caches
    # whose predictor, weights and size match, then verify each PNG hash.
    old_identities=[]
    for metadata in cache.glob('*.json'):
        if metadata.name.endswith('-paths.json'):continue
        identity=read(metadata).get('identity')
        if identity and identity not in old_identities:old_identities.append(identity)
    for model in job['models']:
        _,weight,size=next(c for c in CONFIGS if c[0]==model)
        weight=ROOT.parents[1]/weight
        identity=dict(model=model,weight_sha256=sha(weight),size=size,
                      predictor_sha256=sha(ROOT/'overnight_runner.py'),worker_sha256=sha(Path(__file__)))
        infer_keys[model]=identity
        target=folder/model;target.mkdir(exist_ok=True);predictor=None
        for i,r in enumerate(inputs):
            if cancelled():aborted=True;break
            fingerprint=key(dict(source=sources[i]['sha256'],**identity))
            cached=cache/(fingerprint+'.png');meta=cache/(fingerprint+'.json')
            tick=time.perf_counter()
            live=r.get('live_evidence')
            if live and live.get('raw_sha256')==sources[i]['sha256'] and live.get('weights_sha256')==identity['weight_sha256'] and live.get('recognition',{}).get('input_size')==size and live.get('recognition',{}).get('contract')=='v4-semantic-class1-nearest-v1' and sha(live['mask'])==live['mask_sha256']:
                shutil.copyfile(live['mask'],cached)
                write(meta,dict(sha256=sha(cached),identity=identity,origin='live_pid_original'))
                stats['live_mask_reused']+=1
            if not cached.exists():
                for prior in old_identities:
                    if any(prior.get(k)!=identity[k] for k in ('model','weight_sha256','size','predictor_sha256')):continue
                    oldkey=key(dict(source=sources[i]['sha256'],**prior))
                    oldpng=cache/(oldkey+'.png');oldmeta=cache/(oldkey+'.json')
                    if oldpng.exists() and oldmeta.exists() and read(oldmeta).get('sha256')==sha(oldpng):
                        shutil.copyfile(oldpng,cached)
                        write(meta,dict(read(oldmeta),identity=identity,migrated_from=oldkey))
                        break
            if cached.exists() and meta.exists() and read(meta).get('sha256')==sha(cached):
                stats['mask_reused']+=1
            else:
                if predictor is None:predictor=predictor_factory(weight,size)
                im=picture(r['image']);mask=np.squeeze(predictor(im)).astype(np.uint8)
                if mask.ndim!=2:raise ValueError('V4遮罩維度無效')
                mask=cv2.resize(mask,(im.shape[1],im.shape[0]),interpolation=cv2.INTER_NEAREST)
                cv2.imencode('.png',(mask>0).astype(np.uint8)*255)[1].tofile(str(cached))
                write(meta,dict(sha256=sha(cached),identity=identity,inference_ms=(time.perf_counter()-tick)*1000))
                stats['inferred']+=1
            shutil.copyfile(cached,target/f'{i:06d}.png')
            completed+=1;progress('共用辨識 '+model,completed)
        del predictor
        if aborted:break
    result['recognition']=infer_keys
    if not aborted:
        dependencies={n:sha(ROOT.parent/n) for n in ['web/calibrated_path.py','web/live_model_worker.py',
            'laptop_workbench/temporal_lane.py','laptop_workbench/curve_replay.py','laptop_workbench/ground_calibration.py','laptop_workbench/entrance_paths.py','laptop_workbench/entrance_paths_v2.py','laptop_workbench/road_segments_v3.py','laptop_workbench/road_segments_v31.py','laptop_workbench/junction_straight.py','laptop_workbench/junction_k.py','laptop_workbench/junction_k_mask.py','laptop_workbench/road_segments_v32.py','laptop_workbench/junction_k_entrance.py','laptop_workbench/junction_k_lane.py']}
        for group in spec['groups']:
            name=group['id'];calfile=folder/(name+'-calibration.json')
            if group['calibration']:write(calfile,group['calibration'])
            # PID-only edits reuse paths; legacy9 alone needs its lookahead row.
            cachekey=key(dict(frames=sources,inference=infer_keys[group['model']],algorithm=group['algorithm'],
                calibration=group['calibration'],dependencies=dependencies,worker=sha(Path(__file__)),runtime=dict(opencv=cv2.__version__,numpy=np.__version__),
                row=group['parameters']['row_ratio'] if group['algorithm']=='legacy9' or group['algorithm'].startswith(('entrance-','road-')) else None))
            pathfile=cache/(cachekey+'-paths.json');cached=read(pathfile) if pathfile.exists() else None
            rows=[]
            generator=iter(cached) if cached is not None else path_rows(group,inputs,folder/group['model'],calfile)
            for r in generator:
                if cancelled():aborted=True;break
                rows.append(r);completed+=1
                if cached is not None:stats['path_reused']+=1
                progress('路徑／PID '+name,completed)
            if not aborted and cached is None:write(pathfile,rows)
            models[name],summaries[name]=pid_rows(rows,group['parameters'],group['pid_enabled'],dynamic=group['algorithm'].startswith(('entrance-','road-')))
            # Keep the established review schema: model=group id, target=A.
            target=folder/name;target.mkdir(exist_ok=True)
            for i in range(len(rows)):shutil.copyfile(folder/group['model']/f'{i:06d}.png',target/f'{i:06d}.png')
            write(folder/'result.json',result)
            if aborted:break
    result['partial']=aborted
    write(folder/'result.json',result)
    write(folder/'status.json',dict(state='cancelled' if aborted else 'completed',completed=completed,
                                   total=job['total'],finished=time.time(),cache=stats))
    return result

if __name__=='__main__':
    try:run(sys.argv[1])
    except Exception:
        write(Path(sys.argv[1])/'status.json',dict(state='failed',error=traceback.format_exc()))
        raise
