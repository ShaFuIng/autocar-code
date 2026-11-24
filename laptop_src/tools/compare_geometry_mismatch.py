from pathlib import Path
import sys,json,copy,uuid,time,os,hashlib,collections,shutil
W=Path(__file__).resolve().parents[1]/'laptop_workbench'
sys.path[:0]=[str(W/'runtimes/opencv-4.11'),str(W)]
from analysis_jobs import read,write
from geometry_mismatch_offline import GeometryMismatch
from shadow_worker import pid_rows
import cv2,numpy as np

cv2.setNumThreads(2)
old=W/'data/analysis/f7e170a0869d4fdd8632575997f39a4a'
out=W/'data/experiments/geometry-mismatch-v1';out.mkdir(exist_ok=True)
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before=digest(old/'result.json');r=read(old/'result.json');j=read(old/'job.json')
uid=uuid.uuid4().hex;dest=W/'data/analysis'/uid;dest.mkdir()
ids=[i for i,f in enumerate(r['frames']) if f['player_s']>=160]
# Warm the confirmation state using 2 seconds before the evaluated interval.
candidate=GeometryMismatch()
for i in range(ids[0]-20,ids[0]):candidate.step(r['models']['C'][i]['A'])
rows=[];mask_hashes=[]
for k in ('raw','A','B'):(dest/k).mkdir()
for z,i in enumerate(ids):
    original=r['models']['C'][i]['A']; tick=time.perf_counter()
    a=candidate.step(original);a.update(path_ms=(time.perf_counter()-tick)*1000,timing_scope='frozen_evidence_postprocess_only')
    rows.append(a)
    for k in ('raw','A','B'):
        ext='jpg' if k=='raw' else 'png';source=old/('raw' if k=='raw' else 'C')/f'{i:06d}.{ext}'
        try:os.link(source,dest/k/f'{z:06d}.{ext}')
        except OSError:shutil.copy2(source,dest/k/f'{z:06d}.{ext}')
    assert digest(dest/'A'/f'{z:06d}.png')==digest(dest/'B'/f'{z:06d}.png')
    mask_hashes.append(digest(dest/'A'/f'{z:06d}.png'))
groups=[copy.deepcopy(j['spec']['groups'][2]) for _ in range(2)]
groups[0].update(id='A',label='原共用車道K（保留）')
groups[1].update(id='B',label='幾何失配離線候選',algorithm='geometry-mismatch-offline-v1')
spec=copy.deepcopy(j['spec']);spec.update(start=ids[0],end=ids[-1],groups=groups)
models={'A':[copy.deepcopy(r['models']['C'][i]) for i in ids]}
models['B'],bs=pid_rows(rows,groups[1]['parameters'],False)
_,asum=pid_rows([copy.deepcopy(r['models']['C'][i]['A']) for i in ids],groups[0]['parameters'],False)
limits=['A＝原共用車道K；B＝凍結原A證據上的幾何失配後處理候選，不是完整重跑的新追蹤器。只比較播放器160.0～175.9秒，之前20格暖機。',
        '兩組V4遮罩逐檔SHA256相同，未重跑辨識；遮罩缺陷不會被路徑候選修好。雙線同列中心差超過16px連續3格才改用當前雙線支持段。',
        '單側寬度超過2秒無法核對：只保留y≥270且有當前邊界的低信心推算，移除寬度外推；不是已確認幾何失配或坡道辨識。',
        '16px、3格、2秒、y270是本輪實驗參數，不是安全標準；像素採640×360。寬度年齡採來源時間，不是播放器秒數。無道路真值，縮短或有路徑都不代表更準。PID關閉；未部署、未連Orin、未動車。']
result={k:copy.deepcopy(r[k]) for k in ('runtime','kind','source','recognition','source_video_provenance')}
result.update(id=uid,models=models,frames=[r['frames'][i] for i in ids],spec=spec,summaries={'A':asum,'B':bs},
              cache=dict(inferred=0,mask_reused=len(ids),path_reused=len(ids)),execution='offline',hardware_output=False,limitations=limits,
              partial=False,baseline_job=old.name,geometry_candidate=True,baseline_offset=ids[0])
j.update(id=uid,title='2:40後｜原K vs 幾何失配候選（同遮罩）',created=time.time(),state='completed',spec=spec)
j.pop('validation_summary',None)
write(dest/'job.json',j);write(dest/'result.json',result)
write(dest/'input.json',[read(old/'input.json')[i] for i in ids])
write(dest/'status.json',dict(state='completed',completed=len(ids),total=len(ids)));write(dest/'exit.json',dict(returncode=0))
assert digest(old/'result.json')==before
report=dict(job=uid,frames=len(ids),baseline_sha256=before,baseline_unchanged=True,mask_pairs_identical=len(ids),
            states=dict(collections.Counter(a['mode'] for a in rows)),empty_before=sum(not a['A']['path'] for a in models['A']),
            empty_after=sum(not a['path'] for a in rows),changed=sum(a['path']!=b['A']['path'] for a,b in zip(rows,models['A'])),
            events=[dict(player_s=ids[z]/10,**a['geometry']) for z,a in enumerate(rows) if z==0 or a['mode']!=rows[z-1]['mode']],
            masks_sha256=mask_hashes,limitations=limits,hardware_output=False)
write(out/'report.json',report);write(out/'job.json',dict(job=uid))
print(json.dumps({k:v for k,v in report.items() if k not in ('masks_sha256','limitations')},ensure_ascii=False),flush=True)
