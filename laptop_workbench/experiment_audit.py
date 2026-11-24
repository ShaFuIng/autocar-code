"""Read-only source audit; writes only new experiment metadata."""
from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent/'data/experiments/night-20261018'
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
val={sha(p):p.name for p in (ROOT/'boundary_masks_v4_checked/dataset/images/val').glob('*')}
models={}
for name,folder in [('v3','yolo26_sem_v3_trial'),('v4','yolo26_sem_v4_trial'),('v5','yolo26_sem_v5_focus70')]:
    train=list((ROOT/folder/'dataset/images/train').glob('*'))
    match=[dict(train=p.name,val=val[sha(p)]) for p in train if sha(p) in val]
    models[name]=dict(train_files=len(train),exact_train_overlap_with_val=match,note='僅檔案 SHA256 比對；重新編碼、近鄰影格與預訓練繼承關係仍可能共享場景。')
videos=[]
for folder in [ROOT/'完整跑道錄影_20261016_155847_3e961ecb',ROOT/'Orin_Interface_v1/field_test/review_latest/08dfcb8c29f74ae78dba9f294060e8e9']:
    for p in sorted(folder.glob('video_*.avi')):
        stamps=p.with_name(p.stem.replace('video_','frames_')+'.jsonl')
        videos.append(dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=sha(p),timestamps_sha256=sha(stamps)))
result=dict(models=models,videos=videos,method='固定 v4 開發驗證集，沒有重新隨機切分。全跑道影片含訓練影格；影片是回歸檢查，不是獨立測試集。')
(OUT/'source_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(models,ensure_ascii=False))
