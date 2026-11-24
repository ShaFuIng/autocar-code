"""Refresh human-readable findings from measured results; never change models."""
from pathlib import Path
import json,base64
OUT=Path(__file__).resolve().parent/'data/experiments/night-20261018'
def read(name):return json.loads((OUT/name).read_text(encoding='utf-8-sig'))
def parity():
    values=[]
    for side in ['local','orin']:
        line=next(l for l in (OUT/(side+'-parity.log')).read_text(encoding='utf-8-sig').splitlines() if l.startswith('PARITY_JSON:'))
        values.append(json.loads(line[12:]))
    local,orin=values;a=base64.b64decode(local.pop('packed_mask'));b=base64.b64decode(orin.pop('packed_mask'))
    assert len(a)==len(b)
    diff=sum(bin(x^y).count('1') for x,y in zip(a,b))
    value=dict(local=local,orin=orin,same_input=local['image_sha256']==orin['image_sha256'],same_weights=local['weights_sha256']==orin['weights_sha256'],exact_mask_match=diff==0,different_pixels=diff,total_pixels=720*1280,scope='單張 v003_006 原圖、v4、imgsz640、FP32；不代表全影片一致性或等同效能。')
    (OUT/'parity.json').write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
try:parity()
except (StopIteration,KeyError,FileNotFoundError):pass
r=read('report.json');rows=r['results'];baseline=next((x for x in rows if x['model']=='v4-640'),None)
findings=[]
if baseline:
    findings.append(f"v4-640 開發集白線 IoU {baseline['iou']*100:.2f}%，Dice {baseline['dice']*100:.2f}%。")
    for x in rows:
        if x is baseline:continue
        findings.append(f"{x['model']}：IoU {x['iou']*100:.2f}%，相對 v4-640 差 {(x['iou']-baseline['iou'])*100:+.2f} 個百分點；本機推論中位 {x['latency_median_ms']:.1f} ms。")
if (OUT/'parity.json').exists():
    p=read('parity.json');findings.append(f"本機／Orin 同圖同權重單張檢查：{p['different_pixels']} / {p['total_pixels']} 像素分類不同；完整影片與候選 Orin 效能尚未驗證。")
if r['state']=='completed':
    findings.append('本輪同輸入尺寸比較，v5-focus02 未超過 v4-640，v5-focus03 未超過 v4-960；兩個從零訓練的小模型也未超過 v4。沒有足夠證據替換原 v4。')
    findings.append('本輪驗收：8 組評分、16 組影片分析共 20,480 影格；兩個新模型各完成 30 epoch。原影片、時間戳、資料集與 v4 權重雜湊核對一致。')
findings.append('延遲來自不同時段的少量影像，未控制 GPU 時脈／溫度，僅作本機初步觀察，不作硬體效能排名。')
findings.append('保留原 v4 部署。任何較高分候選仍需逐影格人工確認單側、路口、反光及獨立場地測試，不能依本輪分數直接上車。')
result=dict(state=r['state'],updated=r['updated'],findings=findings,completed_models=len(rows),completed_video_runs=len(r['videos']),errors=len(r['errors']))
(OUT/'findings.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
