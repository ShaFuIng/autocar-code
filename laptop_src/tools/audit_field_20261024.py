import json,hashlib,datetime
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]/'laptop_workbench';out=ROOT/'data/experiments/field-recordings-20261024';sync=json.loads((out/'sync-result.json').read_text(encoding='utf8'));catalog=json.loads((out/'catalogue.json').read_text(encoding='utf8'));lookup={(r['kind'],r['id']):r for r in catalog};items=[];totalfiles=0
notes={'1274e47f':'第二入口接近、通過與暖光隧道，後續彎道；接近至通過可見。','4edf0adc':'暖光隧道出口、第一入口接近與通過、後續道路。','60aee9fc':'斜坡接近、上坡、坡頂及下坡；含相鄰跑道。','dcb92d6b':'起伏／斜坡附近道路；CAMERA_LOST中斷，保留已錄內容。','ed704ccd':'CAMERA_LOST短片，不能當完整通過紀錄。','ef5740a2':'起伏跑道與開口路段，正常按鈕停止。'}
for r in sync['completed']:
 p=Path(r['folder']);manifest=json.loads((p/'sync_manifest.json').read_text());bad=[]
 for f in manifest:
  file=p/f['name'];digest=hashlib.sha256(file.read_bytes()).hexdigest()
  if digest!=f['sha256'] or file.stat().st_size!=f['bytes']:bad.append(f['name'])
 totalfiles+=len(manifest);assert not bad,bad
 rows=json.loads((p/'preview/frames.json').read_text()) if (p/'preview/frames.json').exists() else []
 stamps=[]
 for f in sorted(p.glob('frames_*.jsonl')):stamps += [json.loads(l) for l in f.read_text().splitlines() if l]
 assert len(rows)==len(stamps),(r['id'],len(rows),len(stamps))
 meta=lookup[(r['kind'],r['id'])]['metadata'].get('summary.json',{});gaps=[rows[i]['capture_s']-rows[i-1]['capture_s'] for i in range(1,len(rows))]
 items.append(dict(**r,started_local=datetime.datetime.fromisoformat(meta['created_at']).astimezone().isoformat() if meta.get('created_at') else None,reason=meta.get('reason'),status=meta.get('status'),frames=len(rows),captured_span_s=rows[-1]['capture_s'] if rows else 0,max_frame_interval_s=max(gaps) if gaps else None,video_files=[f.name for f in p.glob('video_*.avi')],hash_verified=True,frame_timestamp_count_matches=True,scene_note=notes.get(r['id'][:8],'歷史試跑／錄影，未逐段判讀場景'),scene_review='12-frame contact sheet' if r['id'][:8] in notes else 'not reviewed'))
report=dict(date='2026-10-24',records=len(items),files=totalfiles,original_bytes=sum(x['bytes'] for x in items),errors=sync['errors'],root=str((ROOT/'data/orin/records').resolve()),items=items,offline_originals_and_replay=True,weights_v4_available=(ROOT.parents[1]/'yolo26_sem_v4_trial/runs/trial01/weights/best.pt').is_file(),vehicle_commands=False,source_deleted=False)
(out/'acceptance.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
(out/'保存清單.txt').write_text('10/24 場地資料已保存（原檔＋時間戳＋控制/PID紀錄，依各筆實際存在內容）\n'+report['root']+'\n'+f"共{len(items)}筆、{totalfiles}檔，原始{report['original_bytes']} bytes，SHA256核對通過。\n"+'\n'.join(f"{r['started_local']} | {r['id']} | {r['frames']}格 | {r['captured_span_s']:.2f}s | {r['reason']} | {r['scene_note']}\n{r['folder']}" for r in items),encoding='utf8')
print(json.dumps({k:v for k,v in report.items() if k!='items'},ensure_ascii=False));print([(x['id'][:8],x['frames'],round(x['captured_span_s'],2),x['max_frame_interval_s']) for x in items[:6]])
