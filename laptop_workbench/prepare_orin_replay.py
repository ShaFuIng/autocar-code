"""Decode synced camera recordings locally. No hardware access."""
import cv2,json,sys
from pathlib import Path
from datetime import datetime
root=Path(sys.argv[1]);out=root/'preview';out.mkdir(exist_ok=True);rows=[];first=None
for p in sorted(root.glob('video_*.avi')):
    stamps=[json.loads(l) for l in p.with_name(p.stem.replace('video_','frames_')+'.jsonl').read_text().splitlines() if l]
    cap=cv2.VideoCapture(str(p));n=0
    while True:
        ok,im=cap.read()
        if not ok:break
        stamp=stamps[n];t=datetime.fromisoformat(stamp['time']).timestamp();first=t if first is None else first
        i=len(rows);cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,88])[1].tofile(str(out/(str(i).zfill(6)+'.jpg')))
        rows.append(dict(frame=i,capture_s=t-first,time=stamp['time'],source=p.name,source_frame=n));n+=1
    cap.release()
    assert n==len(stamps),'Frame/timestamp mismatch'
assert rows,'No frames'
(out/'frames.json').write_text(json.dumps(rows),encoding='utf-8')
