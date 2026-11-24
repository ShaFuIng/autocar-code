"""Optional offline AVI extraction. Writes new cache only; never changes originals."""
import hashlib
import json
from pathlib import Path
import cv2

root=Path(__file__).resolve().parent
source=root.parent/'field_test/review_latest'
for timeline in source.glob('*/frames_*.jsonl'):
    rows=[json.loads(x) for x in timeline.read_text().splitlines() if x]
    video=timeline.parent/(timeline.stem.replace('frames_','video_')+'.avi')
    if not video.exists():
        continue
    out=root/'data'/'field_frames'/timeline.parent.name
    out.mkdir(parents=True,exist_ok=True)
    cap=cv2.VideoCapture(str(video))
    count=0
    while True:
        ok,frame=cap.read()
        if not ok:
            break
        target=out/f'{count:06d}.jpg'
        if not target.exists():
            ok,encoded=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,95])
            if not ok:
                raise RuntimeError('JPEG encoding failed')
            target.write_bytes(encoded.tobytes())
        count+=1
    cap.release()
    if count!=len(rows):
        raise RuntimeError(f'Frame count mismatch: {count} != {len(rows)}')
    (out/'manifest.json').write_text(json.dumps({'source':str(video),'sha256':hashlib.sha256(video.read_bytes()).hexdigest(),'frames':count,'timeline':str(timeline),'original_unchanged':True},indent=2),encoding='utf8')
    print(f'Extracted {count} frames to {out}')
