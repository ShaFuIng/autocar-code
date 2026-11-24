import json,cv2,numpy as np
from pathlib import Path
root=Path('Orin_Interface_v1/laptop_workbench');out=root/'data/experiments/field-recordings-20261024';s=json.loads((out/'sync-result.json').read_text(encoding='utf8'))
for r in s['completed'][:6]:
 p=Path(r['folder']);frames=p/'preview/frames.json'
 if not frames.exists():continue
 rows=json.loads(frames.read_text());indices=np.linspace(0,len(rows)-1,12,dtype=int);tiles=[]
 for i in indices:
  im=cv2.imdecode(np.fromfile(str(p/f'preview/{i:06d}.jpg'),np.uint8),1);im=cv2.resize(im,(320,180));cv2.rectangle(im,(0,0),(320,24),(0,0,0),-1);cv2.putText(im,f'{r["id"][:8]} #{i} {rows[i]["capture_s"]:.1f}s',(5,17),0,.45,(255,255,255),1);tiles.append(im)
 sheet=np.vstack([np.hstack(tiles[j:j+3]) for j in range(0,12,3)]);cv2.imencode('.jpg',sheet)[1].tofile(str(out/(r['id'][:8]+'-contact.jpg')))
 print(r['id'][:8],len(rows),rows[-1]['capture_s'])
