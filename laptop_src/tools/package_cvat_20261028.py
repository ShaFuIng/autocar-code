"""Local raw-frame annotation packages. No predictions or hardware access."""
import json,csv,hashlib,zipfile,io
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];WB=ROOT/'laptop_workbench'
OUT=WB/'data/experiments/cvat-pack-20261028';OUT.mkdir(exist_ok=True)
clips={c['name']:c for c in json.loads((WB/'data/experiments/new-junctions-20261028/report.json').read_text(encoding='utf8'))['clips']}
for c in clips.values():c['stamps']=json.loads((Path(c['source_folder'])/'preview/frames.json').read_text())
groups=[('01_warm_light','暖光隧道與明暗出口',[('entry1',0,18,23),('entry2',27,34,9)]),('02_curves','彎道與單側白線',[('slope',0,18,16),('entry2',35,52,16)]),('03_slope','斜坡、坡頂與坡腳',[('slope',20,64.5,32)]),('04_junctions','兩個路口與支路開口',[('entry1',25,57,16),('entry2',10,26,16)])]
manifest=[];packs=[];used=set()
for group,title,ranges in groups:
 selected=[]
 for name,start,end,count in ranges:
  stamps=clips[name]['stamps']
  for t in np.linspace(start,end,count):
   i=min(range(len(stamps)),key=lambda i:abs(stamps[i]['capture_s']-t))
   if (name,i) not in used:used.add((name,i));selected.append((name,i))
 # Include actual failure frame, replacing nearest sample within same clip.
 targets=[('entry1',139)] if group=='01_warm_light' else ([('slope',361)] if group=='03_slope' else [])
 for name,i in targets:
  j=min([j for j,x in enumerate(selected) if x[0]==name],key=lambda j:abs(selected[j][1]-i));used.discard(selected[j]);selected[j]=(name,i);used.add((name,i))
 selected.sort();images=[];rows=[];zip_path=OUT/(group+'.zip')
 with zipfile.ZipFile(zip_path,'w',zipfile.ZIP_DEFLATED) as z:
  for seq,(name,i) in enumerate(selected):
   c=clips[name];st=c['stamps'][i];cap=cv2.VideoCapture(str(Path(c['source_folder'])/st['source']));cap.set(cv2.CAP_PROP_POS_FRAMES,st['source_frame']);ok,im=cap.read();cap.release();assert ok
   filename=f"{seq:03d}_{name}_{c['id'][:8]}_f{i:04d}_t{st['capture_s']:07.3f}.png"
   data=cv2.imencode('.png',im)[1].tobytes();z.writestr(filename,data)
   row=dict(package=group,image=filename,record_id=c['id'],source_video=str(Path(c['source_folder'])/st['source']),frame_index=i,source_frame=st['source_frame'],capture_s=st['capture_s'],captured_at=st['time'],width=im.shape[1],height=im.shape[0],sha256=hashlib.sha256(data).hexdigest(),annotation='unannotated',split_group=c['id'],purpose='human boundary annotation candidate; not ground truth')
   rows.append(row);thumb=cv2.resize(im,(320,180));cv2.putText(thumb,f"{seq:03d} {name} {st['capture_s']:.2f}s",(5,17),0,.43,(0,255,255),1);images.append(thumb)
 with zipfile.ZipFile(zip_path) as z:
  assert z.testzip() is None and len(z.namelist())==32
  for row in rows:
   data=z.read(row['image']);dec=cv2.imdecode(np.frombuffer(data,np.uint8),1);assert dec.shape[:2]==(row['height'],row['width']) and hashlib.sha256(data).hexdigest()==row['sha256']
 sheet=np.vstack([np.hstack(images[j:j+4]) for j in range(0,len(images),4)]);cv2.imencode('.jpg',sheet)[1].tofile(str(OUT/(group+'-contact.jpg')))
 with (OUT/(group+'-manifest.csv')).open('w',encoding='utf-8-sig',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 manifest.extend(rows);packs.append(dict(name=group,title=title,file=zip_path.name,count=len(rows),bytes=zip_path.stat().st_size,sha256=hashlib.sha256(zip_path.read_bytes()).hexdigest()))
(OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf8')
(OUT/'labels.json').write_text(json.dumps([dict(name='boundary',type='any',attributes=[])],indent=2),encoding='utf8')
(OUT/'report.json').write_text(json.dumps(dict(packs=packs,total=len(manifest),verified=True,raw_native_resolution=True,predictions_included=False,split_warning='All frames from a source recording must stay in the same dataset split. These clips have been used for development; do not claim an independent test on them.'),ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(packs,ensure_ascii=False))
