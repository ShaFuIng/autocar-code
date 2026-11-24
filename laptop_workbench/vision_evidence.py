"""Index exact captured PID evidence; never synthesize masks from nearby video."""
import hashlib,json,re
from datetime import datetime,timezone

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def evidence(folder):
    rows=[]
    for file in sorted(folder.glob('vision-*.json')):
        if not re.fullmatch(r'vision-\d{6}\.json',file.name):continue
        try:
            d=json.loads(file.read_text(encoding='utf-8'));stem=file.stem
            if d['raw_file']!=stem+'.jpg' or d['mask_file']!=stem+'.png':continue
            if digest(folder/d['raw_file'])!=d['raw_sha256'] or digest(folder/d['mask_file'])!=d['mask_sha256']:continue
            rows.append(d)
        except (OSError,KeyError,ValueError):continue
    return rows

def index(app,folder,uid,version):
    data=evidence(folder)
    if not data:return None
    key=uid.replace('orin-manual-','orin-pidvision-');origin=data[0]['capture_at'];rows=[]
    for i,d in enumerate(data):
        rows.append(dict(frame=i,capture_s=d['capture_at']-origin,time=datetime.fromtimestamp(d['captured_wall'],timezone.utc).isoformat(),vision=d,exact_evidence=True))
        app.assets[key+'/'+str(i).zfill(6)+'.jpg']=folder/d['raw_file']
        app.assets[key+'/'+str(i).zfill(6)+'.png']=folder/d['mask_file']
    app.replays[key]=dict(id=key,title='PID當時辨識原圖／遮罩 · '+uid[-32:][:8],kind='field',rows=rows,has_images=True,version=version,source='orin',folder=folder,pid_events=app.replays[uid]['pid_events'],events=app.replays[uid]['events'],data_warnings=['辨識頻率非影片幀率；缺幀不補造，使用原始capture_at間隔。'],mask_origin='live_pid_original')
    return key
