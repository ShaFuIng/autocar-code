"""Run on Orin via python3 stdin. Only GET existing service caches; no imports from control code."""
import base64,json,socket,subprocess,urllib.request,urllib.error
from pathlib import Path

addresses=['127.0.0.1']+subprocess.check_output(['hostname','-I'],text=True).split()
credential=base64.b64encode(('jetson:'+ (Path.home()/'.config/orin_lab/token').read_text().strip()).encode()).decode()
out={'checks':[]}
for port in (8000,8091):
    for address in addresses:
        if ':' in address:continue
        base=f'http://{address}:{port}'
        try:
            try:
                urllib.request.urlopen(base+'/',timeout=2)
                auth_required=False
            except urllib.error.HTTPError as exc:
                auth_required=exc.code==401
            req=urllib.request.Request(base+'/',headers={'Authorization':'Basic '+credential})
            with urllib.request.urlopen(req,timeout=3) as r:
                out['checks'].append({'port':port,'login_protected':auth_required,'page_status':r.status,'page_bytes':len(r.read())})
            if port==8000:
                def get(path):
                    req=urllib.request.Request(base+path,headers={'Authorization':'Basic '+credential})
                    return json.load(urllib.request.urlopen(req,timeout=4))
                s=get('/api/status');v=s.get('vehicle') or {}
                out['telemetry']={k:s.get(k) for k in ('uart_age_s','uart_error','camera_age_s','camera_error','servo_settings_synced','motor_busy')}
                out['vehicle']={k:v.get(k) for k in ('fw','state','reason','generation','web_manual_start','throttle_limit_pct','pca_ok','i2c_errors','voltage_v','current_a','speed_mps')}
                out['vehicle']['motor_a']=v.get('motor_a');out['vehicle']['motor_b']=v.get('motor_b')
                out['recording']=get('/api/record/status')
                try:
                    req=urllib.request.Request(base+'/camera.jpg',headers={'Authorization':'Basic '+credential})
                    with urllib.request.urlopen(req,timeout=4) as r:
                        b=r.read();out['camera_snapshot']={'status':r.status,'bytes':len(b),'jpeg':b[:2]==b'\xff\xd8'}
                except Exception as exc:out['camera_snapshot']={'error':type(exc).__name__}
            break
        except (OSError,urllib.error.URLError):continue
    else:out['checks'].append({'port':port,'reachable':False})
print(json.dumps(out,ensure_ascii=True,indent=2))
