"""SSH stdio transport. Credentials stay on Orin; no listeners, UART, or service restarts."""
import base64,json,os,re,socket,select,sys,urllib.request,urllib.error,subprocess
from pathlib import Path
SECRET=(Path.home()/'.config/orin_lab/token').read_text().strip()
AUTH='Basic '+base64.b64encode(('jetson:'+SECRET).encode()).decode()
ROOT=Path.home()/'.local/share/laptop_workbench/reviews';ROOT.mkdir(parents=True,exist_ok=True)
HOSTS={}
def permitted(method,path):
 p=path.split('?')[0]
 if p.startswith('/research/'):return method in ('GET','HEAD','POST','PUT','PATCH','DELETE')
 if p.startswith('/shadow/'):
  p=p[7:]
  return (method=='GET' and (p=='/api/runs' or bool(re.fullmatch(r'/runs/[A-Za-z0-9_-]{1,80}/(frames.json|manifest.json|reviews.jsonl|\d{6}\.(jpg|png))',p)))) or (method=='POST' and p in ['/api/analyze','/api/review'])
 if method=='GET':return p in ['/api/ops07/status','/api/pid/layers','/api/pid/status','/api/pid/frame','/api/status','/api/manual/status','/api/manual/ws','/api/motor/status','/camera.jpg','/camera.mjpg'] or bool(re.fullmatch(r'/api/(manual/files/[A-Za-z0-9_-]+|manual/file/[A-Za-z0-9_-]+/[A-Za-z0-9_.-]+|motor/result/[A-Za-z0-9_.-]+)',p))
 return method=='POST' and p in ['/api/ops07/load','/api/ops07/arm-hold','/api/ops07/release','/api/ops07/pause','/api/ops07/resume','/api/ops07/stop','/api/ops07/reset','/api/pid/preview-config','/api/pid/prepare','/api/pid/start','/api/pid/heartbeat','/api/pid/stop','/api/pid/preview-stop','/api/servo','/api/servo/settings','/api/motor/start','/api/motor/heartbeat','/api/motor/stop','/api/manual/start','/api/manual/input','/api/manual/settings','/api/manual/stop','/api/manual/record','/api/manual/diagnostic']
def host(port):
 if port in HOSTS:return HOSTS[port]
 for name in ['127.0.0.1']+subprocess.check_output(['hostname','-I'],text=True).split():
  if ':' in name:continue
  try:
   with socket.create_connection((name,port),timeout=2):pass
   HOSTS[port]=name;return name
  except OSError:pass
 raise OSError('Upstream unavailable')
def config(p):
 method,path=p['method'],p['path']
 if not permitted(method,path):raise ValueError('Endpoint not allowed')
 shadow=path.startswith('/shadow/');research=path.startswith('/research/');path=path[7:] if shadow else path;port=8889 if research else (8091 if shadow else 8000);name=host(port);base=f'http://{name}:{port}'
 headers={'Host':f'{name}:{port}','Authorization':AUTH,'Origin':base,'Content-Type':'application/json','Connection':'close'}
 if method=='POST':headers['X-Shadow' if shadow else 'X-Orin-Control']='1'
 if research:headers['Authorization']='token '+SECRET
 headers['Content-Type']=p.get('mime','application/json')
 return base,path,headers

def rpc(p):
 if p['path'].startswith('/ops07/observation/'):
  if p['method']!='GET' or not re.fullmatch(r'/ops07/observation/[0-9a-f]{32}',p['path']):raise ValueError('Read-only observation endpoint')
  # Bounded read of an existing log, no imports from vehicle code and no devices.
  import time
  run_id=p['path'].rsplit('/',1)[1]
  file=Path.home()/'orin_interface_v1/logs/manual_driving'/run_id/'pid.jsonl'
  out=dict(run_id=run_id,position=None,stop_reason=None)
  if file.is_file():
   with file.open('rb') as stream:
    stream.seek(0,2);size=stream.tell();stream.seek(max(0,size-262144))
    if size>262144:stream.readline()
    lines=stream.read(262144).splitlines()
   for line in reversed(lines):
    try:row=json.loads(line)
    except (ValueError,UnicodeDecodeError):continue
    if row.get('kind')=='stop' and out['stop_reason'] is None:out['stop_reason']=row.get('reason')
    if row.get('kind')=='sample':
     position=(row.get('proposal') or {}).get('track_pos')
     capture=(row.get('vision') or {}).get('capture_at')
     if isinstance(position,dict) and type(capture) in (int,float):
      age=time.monotonic()-capture
      if age>=0:out.update(position=position,sample_age_s=age,source='pid.jsonl',sample_seq=(row.get('vision') or {}).get('frame_seq'))
     break
  return dict(code=200,mime='application/json',body=base64.b64encode(json.dumps(out).encode()).decode())
 if p['path']=='/reviews':
  import fcntl
  with (ROOT/'sync.lock').open('w') as lock:
   fcntl.flock(lock,fcntl.LOCK_EX)
   if p['method']=='POST':
    row=json.loads(base64.b64decode(p['body']));uid=row.get('id','')
    if not re.fullmatch('[0-9a-f]{32}',uid) or not isinstance(row.get('notes'),str) or len(row['notes'])>5000:raise ValueError('Invalid review')
    f=ROOT/(uid+'.json')
    if f.exists() and json.loads(f.read_text())!=row:raise ValueError('Immutable review conflict')
    if not f.exists():
     t=f.with_suffix('.tmp');t.write_text(json.dumps(row,ensure_ascii=False));os.replace(t,f)
    data={'saved':True,'id':uid}
   else:data=[json.loads(f.read_text()) for f in ROOT.glob('*.json')]
   return dict(code=200,mime='application/json',body=base64.b64encode(json.dumps(data).encode()).decode())
 base,path,headers=config(p);body=base64.b64decode(p.get('body','')) if p['method'] not in ('GET','HEAD') else None
 try:
  with urllib.request.urlopen(urllib.request.Request(base+path,body,headers,method=p['method']),timeout=8) as r:
   data=r.read();mime=r.headers.get('Content-Type','application/octet-stream')
   if path.startswith('/research/') and ('html' in mime or 'json' in mime):data=data.replace(SECRET.encode(),b'')
   return dict(code=r.status,mime=mime,body=base64.b64encode(data).decode())
 except urllib.error.HTTPError as e:return dict(code=e.code,mime='application/json',body=base64.b64encode(e.read()).decode())
def stream(p):
 base,path,headers=config(p);port=8889 if path.startswith('/research/') else 8000;name=host(port);ws=bool(p.get('key'))
 if ws:
  headers.update({'Connection':'Upgrade','Upgrade':'websocket','Sec-WebSocket-Key':p['key'],'Sec-WebSocket-Version':'13'})
  if p.get('protocol'):headers['Sec-WebSocket-Protocol']=p['protocol']
 with socket.create_connection((name,port),timeout=3) as upstream:
  upstream.sendall(('GET '+path+' HTTP/1.1\r\n'+''.join(k+': '+v+'\r\n' for k,v in headers.items())+'\r\n').encode())
  while True:
   ready,_,_=select.select([upstream,sys.stdin.fileno()] if ws else [upstream],[],[],15)
   if not ready:return
   for src in ready:
    data=upstream.recv(65536) if src is upstream else os.read(sys.stdin.fileno(),65536)
    if not data:return
    if src is upstream:sys.stdout.buffer.write(data);sys.stdout.buffer.flush()
    else:upstream.sendall(data)
if sys.argv[-1]=='stream':stream(json.loads(sys.stdin.buffer.readline()))
else:
 for line in sys.stdin:
  try:out=rpc(json.loads(line))
  except Exception as e:out={'error':str(e)}
  print(json.dumps(out),flush=True)
