"""Read-only live transport benchmark: only WebSocket probes and GET requests."""
import sys,pathlib,subprocess,base64,socket,json,time,threading,urllib.request,statistics
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'web/ws_vendor'))
from wsproto import WSConnection,ConnectionType
from wsproto.events import Request,TextMessage,CloseConnection
key=str(pathlib.Path.home()/'.ssh/codex_orin_ed25519')
token=subprocess.check_output(['ssh','-i',key,'-o','BatchMode=yes','jetson@orinnano-super.local','cat .config/orin_lab/token'],text=True).strip()
host=socket.gethostbyname('orinnano-super.local');authority=host+':8000';auth='Basic '+base64.b64encode(('jetson:'+token).encode()).decode()
def get(path):return json.load(urllib.request.urlopen(urllib.request.Request('http://'+authority+path,headers={'Authorization':auth}),timeout=4))
before=get('/api/status');v=before['vehicle'];assert v['state']=='LOCKED' and not v['steer_output_enabled'] and all(v[k]['pwm_command']==0 for k in ('motor_a','motor_b'))
result={'started':time.time(),'host':host,'before':v,'phases':[]}
for video in (False,True):
 done=threading.Event();video_bytes=[0];video_errors=[]
 def consume():
  try:
   with urllib.request.urlopen(urllib.request.Request('http://'+authority+'/camera.mjpg',headers={'Authorization':auth}),timeout=3) as r:
    while not done.is_set():video_bytes[0]+=len(r.read(4096))
  except Exception as e:video_errors.append(str(e))
 thread=None
 if video:thread=threading.Thread(target=consume,daemon=True);thread.start()
 ws=WSConnection(ConnectionType.CLIENT);samples=[];error=None
 try:
  with socket.create_connection((host,8000),timeout=2) as sock:
   sock.sendall(ws.send(Request(host=authority,target='/api/manual/ws',extra_headers=[(b'authorization',auth.encode()),(b'origin',('http://'+authority).encode())])))
   def receive():
    while True:
     raw=sock.recv(8192)
     if not raw:raise RuntimeError('socket closed')
     ws.receive_data(raw)
     for event in ws.events():
      if isinstance(event,TextMessage):return json.loads(event.data)
   hello=receive();token=hello['freshness'];previous=None
   for i in range(100):
    started=time.perf_counter()
    sock.sendall(ws.send(TextMessage(data=json.dumps({'id':i,'action':'probe','freshness':token,'client_send_ms':started*1000,'last_ack':previous}))))
    ack=receive();rtt=(time.perf_counter()-started)*1000;assert ack['ok'];token=ack['freshness']
    previous={'id':i,'rtt_ms':rtt,'server_processing_ms':ack['server_processing_ms'],'transport_and_browser_ms':max(0,rtt-ack['server_processing_ms'])}
    samples.append({**previous,'receive_gap_ms':ack['receive_gap_ms']})
    time.sleep(max(0,.2-(time.perf_counter()-started)))
   sock.sendall(ws.send(CloseConnection(code=1000)))
 except Exception as exc:error=str(exc)
 finally:
  done.set()
  if thread:thread.join(timeout=4)
 rtts=[s['rtt_ms'] for s in samples];phase={'video':video,'samples':samples,'error':error,'video_bytes':video_bytes[0],'video_errors':video_errors,'count':len(samples),'mean_rtt_ms':statistics.mean(rtts) if rtts else None,'max_rtt_ms':max(rtts) if rtts else None,'p95_rtt_ms':sorted(rtts)[int(len(rtts)*.95)-1] if rtts else None,'max_server_ms':max((s['server_processing_ms'] for s in samples),default=None)}
 result['phases'].append(phase);print(json.dumps({k:v for k,v in phase.items() if k!='samples'}),flush=True)
result['after']=get('/api/status')['vehicle'];result['ended']=time.time()
path=ROOT/'logs/websocket_live_validation_20261016.json';path.write_text(json.dumps(result,indent=2),encoding='utf-8')
v=result['after'];print(json.dumps({'state':v['state'],'motors':[v[k]['pwm_command'] for k in ('motor_a','motor_b')],'servo_enabled':v['steer_output_enabled']}))
assert v['state']=='LOCKED' and not v['steer_output_enabled'] and all(v[k]['pwm_command']==0 for k in ('motor_a','motor_b'))
assert all(p['count']==100 and not p['error'] for p in result['phases'])
