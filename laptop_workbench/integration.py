"""SSH stdio RPC plus byte relay; does not require SSH TCP forwarding."""
import base64,hashlib,json,socket,subprocess,threading,queue,os,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
class Integration:
 def __init__(self,app):
  self.app=app;self.lock=threading.Lock();self.remote=None;self.pool=queue.LifoQueue()
  for _ in range(4):self.pool.put(None)
  self.heartbeat_pool=queue.LifoQueue();self.heartbeat_pool.put(None)
  self.stop_pool=queue.LifoQueue();self.stop_pool.put(None)
  self.diagnostic_lock=threading.Lock()
 def diagnostic(self,event):
  with self.diagnostic_lock:
   with (self.app.store.root/'pid-connection.jsonl').open('a',encoding='utf-8') as f:
    f.write(json.dumps(dict(event,server_time=time.time()),ensure_ascii=False)+'\n')
 def ensure(self):
  if self.remote:return
  with self.lock:
   if self.remote:return
   s=self.app.orin.status()
   if not s.get('connected'):raise ValueError(s.get('error','Orin unavailable'))
   source=(ROOT/'integration_remote.py').read_bytes();digest=hashlib.sha256(source).hexdigest()[:16];remote='/home/jetson/.local/share/laptop_workbench/integration-'+digest+'.py'
   self.ssh=self.app.orin.ssh(self.app.orin.host)
   code="import pathlib,sys;p=pathlib.Path('"+remote+"');p.write_bytes(sys.stdin.buffer.read())"
   r=subprocess.run(self.ssh+['python3 -c "'+code+'"'],input=source,capture_output=True,timeout=15)
   if r.returncode:raise ValueError('Cannot prepare integration transport')
   self.remote=remote
 def process(self,mode):
  self.ensure();return subprocess.Popen(self.ssh[:-1]+['-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2',self.ssh[-1],'python3 -u '+self.remote+' '+mode],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(self.app.store.root/'integration-transport.log').open('ab'))
 def raw(self,path,body=None,method=None,mime="application/json",channel=None):
  self.ensure()
  heartbeat=path=='/api/pid/heartbeat'
  lane='heartbeat' if heartbeat or channel=='heartbeat' else 'stop' if path in ('/api/pid/stop','/api/pid/preview-stop','/api/ops07/pause','/api/ops07/stop') else 'general'
  pool=self.heartbeat_pool if lane=='heartbeat' else self.stop_pool if lane=='stop' else self.pool
  started=time.monotonic();proc=pool.get(timeout=.15 if heartbeat else 12);acquired=time.monotonic()
  try:
   if proc is None or proc.poll() is not None:proc=self.process('rpc')
   p=dict(path=path,method=method or ('GET' if body is None else 'POST'),mime=mime,body=base64.b64encode(body or b'').decode());proc.stdin.write((json.dumps(p)+'\n').encode());proc.stdin.flush()
   replies=[];t=threading.Thread(target=lambda:replies.append(proc.stdout.readline()),daemon=True);t.start();t.join(.65 if heartbeat else 12)
   if not replies or not replies[0]:raise ValueError('Orin acknowledgment unknown; do not retry motion automatically')
   d=json.loads(replies[0]);
   if 'error' in d:raise ValueError(d['error'])
   return d
  except Exception:
   if proc:proc.terminate()
   proc=None;self.remote=None;raise
  finally:
   pool.put(proc)
   if lane=='heartbeat':
    try:
     payload=json.loads(body or b'{}')
     self.diagnostic(dict(kind='transport',path=path,run_id=payload.get('run_id'),seq=payload.get('seq'),queue_ms=round((acquired-started)*1000,2),total_ms=round((time.monotonic()-started)*1000,2),transport_ok=proc is not None,http_code=d.get('code') if 'd' in locals() else None))
    except (OSError,ValueError):pass
 def request(self,path,payload=None,channel=None):
  d=self.raw(path,None if payload is None else json.dumps(payload).encode(),channel=channel);body=base64.b64decode(d['body'])
  if d['code']>=400:raise ValueError(body.decode(errors='replace')[:1000])
  return json.loads(body)
 def relay(self,handler,path,body):
  if handler.headers.get('Upgrade','').lower()!='websocket' and path.split('?')[0]!='/camera.mjpg':
   d=self.raw(path,body if handler.command not in ('GET','HEAD') else None,handler.command,handler.headers.get('Content-Type','application/json'));return handler.send(base64.b64decode(d['body']),d['code'],d['mime'])
  ws=handler.headers.get('Upgrade','').lower()=='websocket';proc=self.process('stream');p=dict(path=path,method='GET',key=handler.headers.get('Sec-WebSocket-Key',''),protocol=handler.headers.get('Sec-WebSocket-Protocol',''))
  proc.stdin.write((json.dumps(p)+'\n').encode());proc.stdin.flush();handler.close_connection=True
  if ws:
   def incoming():
    try:
     while True:
      chunk=handler.connection.recv(65536)
      if not chunk:break
      proc.stdin.write(chunk);proc.stdin.flush()
    except OSError:pass
    finally:
     try:proc.stdin.close()
     except OSError:pass
   threading.Thread(target=incoming,daemon=True).start()
  try:
   while True:
    chunk=os.read(proc.stdout.fileno(),65536)
    if not chunk:break
    handler.connection.sendall(chunk)
  except OSError:pass
  finally:
   proc.terminate()
 def sync_reviews(self):
  store=self.app.store
  with store.lock:local=[json.loads(r[0]) for r in store.db.execute("SELECT body FROM documents WHERE key LIKE 'analysis-review-%'")]
  for row in local:self.request('/reviews',row)
  imported=0;conflicts=[]
  for row in self.request('/reviews'):
   key='analysis-review-'+row['job']+'-'+row['id'];current=store.get(key)
   if current['rev'] and current['data']!=row:conflicts.append(row['id']);continue
   if not current['rev']:store.save(key,0,row);imported+=1
  return dict(uploaded=len(local),imported=imported,conflicts=conflicts,storage='筆電 SQLite + Orin 獨立評語庫；原始評語不覆蓋')

 def stop_all(self):
  from concurrent.futures import ThreadPoolExecutor
  calls=[('/api/servo',dict(action='stop')),('/api/manual/stop',dict(reason='BUTTON_STOP')),('/api/motor/stop',{})]
  def one(call):
   try:return dict(endpoint=call[0],result=self.request(*call))
   except Exception as e:return dict(endpoint=call[0],error=str(e))
  with ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(one,calls))
  return dict(results=results,confirmed=all('error' not in r and r['result'].get('ok',True) for r in results))
