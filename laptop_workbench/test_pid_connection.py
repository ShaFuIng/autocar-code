import io,json,tempfile,unittest,threading,time
from pathlib import Path
from types import SimpleNamespace
from integration import Integration
from server import App

class HeartbeatTransportTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory(dir=Path(__file__).parent / 'data');self.app=SimpleNamespace(store=SimpleNamespace(root=Path(self.temp.name)))
  self.i=Integration(self.app);self.app.integration=self.i;self.i.ensure=lambda:None
 def tearDown(self):self.temp.cleanup()
 def fake(self):
  return SimpleNamespace(stdin=io.BytesIO(),stdout=io.BytesIO(b'{"code":200,"body":"e30="}\n'),poll=lambda:None,terminate=lambda:None)
 def test_heartbeat_and_stop_bypass_exhausted_general_pool(self):
  for _ in range(4):self.i.pool.get_nowait()
  self.i.process=lambda mode:self.fake()
  self.assertEqual(self.i.request('/api/pid/heartbeat',dict(run_id='a',seq=1)),{})
  self.assertEqual(self.i.request('/api/pid/stop',{}),{})
  self.assertEqual(self.i.pool.qsize(),0)
  log=json.loads((self.app.store.root/'pid-connection.jsonl').read_text())
  self.assertEqual(log['seq'],1);self.assertEqual(log['http_code'],200)
 def test_stalled_heartbeat_terminated_no_retry(self):
  release=threading.Event();calls=[]
  p=self.fake();p.stdout=SimpleNamespace(readline=lambda:(release.wait(2) or b''));p.terminate=release.set
  def make(mode):calls.append(mode);return p
  self.i.process=make;t=time.monotonic()
  with self.assertRaises(ValueError):self.i.request('/api/pid/heartbeat',dict(seq=1))
  self.assertLess(time.monotonic()-t,1);self.assertEqual(calls,['rpc']);self.assertIsNone(self.i.heartbeat_pool.get_nowait())
 def test_client_diagnostics_append_and_validation(self):
  App.post(self.app,'/api/pid/connection-event',dict(event='blur',run_id='a',client_time=1))
  App.post(self.app,'/api/pid/connection-event',dict(event='heartbeat_ack',seq=1,rtt_ms=42))
  rows=App.get(self.app,'/api/pid/connection-log');self.assertEqual(len(rows),2)
  self.assertEqual(rows[0]['rtt_ms'],42)
  with self.assertRaises(ValueError):App.post(self.app,'/api/pid/connection-event',dict(event='heartbeat_ack',rtt_ms=float('nan')))
 def test_probe_is_readonly(self):
  calls=[]
  self.i.request=lambda path,**kw:(calls.append((path,kw)) or {'state':'ready'})
  r=App.post(self.app,'/api/pid/heartbeat-probe',{})
  self.assertTrue(r['ok']);self.assertEqual(calls,[('/api/pid/status',{'channel':'heartbeat'})]*2)
if __name__=='__main__':unittest.main()

