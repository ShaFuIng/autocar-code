import http.client,json
from types import SimpleNamespace
import base64,tempfile,threading,time,unittest,urllib.request,urllib.error
from pathlib import Path
from unittest.mock import patch
from test_dashboard import dashboard,ROOT

class CameraStreamTests(unittest.TestCase):
    def test_authenticated_stream_has_lengths_and_frame_sequence(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            home=Path(tmp);secret=home/'.config/orin_lab';secret.mkdir(parents=True);(secret/'token').write_text('test-password')
            with patch.object(dashboard.Path,'home',return_value=home),patch.object(dashboard,'jpeg',b'\xff\xd8test\xff\xd9'),patch.dict(dashboard.data,camera_at=time.monotonic(),camera_seq=123):
                server=dashboard.ThreadingHTTPServer(('127.0.0.1',0),dashboard.Handler)
                worker=threading.Thread(target=server.serve_forever);worker.start()
                base='http://127.0.0.1:'+str(server.server_port)
                try:
                    with self.assertRaises(urllib.error.HTTPError) as error:urllib.request.urlopen(base+'/camera.mjpg')
                    self.assertEqual(error.exception.code,401)
                    headers={'Authorization':'Basic '+base64.b64encode(b'jetson:test-password').decode()}
                    with urllib.request.urlopen(urllib.request.Request(base+'/camera.mjpg',headers=headers),timeout=3) as response:
                        self.assertIn('multipart/x-mixed-replace',response.headers['Content-Type'])
                        self.assertEqual(response.readline(),b'--frame\r\n')
                        parts={}
                        while True:
                            line=response.readline()
                            if line==b'\r\n':break
                            k,v=line.decode().strip().split(': ',1);parts[k]=v
                        self.assertEqual(parts['X-Frame-Seq'],'123')
                        self.assertEqual(response.read(int(parts['Content-Length'])),b'\xff\xd8test\xff\xd9')
                finally:
                    server.shutdown();worker.join();server.server_close()

    def test_stream_limit_does_not_block_control_status(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            home=Path(tmp);secret=home/'.config/orin_lab';secret.mkdir(parents=True);(secret/'token').write_text('test-password')
            with patch.object(dashboard.Path,'home',return_value=home),patch.object(dashboard,'stream_slots',threading.BoundedSemaphore(0)):
                server=dashboard.ThreadingHTTPServer(('127.0.0.1',0),dashboard.Handler)
                worker=threading.Thread(target=server.serve_forever);worker.start()
                base='http://127.0.0.1:'+str(server.server_port)
                headers={'Authorization':'Basic '+base64.b64encode(b'jetson:test-password').decode()}
                try:
                    with self.assertRaises(urllib.error.HTTPError) as error:urllib.request.urlopen(urllib.request.Request(base+'/camera.mjpg',headers=headers))
                    self.assertEqual(error.exception.code,429)
                    with urllib.request.urlopen(urllib.request.Request(base+'/api/status',headers=headers)) as response:self.assertEqual(response.status,200)
                finally:server.shutdown();worker.join();server.server_close()

    def test_control_connection_reused_while_camera_stream_open(self):
        # No hardware commands: exercise real sockets with a fake input manager.
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            home=Path(tmp);secret=home/'.config/orin_lab';secret.mkdir(parents=True);(secret/'token').write_text('test-password')
            accepted=[]
            fake=SimpleNamespace(input=lambda p:accepted.append(p) or {'ok':True})
            with patch.object(dashboard.Path,'home',return_value=home),patch.object(dashboard,'manual_drive',fake,create=True),patch.object(dashboard,'jpeg',b'jpeg'),patch.dict(dashboard.data,camera_at=time.monotonic(),camera_seq=456):
                server=dashboard.DashboardHTTPServer(('127.0.0.1',0),dashboard.Handler)
                worker=threading.Thread(target=server.serve_forever);worker.start()
                host='127.0.0.1:'+str(server.server_port)
                auth='Basic '+base64.b64encode(b'jetson:test-password').decode()
                headers={'Authorization':auth,'Content-Type':'application/json','X-Orin-Control':'1','Origin':'http://'+host}
                connection=http.client.HTTPConnection(host,timeout=3)
                try:
                    with urllib.request.urlopen(urllib.request.Request('http://'+host+'/camera.mjpg',headers={'Authorization':auth}),timeout=3) as stream:
                        stream.readline()
                        original=None
                        for seq in range(20):
                            connection.request('POST','/api/manual/input',json.dumps({'seq':seq}),headers)
                            response=connection.getresponse();self.assertEqual(response.status,200);self.assertTrue(json.loads(response.read())['ok'])
                            if original is None:original=connection.sock
                            self.assertIs(connection.sock,original,'Control requests should reuse a TCP connection')
                        self.assertEqual(len(accepted),20)
                    connection.request('POST','/api/manual/input','{}',{'Content-Type':'application/json'})
                    response=connection.getresponse();self.assertEqual(response.status,401);response.read()
                    self.assertIsNone(connection.sock,'Rejected auth must close an unread POST body')
                finally:connection.close();server.shutdown();worker.join();server.server_close()

    def test_browser_diagnostic_is_bounded_and_never_arms(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            with patch.object(dashboard.Path,'home',return_value=Path(tmp)),patch.object(dashboard,'manual_drive',None):
                for event in ('sample','input_error','preview_error','finished'):
                    self.assertTrue(dashboard.browser_diagnostic({'owner':'a'*32,'event':event,'seq':1})['ok'])
                for payload in ({'owner':'bad','event':'sample'},{'owner':'a'*32,'event':'start'},{'owner':'a'*32,'event':'sample','rtt_ms':float('nan')}):
                    with self.assertRaises(ValueError):dashboard.browser_diagnostic(payload)
                rows=(Path(tmp)/'orin_interface_v1/logs/browser_link.jsonl').read_text().splitlines()
                self.assertEqual(len(rows),4)
