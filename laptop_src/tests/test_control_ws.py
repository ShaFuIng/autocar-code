import base64
import json
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
from test_dashboard import dashboard, ROOT
import control_ws
from wsproto import WSConnection, ConnectionType
from wsproto.events import Request, TextMessage, CloseConnection, AcceptConnection


class WebSocketTests(unittest.TestCase):
    def test_probes_and_reports_never_dispatch_or_extend_input_lease(self):
        dispatch=Mock();now=[10.];c=control_ws.Channel(dispatch,lambda:now[0])
        with patch.object(control_ws,'record'):
            for i in range(10):
                self.assertTrue(c.handle({'id':i,'action':'probe'})['ok']);now[0]+=.2
            c.handle({'id':11,'action':'report'})
        dispatch.assert_not_called();self.assertIsNone(c.owner_run)

    def test_old_token_and_cross_connection_token_rejected_stop_still_allowed(self):
        dispatch=Mock(return_value={'ok':True});now=[10.];c=control_ws.Channel(dispatch,lambda:now[0])
        token=c.stamp()['freshness'];now[0]+=.801
        with patch.object(control_ws,'record'):
            result=c.handle({'id':1,'action':'input','freshness':token})
            self.assertFalse(result['ok']);dispatch.assert_not_called()
            fresh=c.stamp()['freshness']
            other=control_ws.Channel(dispatch,lambda:now[0])
            self.assertFalse(other.handle({'id':2,'action':'start','freshness':fresh})['ok'])
            self.assertTrue(c.handle({'id':3,'action':'stop'})['ok'])
        dispatch.assert_called_once_with('stop',{})

    def test_socket_auth_origin_fragmentation_and_camera_isolation(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            home=Path(tmp);secret=home/'.config/orin_lab';secret.mkdir(parents=True);(secret/'token').write_text('test-password')
            manager=Mock();logs=[]
            with patch.object(dashboard.Path,'home',return_value=home),patch.object(dashboard,'manual_drive',manager),patch.object(control_ws,'record',side_effect=logs.append),patch.object(dashboard,'jpeg',b'jpeg'),patch.dict(dashboard.data,camera_at=time.monotonic(),camera_seq=1):
                server=dashboard.DashboardHTTPServer(('127.0.0.1',0),dashboard.Handler)
                thread=threading.Thread(target=server.serve_forever);thread.start()
                host='127.0.0.1:'+str(server.server_port)
                auth=b'Basic '+base64.b64encode(b'jetson:test-password')
                def connect(origin,authorized=True):
                    sock=socket.create_connection(server.server_address,timeout=2);ws=WSConnection(ConnectionType.CLIENT)
                    headers=[(b'origin',origin.encode())]
                    if authorized:headers.append((b'authorization',auth))
                    sock.sendall(ws.send(Request(host=host,target='/api/manual/ws',extra_headers=headers)))
                    return sock,ws
                try:
                    for origin,authorized,code in [('http://'+host,False,b'401'),('http://evil.test',True,b'403')]:
                        sock,ws=connect(origin,authorized)
                        with sock:self.assertIn(code,sock.recv(4096).split(b'\r\n')[0])
                    # Leave an HTTP image stream open while using a separate control socket.
                    camera=socket.create_connection(server.server_address,timeout=2)
                    camera.sendall(b'GET /camera.mjpg HTTP/1.1\r\nHost: '+host.encode()+b'\r\nAuthorization: '+auth+b'\r\n\r\n')
                    self.assertIn(b'200',camera.recv(4096))
                    sock,ws=connect('http://'+host)
                    with sock,camera:
                        def message():
                            while True:
                                ws.receive_data(sock.recv(4096))
                                for event in ws.events():
                                    if isinstance(event,TextMessage):return json.loads(event.data)
                        hello=message();self.assertEqual(hello['type'],'hello')
                        for i in range(20):
                            p=json.dumps({'id':i,'action':'probe','freshness':hello['freshness']})
                            sock.sendall(ws.send(TextMessage(data=p[:10],message_finished=False)))
                            sock.sendall(ws.send(TextMessage(data=p[10:])))
                            hello=message();self.assertTrue(hello['ok']);self.assertEqual(hello['id'],i)
                        sock.sendall(ws.send(CloseConnection(code=1000)))
                    time.sleep(.05)
                    manager.input.assert_not_called();manager.start.assert_not_called();manager.stop_for_page.assert_not_called()
                    self.assertEqual(len([r for r in logs if r.get('action')=='probe']),20)
                finally:server.shutdown();thread.join();server.server_close()


if __name__=='__main__':unittest.main()
