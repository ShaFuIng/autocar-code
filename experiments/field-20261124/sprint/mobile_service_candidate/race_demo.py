"""laptop demo of the r3 race start mode (NOT deployed to the Orin): fake car control + fake organiser flag + the real mobile Handler.
python race_demo.py   ->  page http://127.0.0.1:18092
fake flag (same behaviour as fake_go.py in CAR-INTEGRATION.md): POST http://127.0.0.1:18080/start (1 after 3 s), POST /stop (0).
Nothing here talks to a real car: the start request lands in the fake car control, which only records it."""
import json, tempfile, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import server as S
import test_server as T

go_at = [None]


class Go(BaseHTTPRequestHandler):
    def _send(self, body):
        self.send_response(200); self.send_header('Content-Type', 'text/plain'); self.send_header('Content-Length', str(len(body))); self.end_headers(); self.wfile.write(body)
    def do_GET(self): self._send(b'1' if go_at[0] and time.time() >= go_at[0] else b'0')
    def do_POST(self):
        go_at[0] = time.time() + 3 if self.path == '/start' else None; self._send(b'ok')
    def log_message(self, *a): pass


class Car(T.Fake):
    def do_POST(self):
        super().do_POST()
        if self.path == '/api/pid/start':   # pretend the run started so the page shows it
            T.CAR['pid']['run'] = dict(id='demo' + str(len(T.CAR['posts'])), state='running', seconds=None)
        elif self.path == '/api/pid/stop' and T.CAR['pid']['run']:
            T.CAR['pid']['run'] = dict(T.CAR['pid']['run'], state='stopped', reason='USER_STOP')


def main():
    tmp = Path(tempfile.mkdtemp()); (tmp / 'token').write_text(T.TOK); S.TOKEN = tmp / 'token'
    (tmp / 'service.json').write_text(json.dumps({'login': False, 'imu_auto_restart': False, 'race_go_url': 'http://127.0.0.1:18080/go'})); S.SERVICE = tmp / 'service.json'
    S.addresses = lambda: ['127.0.0.1']; S.UPSTREAM_PORT = 18000; S.ACCESS = tmp / 'access.log'
    for port, h in ((18000, Car), (18080, Go), (18092, S.Handler)):
        s = ThreadingHTTPServer(('127.0.0.1', port), h); s.daemon_threads = True; threading.Thread(target=s.serve_forever, daemon=True).start()
    threading.Thread(target=S.race_loop, daemon=True).start()
    print('race demo: page http://127.0.0.1:18092  flag http://127.0.0.1:18080  log', S.ACCESS, flush=True)
    while True: time.sleep(60)


if __name__ == '__main__':
    main()
