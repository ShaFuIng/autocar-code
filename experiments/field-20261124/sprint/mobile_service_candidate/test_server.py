"""mobile_service tests (laptop, standard library): a fake dashboard on 127.0.0.1 + the real Handler. python -m unittest -v test_server"""
import base64, json, tempfile, threading, time, unittest, urllib.request, urllib.error
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import server as S

TOK = 'test-token-not-real'
AUTH = 'Basic ' + base64.b64encode(('jetson:' + TOK).encode()).decode()
CAR = dict(pid=dict(state='ready', ready_frames=9, pause_resume_version=1, run=None, pause=dict(state='idle', can_resume=False), timing=None, remaining_s=0),
           veh=dict(uart_error=None, servo_settings=dict(revision=43, min_us=780, center_us=1475, max_us=2050),
                    vehicle=dict(fw='orin-test', state='LOCKED', reason='REMOTE_STOP', pca_ok=True, pins=dict(D11=1))), posts=[], next=None,
           go=b'0', go_reads=0, go_paths='', queue=[])
PREPARE = dict(model_id='v4-default', path_algorithm='camera-pair-v2-junction-flag', row_ratio=0.6, visual_center_offset_px_640=18, curve_outward_px_640=0)


class Fake(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def out(self, code, obj):
        d = json.dumps(obj).encode(); self.send_response(code); self.send_header('Content-Length', str(len(d))); self.end_headers(); self.wfile.write(d)

    def do_GET(self):
        if self.path.startswith('/go'):   # r3: the race organiser's start flag (no login)
            CAR['go_reads'] += 1; CAR['go_paths'] = self.path
            if CAR['go'] is None: return self.out(500, {})
            self.send_response(200); self.send_header('Content-Length', str(len(CAR['go']))); self.end_headers(); return self.wfile.write(CAR['go'])
        if self.headers.get('Authorization') != AUTH: return self.out(401, {})
        if self.path == '/api/pid/status': return self.out(200, CAR['pid'])
        if self.path == '/api/status': return self.out(200, CAR['veh'])
        if self.path == '/api/pid/mask':
            return self.out(200, dict(run_id=None, seq=1, valid=True, age_s=.05, mask=dict(png_base64='AA==', width=640, height=360), path=dict(points=[[1, 2]], width=1280, height=720)))
        self.out(404, {})

    def do_POST(self):
        if self.headers.get('Authorization') != AUTH or self.headers.get('X-Orin-Control') != '1' or self.headers.get('Origin') != 'http://' + self.headers.get('Host'):
            return self.out(403, {})
        body = json.loads(self.rfile.read(int(self.headers['Content-Length']))); CAR['posts'].append((self.path, body))
        if CAR['queue'] and self.path == '/api/pid/start': return self.out(*CAR['queue'].pop(0))
        code, r = CAR['next'] or (200, dict(ok=True, run_id='a' * 32)); CAR['next'] = None; self.out(code, r)


class T(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(); tok = Path(cls.tmp.name) / 'token'; tok.write_text(TOK + '\n'); S.TOKEN = tok
        S.SERVICE = Path(cls.tmp.name) / 'absent-service.json'   # the tests start from "login required", whatever service.json sits next to server.py
        cls.fake = ThreadingHTTPServer(('127.0.0.1', 0), Fake); S.UPSTREAM_PORT = cls.fake.server_address[1]; S.addresses = lambda: ['127.0.0.1']
        cls.svc = ThreadingHTTPServer(('127.0.0.1', 0), S.Handler); cls.base = 'http://127.0.0.1:%d' % cls.svc.server_address[1]
        for s in (cls.fake, cls.svc): threading.Thread(target=s.serve_forever, daemon=True).start()
        threading.Thread(target=S.race_loop, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.fake.shutdown(); cls.svc.shutdown(); cls.tmp.cleanup()

    def setUp(self):
        CAR['posts'].clear(); CAR['next'] = None; CAR['pid'].update(state='ready', run=None); CAR['veh']['servo_settings']['center_us'] = 1475
        S.RACE.disarm(); S.RACE = S.RaceGo(); CAR.update(go=b'0', go_reads=0, queue=[]); CAR['veh']['vehicle']['pins']['D11'] = 1

    def call(self, path, body=None, auth=AUTH, headers=None):
        h = {'Authorization': auth} if auth else {}
        if body is not None: h.update({'Content-Type': 'application/json', 'X-Orin-Control': '1'})
        h.update(headers or {})
        req = urllib.request.Request(self.base + path, data=None if body is None else json.dumps(body).encode(), headers=h)
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                raw = r.read(); return r.status, (json.loads(raw) if r.headers.get('Content-Type', '').startswith('application/json') else raw)
        except urllib.error.HTTPError as e:
            raw = e.read(); return e.code, (json.loads(raw) if raw else None)

    def test_login_required_everywhere(self):
        for p in ('/', '/app.js', '/api/pid/status', '/api/pid/mask'):
            self.assertEqual(self.call(p, auth=None)[0], 401)
        wrong = 'Basic ' + base64.b64encode(b'jetson:wrong').decode()
        self.assertEqual(self.call('/api/pid/stop', {}, auth=wrong)[0], 401)
        self.assertEqual(CAR['posts'], [])

    def test_open_access_switch(self):
        cfg = Path(self.tmp.name) / 'service.json'; old = S.SERVICE; S.SERVICE = cfg
        try:
            self.assertEqual(self.call('/', auth=None)[0], 401)                     # no file: login required
            cfg.write_text('{"login": true}'); self.assertEqual(self.call('/api/pid/status', auth=None)[0], 401)
            cfg.write_text('not json'); self.assertEqual(self.call('/api/pid/status', auth=None)[0], 401)
            cfg.write_text('{"login": false}')
            self.assertEqual(self.call('/', auth=None)[0], 200)
            code, s = self.call('/api/pid/status', auth=None); self.assertEqual((code, s['centre_us']), (200, 1475)); self.assertNotIn(TOK, json.dumps(s))
            self.assertEqual(self.call('/api/pid/stop', {}, auth=None)[0], 200); self.assertEqual(CAR['posts'][-1], ('/api/pid/stop', {}))
            self.assertEqual(self.call('/api/pid/stop', {}, auth=None, headers={'X-Orin-Control': '0'})[0], 403)      # the POST rules stay
            self.assertEqual(self.call('/api/pid/stop', {}, auth=None, headers={'Origin': 'http://evil.example'})[0], 403)
            self.assertEqual(self.call('/api/servo', {}, auth=None)[0], 404)
        finally:
            S.SERVICE = old

    def test_imu_watch(self):
        w = S.ImuWatch(); t = 100.
        def feed(n, gyro, pitch=1.3, dt=.5, age=.02):
            nonlocal t
            for i in range(n):
                t += dt; st = w.judge(dict(t=t - age, gyro_dps=gyro(i), pitch_deg=pitch, roll_deg=1.1), t)
            return st
        self.assertEqual(feed(10, lambda i: [0.01 * i, 0.02, -0.01]), 'ok')            # live sensor: the last digit moves
        self.assertEqual(feed(3, lambda i: [-7.924, 1.303, 0.822], pitch=-1.28), 'ok')   # 1.5 s without a change: not yet
        self.assertEqual(feed(2, lambda i: [-7.924, 1.303, 0.822], pitch=-1.28), 'frozen')   # 18:10 signature: constant values, time advancing
        s = w.snapshot(t); self.assertEqual(s['state'], 'frozen'); self.assertGreaterEqual(s['bad_s'], 0.)
        self.assertEqual(feed(1, lambda i: [0.03, 0.01, 0.0]), 'ok'); self.assertIsNone(w.snapshot(t)['bad_s'])
        self.assertEqual(w.judge(dict(t=t - 5., gyro_dps=[0.5, 0, 0], pitch_deg=1., roll_deg=1.), t), 'stale')   # logger stopped
        self.assertEqual(w.judge(None, t), 'missing')
        w.forget(); self.assertEqual(w.snapshot(t)['state'], 'unknown')

    def test_imu_fault_in_view(self):
        ok = dict(state='ok', bad_s=None); bad = dict(state='frozen', bad_s=4.2); blip = dict(state='stale', bad_s=.4)
        self.assertIsNone(S.view(dict(state='ready'), {}, ok)['fault'])
        self.assertEqual(S.view(dict(state='ready'), {}, bad)['fault'], 'IMU_FAULT')
        self.assertIsNone(S.view(dict(state='ready'), {}, blip)['fault'])                 # shorter than 1 s: no fault yet
        self.assertEqual(S.view(dict(state='ready'), dict(vehicle=dict(pins=dict(D11=0))), bad)['fault'], 'ESTOP')   # e-stop first
        self.assertIsNone(S.view(dict(state='ready'), {})['fault'])                       # no IMU information: as before
        self.assertEqual(S.view(dict(state='ready'), {}, bad)['imu'], bad)

    def test_repairs(self):
        calls = []; old = S.restart_unit; S.restart_unit = lambda unit: calls.append(unit)
        try:
            self.assertEqual(self.call('/api/repair/imu', {})[0], 200); self.assertEqual(calls, ['orin-imu-logger.service'])
            self.assertEqual(self.call('/api/repair/model', {})[0], 200)
            self.assertEqual([p for p, _ in CAR['posts'][-2:]], ['/api/pid/preview-stop', '/api/pid/prepare']); self.assertEqual(CAR['posts'][-1][1], PREPARE)
            CAR['pid']['run'] = dict(id='r', state='running'); n = len(CAR['posts'])
            code, r = self.call('/api/repair/imu', {}); self.assertEqual((code, r['reason']), (409, 'RUN_ACTIVE'))
            code, r = self.call('/api/repair/model', {}); self.assertEqual((code, r['reason']), (409, 'RUN_ACTIVE'))
            self.assertEqual((calls, len(CAR['posts'])), (['orin-imu-logger.service'], n))   # nothing restarted, nothing sent during a run
            self.assertEqual(self.call('/api/repair/dashboard', {})[0], 404)                 # the car control itself cannot be restarted from the page
            self.assertEqual(self.call('/api/repair/imu', {}, headers={'X-Orin-Control': '0'})[0], 403)
        finally:
            S.restart_unit = old

    def test_auto_restart_rule(self):
        calls = []; old = (S.restart_unit, S.IMU, S.SERVICE); S.restart_unit = lambda unit: calls.append(unit); S.IMU = S.ImuWatch()
        cfg = Path(self.tmp.name) / 'auto.json'; S.SERVICE = cfg
        def step(now, d):   # one iteration of the rule in imu_loop
            st = S.IMU.judge(d, now)
            if (st in ('frozen', 'stale', 'missing') and S.setting('imu_auto_restart', True) is not False and S.IMU.bad_since is not None
                    and now - S.IMU.bad_since >= S.AUTO_AFTER_S and (S.IMU.auto_at is None or now - S.IMU.auto_at >= S.AUTO_GAP_S)):
                S.IMU.auto_at = now; return S.imu_restart('auto')
            return None
        try:
            t = 50.; frozen = lambda t: dict(t=t - .02, gyro_dps=[-7.924, 1.303, 0.822], pitch_deg=-1.28, roll_deg=-7.76)
            out = []
            for i in range(14): t += .5; out.append(step(t, frozen(t)))
            self.assertEqual(calls, ['orin-imu-logger.service']); self.assertEqual([o for o in out if o], [(True, None)])   # once, about 5 s after it froze
            CAR['pid']['run'] = dict(id='r', state='running'); S.IMU = S.ImuWatch(); calls.clear()
            for i in range(14): t += .5; r = step(t, frozen(t))
            self.assertEqual(calls, [])                                                    # never during a run
            CAR['pid']['run'] = None; cfg.write_text('{"imu_auto_restart": false}'); S.IMU = S.ImuWatch()
            for i in range(14): t += .5; step(t, frozen(t))
            self.assertEqual(calls, [])                                                    # switched off
        finally:
            S.restart_unit, S.IMU, S.SERVICE = old

    def test_static_and_no_token_leak(self):
        code, page = self.call('/')
        self.assertEqual(code, 200); self.assertIn('跑車操作'.encode(), page); self.assertNotIn('模擬'.encode(), page)
        for p in ('/', '/app.js', '/model.mjs', '/style.css'):
            self.assertNotIn(TOK.encode(), self.call(p)[1])
        for p in ('/server.py', '/start_params.json', '/../server.py', '/web/index.html'):
            self.assertEqual(self.call(p)[0], 404)

    def test_status_view(self):
        code, s = self.call('/api/pid/status')
        self.assertEqual(code, 200)
        self.assertEqual((s['model_ready'], s['path_ready'], s['run'], s['fault'], s['centre_us']), (True, True, None, None, 1475))
        self.assertNotIn(TOK, json.dumps(s))
        ended = dict(state='no_reliable_path', run=dict(id='r', state='stopped', reason='PHYSICAL_ESTOP', seconds=None), pause=dict(state='ended'),
                     timing=dict(active_s=130.2, total_s=140., laps=[dict(lap=1, active_s=60.4, total_s=60.4)], current_lap_active_s=5.5))
        v = S.view(ended, dict(vehicle=dict(pins=dict(D11=0))))
        self.assertEqual((v['model_ready'], v['path_ready'], v['run']['state'], v['run']['reason'], v['fault']), (True, False, 'ended', 'PHYSICAL_ESTOP', 'ESTOP'))
        self.assertEqual((v['timing']['laps'], v['timing']['current_lap_s']), ([dict(active_s=60.4, total_s=60.4)], 5.5))
        running = dict(state='ready', run=dict(id='r', state='running', seconds=None), timing=dict(active_s=3.2, total_s=3.2, laps=[], current_lap_active_s=3.2))
        v = S.view(running, dict(vehicle=dict(pca_ok=False)))
        self.assertEqual((v['run']['state'], v['timing']['current_lap_s'], v['fault']), ('running', None, 'FAULT'))
        self.assertEqual(S.view(dict(state='idle'), {})['model_ready'], False)
        self.assertEqual(S.view(dict(state='failed'), {})['fault'], 'MODEL_FAILED')

    def test_post_rules(self):
        self.assertEqual(self.call('/api/pid/stop', {}, headers={'X-Orin-Control': '0'})[0], 403)
        self.assertEqual(self.call('/api/pid/stop', {}, headers={'Origin': 'http://evil.example'})[0], 403)
        self.assertEqual(self.call('/api/pid/stop', {'x': 'y' * 3000})[0], 409)
        for p in ('/api/servo', '/api/servo/settings', '/api/motor/start', '/api/manual/start', '/api/pid/preview-stop', '/api/pid/preview-config'):
            self.assertEqual(self.call(p, {})[0], 404)
        self.assertEqual(CAR['posts'], [])

    def test_prepare(self):
        self.assertEqual(self.call('/api/pid/prepare', {})[1], dict(ok=True, already=True))
        self.assertEqual(CAR['posts'], [])   # already loaded: nothing sent to the car
        CAR['pid']['state'] = 'idle'
        self.assertEqual(self.call('/api/pid/prepare', {})[0], 200)
        self.assertEqual(CAR['posts'][-1], ('/api/pid/prepare', PREPARE))

    def test_start_and_stop(self):
        CAR['veh']['servo_settings']['center_us'] = 1480
        code, r = self.call('/api/pid/start', {'parameters': {'kp': 9999}, 'seconds': 5})   # nothing from the page reaches the car's start body
        self.assertEqual((code, r['run_id']), (200, 'a' * 32))
        path, b = CAR['posts'][-1]
        self.assertEqual(path, '/api/pid/start')
        self.assertEqual((b['execution_mode'], b['seconds'], b['area_clear'], b['parameter_revision'], b['path_algorithm']),
                         ('orin_timed', None, True, 154, 'camera-pair-v2-junction-flag'))
        self.assertRegex(b['owner'], '^[0-9a-f]{32}$')
        self.assertEqual((b['parameters']['kp'], b['parameters']['center_us'], b['parameters']['min_us'], len(b['parameters'])), (220, 1480, 780, 31))
        owner1 = b['owner']; self.call('/api/pid/start', {})
        self.assertNotEqual(CAR['posts'][-1][1]['owner'], owner1)
        CAR['next'] = (409, dict(ok=False, reason='Need 3 consecutive fresh valid model results'))
        code, r = self.call('/api/pid/start', {})
        self.assertEqual((code, r['reason']), (409, 'NO_FRESH_PATH')); self.assertIn('fresh valid', r['reason_text'])
        self.assertEqual(self.call('/api/pid/stop', {'anything': 1})[0], 200)
        self.assertEqual(CAR['posts'][-1], ('/api/pid/stop', {}))

    def test_pause_resume(self):
        self.assertEqual(self.call('/api/pid/pause', {'run_id': 'r1', 'extra': 1})[0], 200)
        self.assertEqual(CAR['posts'][-1], ('/api/pid/pause', {'run_id': 'r1'}))
        CAR['next'] = (409, dict(ok=False, reason='目前在上層', code='UPPER_DECK', pause=dict(state='paused', can_resume=False)))
        code, r = self.call('/api/pid/resume', {'run_id': 'r1'})
        self.assertEqual((code, r['reason'], r['reason_text'], r['pause']['state']), (409, 'UPPER_DECK', '目前在上層', 'paused'))

    def test_mask_and_unreachable(self):
        code, m = self.call('/api/pid/mask')
        self.assertEqual((code, m['mask']['width'], m['path']['width']), (200, 640, 1280))
        port = S.UPSTREAM_PORT; S.UPSTREAM_PORT = 1; S._base[0] = None
        try:
            self.assertEqual(self.call('/api/pid/status')[0], 502)
            code, r = self.call('/api/pid/stop', {})
            self.assertEqual((code, r['reason']), (502, 'CAR_UNREACHABLE'))
        finally:
            S.UPSTREAM_PORT = port

    # ------------------------------------------------------------------ r3 race start flag
    def race_cfg(self, practice=False):
        cfg = Path(self.tmp.name) / 'race.json'
        cfg.write_text(json.dumps({'login': False, 'race_go_url': 'http://127.0.0.1:%d/go' % S.UPSTREAM_PORT, 'race_practice': practice}))
        old = S.SERVICE; S.SERVICE = cfg; self.addCleanup(lambda: setattr(S, 'SERVICE', old))

    def wait(self, cond, s=2.):
        end = time.monotonic() + s
        while time.monotonic() < end:
            if cond(): return True
            time.sleep(.02)
        return False

    def starts(self): return [b for p, b in CAR['posts'] if p == '/api/pid/start']

    def test_race_edge_rule(self):
        g = S.RaceGo()
        self.assertFalse(g.step(1)); self.assertFalse(g.step(0))           # not armed: nothing
        g.arm(); self.assertFalse(g.step(1)); self.assertFalse(g.step(1))  # flag already 1 when armed: no start
        self.assertFalse(g.step(0)); self.assertFalse(g.step(None))        # unreadable keeps saw_zero
        self.assertTrue(g.step(1)); self.assertFalse(g.armed)              # 0 -> 1: start, race mode already off
        self.assertFalse(g.step(0)); self.assertFalse(g.step(1))           # one-shot
        self.assertFalse(g.arm())                                          # still firing until finish()
        g.finish(dict(ok=True)); self.assertTrue(g.arm())

    def test_race_green_starts_once(self):
        self.race_cfg()
        time.sleep(.3); self.assertEqual(CAR['go_reads'], 0)              # not armed: the flag is never read
        code, r = self.call('/api/race/arm', {}); self.assertEqual((code, r['race']['armed']), (200, True))
        self.assertTrue(self.wait(lambda: S.RACE.snapshot()['saw_zero'])); time.sleep(.3)
        self.assertEqual(self.starts(), [])
        self.assertGreaterEqual(CAR['go_reads'], 5)                         # about every 50 ms
        CAR['go'] = b'1'
        self.assertTrue(self.wait(lambda: (S.RACE.snapshot()['result'] or {}).get('ok')))
        st = self.starts(); self.assertEqual(len(st), 1)
        self.assertEqual((st[0]['execution_mode'], st[0]['seconds'], st[0]['parameters']['center_us']), ('orin_timed', None, 1475))
        g = self.call('/api/pid/status')[1]['race']
        self.assertEqual((g['armed'], g['result']['run_id'], g['result']['tries']), (False, 'a' * 32, 1)); self.assertLess(g['result']['latency_ms'], 1000)
        reads = CAR['go_reads']; CAR['go'] = b'0'; time.sleep(.2); CAR['go'] = b'1'; time.sleep(.3)
        self.assertEqual((CAR['go_reads'], len(self.starts())), (reads, 1))   # after the start: no more reads, no second start
        self.assertNotIn('practice', CAR['go_paths'])
        self.assertFalse(any(p == '/api/pid/stop' for p, _ in CAR['posts']))   # flag back to 0 never stops the car

    def test_race_flag_already_one(self):
        self.race_cfg(practice=True); CAR['go'] = b'1'
        self.assertEqual(self.call('/api/race/arm', {})[0], 200); time.sleep(.4)
        self.assertEqual(self.starts(), []); self.assertTrue(self.call('/api/pid/status')[1]['race']['practice'])
        self.assertIn('practice=1', CAR['go_paths'])
        CAR['go'] = b'0'; self.assertTrue(self.wait(lambda: S.RACE.snapshot()['saw_zero']))
        CAR['go'] = b'1'; self.assertTrue(self.wait(lambda: len(self.starts()) == 1))

    def test_race_offline_flag(self):
        self.race_cfg(); CAR['go'] = None
        self.call('/api/race/arm', {}); self.assertTrue(self.wait(lambda: S.RACE.snapshot()['misses'] >= 3))
        g = S.RACE.snapshot(); self.assertEqual((g['armed'], g['flag']), (True, None)); self.assertEqual(self.starts(), [])
        self.assertEqual(self.call('/api/race/disarm', {})[0], 200); time.sleep(.1); n = CAR['go_reads']; time.sleep(.3)
        self.assertEqual(CAR['go_reads'], n)                                # disarmed: reading stops

    def test_race_arm_refusals(self):
        self.race_cfg()
        CAR['pid']['run'] = dict(id='r', state='running'); self.assertEqual(self.call('/api/race/arm', {})[1]['reason'], 'BUSY'); CAR['pid']['run'] = None
        CAR['pid']['state'] = 'idle'; self.assertEqual(self.call('/api/race/arm', {})[1]['reason'], 'MODEL_NOT_READY')
        CAR['pid']['state'] = 'no_reliable_path'; self.assertEqual(self.call('/api/race/arm', {})[1]['reason'], 'NO_FRESH_PATH')
        CAR['pid']['state'] = 'ready'; CAR['veh']['vehicle']['pins']['D11'] = 0; self.assertEqual(self.call('/api/race/arm', {})[1]['reason'], 'ESTOP')
        CAR['veh']['vehicle']['pins']['D11'] = 1
        self.assertEqual(self.call('/api/race/arm', {})[0], 200); self.assertEqual(self.call('/api/race/arm', {})[1]['reason'], 'RACE_ARMED')

    def test_race_disarm_paths(self):
        self.race_cfg(); calls = []; old = S.restart_unit; S.restart_unit = lambda u: calls.append(u); self.addCleanup(lambda: setattr(S, 'restart_unit', old))
        self.call('/api/race/arm', {})
        self.assertEqual(self.call('/api/repair/imu', {})[1]['reason'], 'RACE_ARMED'); self.assertEqual(self.call('/api/repair/model', {})[1]['reason'], 'RACE_ARMED')
        self.assertEqual(calls, [])
        self.assertEqual(self.call('/api/pid/stop', {})[0], 200); self.assertFalse(S.RACE.armed)
        self.call('/api/race/arm', {}); self.assertEqual(self.call('/api/pid/start', {})[0], 200); self.assertFalse(S.RACE.armed)
        self.assertEqual(len(self.starts()), 1)

    def test_race_fire_retry_and_refusals(self):
        self.race_cfg()
        CAR['queue'] = [(409, dict(ok=False, reason='Need 3 consecutive fresh valid model results'))] * 2
        self.call('/api/race/arm', {}); self.wait(lambda: S.RACE.snapshot()['saw_zero']); CAR['go'] = b'1'
        self.assertTrue(self.wait(lambda: S.RACE.snapshot()['result'] is not None))
        r = S.RACE.snapshot()['result']; self.assertEqual((r['ok'], r['tries']), (True, 3))   # NO_FRESH_PATH retried within 1 s
        S.RACE = S.RaceGo(); CAR['posts'].clear(); CAR['go'] = b'0'
        CAR['queue'] = [(409, dict(ok=False, reason='Another controller is active'))]
        self.call('/api/race/arm', {}); self.wait(lambda: S.RACE.snapshot()['saw_zero']); CAR['go'] = b'1'
        self.assertTrue(self.wait(lambda: S.RACE.snapshot()['result'] is not None))
        r = S.RACE.snapshot()['result']; self.assertEqual((r['ok'], r['code'], r['tries']), (False, 'BUSY', 1))   # other refusals: once
        S.RACE = S.RaceGo(); CAR['posts'].clear(); CAR['go'] = b'0'
        self.call('/api/race/arm', {}); self.wait(lambda: S.RACE.snapshot()['saw_zero'])
        CAR['veh']['vehicle']['pins']['D11'] = 0; CAR['go'] = b'1'                       # e-stop pressed after arming
        self.assertTrue(self.wait(lambda: S.RACE.snapshot()['result'] is not None))
        self.assertEqual((S.RACE.snapshot()['result']['code'], self.starts()), ('ESTOP', []))


if __name__ == '__main__':
    unittest.main()
