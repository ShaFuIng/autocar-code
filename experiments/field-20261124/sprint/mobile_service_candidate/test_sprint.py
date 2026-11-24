"""mobile-service-r7 (field-20261124 sprint): straight-line sprint mode (sprint_cfg.py + the server routes). Laptop only, no car: the car's five
files of snapshot 20261123d are copied to a temporary folder. python -m unittest -v test_sprint"""
import hashlib, importlib.util, json, os, tempfile, threading, unittest, urllib.request, urllib.error
from http.server import ThreadingHTTPServer
from pathlib import Path
import start_cfg as C
import sprint_cfg as SP
import server as S

EXP = next((c for q in Path(__file__).resolve().parents for c in (q / 'laptop_workbench/data/experiments', q) if (c / 'orin-web-snapshot-20261123d').is_dir()), None)
CAR = EXP / 'orin-web-snapshot-20261123d' if EXP else None          # = the car's web/ on 2026-11-23 18:1x (flag spot, back 0)
R6A = EXP.parents[2] / 'mobile_service' / 'start_cfg.py' if EXP else None   # the phone service on the car (r6a), for the "sprint off = unchanged" proof
HAVE = bool(CAR and CAR.is_dir())
CAR_SHA = {'speed_adapt.json': '398336aff3f04c674eb7d3d16926579a6ad3dab2675bb26e2bcc1025868184f8'}


def raw(d, names=SP.ALL): return {n: (Path(d) / n).read_bytes() for n in names}


def load(d, names=SP.ALL): return {n: json.loads((Path(d) / n).read_text(encoding='utf8')) for n in names}


@unittest.skipUnless(HAVE, 'snapshot 20261123d not found')
class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); t = Path(self.tmp.name); self.cfg = t / 'web'; self.cfg.mkdir()
        for n in SP.ALL: (self.cfg / n).write_bytes((CAR / n).read_bytes())
        self.ref = t / 'start_ref.json'; self.bk = t / 'backups'; self.log = []; self.p = SP.params()
        C.rebuild_ref(self.cfg, self.ref, self.bk)

    def tearDown(self): C._replace = os.replace; self.tmp.cleanup()

    def start(self, back): return C.apply(self.cfg, self.ref, self.bk, back)

    def on(self, finish=None, p=None): return SP.apply(self.cfg, self.ref, self.bk, True, finish, p or self.p, log=self.log.append)

    def off(self): return SP.apply(self.cfg, self.ref, self.bk, False, None, self.p, log=self.log.append)

    def st(self): return SP.state(self.cfg, self.ref, self.p)


class Plan(unittest.TestCase):
    def test_numbers(self):
        p = SP.params()
        self.assertEqual((p['brake_before_c1s_m'], p['up_rate'], p['max_pwm'], p['after_cap_pwm']), (3.44, 60.0, 100.0, 50.0))
        z, info = SP.plan(7.0, p)
        self.assertEqual((info['D'], info['brake_from_m'], len(z)), (10.19, 6.75, 1))
        self.assertEqual((z[0]['anchor'], z[0]['from_m'], z[0]['to_m'], z[0]['cap_pwm'], z[0]['sprint']), ('start', 6.75, 100000.0, 50.0, True))
        z, info = SP.plan(7.0, p, 8.0)        # finish after the brake point: after-cap then PWM 0
        self.assertEqual([(q['from_m'], q['cap_pwm']) for q in z], [(6.75, 50.0), (8.0, 0.0)])
        z, info = SP.plan(7.0, p, 5.0)        # finish before the brake point: full throttle until the finish, then 0
        self.assertEqual([(q['from_m'], q['cap_pwm']) for q in z], [(5.0, 0.0)])
        self.assertEqual(SP.plan(2.6, p)[1]['brake_from_m'], round(C.d_of(2.6) - 3.44, 3))

    def test_finish_input(self):
        p = SP.params(); D = C.d_of(7.0)
        for v, want in ((None, None), ('', None), (8.0, 8.0), (1, 1.0), (8.8, 8.8)):
            self.assertEqual(SP.parse_finish(v, D, p), (want, None), v)
        for v, code in (('8', 'SPRINT_BAD_INPUT'), (True, 'SPRINT_BAD_INPUT'), (float('nan'), 'SPRINT_BAD_INPUT'), (8.25, 'SPRINT_STEP'),
                        (0.5, 'SPRINT_FINISH_RANGE'), (9.0, 'SPRINT_FINISH_RANGE')):   # 10.19 - 1.3 = 8.89 -> max 8.9; 9.0 is out
            self.assertEqual(SP.parse_finish(v, D, p), (None, code), v)

    def test_params_from_service_json(self):
        self.assertEqual(SP.params({'up_rate': 100, 'brake_before_c1s_m': 4.0})['up_rate'], 100.0)
        self.assertEqual(SP.params({'up_rate': 500, 'after_cap_pwm': 80, 'max_pwm': True})['up_rate'], 60.0)   # out of range -> default
        self.assertEqual(SP.params({'after_cap_pwm': 80})['after_cap_pwm'], 50.0)


class OffIsUnchanged(Base):
    def test_car_files_report_off(self):
        s = self.st()
        self.assertEqual((s['on'], s['consistent'], s['reason'], s['back_m'], s['can_on']), (False, True, None, 0.0, False))
        self.assertEqual(hashlib.sha256((CAR / 'speed_adapt.json').read_bytes()).hexdigest(), CAR_SHA['speed_adapt.json'])

    @unittest.skipUnless(R6A and R6A.is_file(), 'r6a start_cfg not found')
    def test_start_cfg_same_bytes_as_r6a_without_sprint(self):
        """with no sprint zone the candidate start_cfg writes exactly what the car's r6a start_cfg writes, for every start position."""
        spec = importlib.util.spec_from_file_location('start_cfg_r6a', R6A); old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)
        files_new = C.read_files(self.cfg); files_old = old.read_files(self.cfg)
        self.assertEqual(C.ref_from_files(files_new, 0.0), old.ref_from_files(files_old, 0.0))
        ref = json.loads(self.ref.read_text(encoding='utf8'))
        for i in range(71):
            b = i / 10.
            a = C.build(files_new, ref, b); o = old.build(files_old, ref, b)
            self.assertEqual({n: C.dumps(a[n], files_new[n]['style']) for n in C.FILES}, {n: old.dumps(o[n], files_old[n]['style']) for n in C.FILES}, b)
            self.assertEqual(C.judge({n: dict(obj=a[n]) for n in C.FILES}, ref), old.judge({n: dict(obj=o[n]) for n in C.FILES}, ref))


class RoundTrip(Base):
    def test_on_off_byte_identical_at_0_25_7(self):
        for back, can in ((0.0, False), (2.5, False), (2.6, True), (7.0, True)):
            self.start(back); before = raw(self.cfg)
            s = self.st(); self.assertEqual((s['back_m'], s['can_on']), (back, can))
            if not can:
                with self.assertRaises(C.Refused) as e: self.on()
                self.assertEqual(e.exception.code, 'SPRINT_BACK_TOO_NEAR'); self.assertEqual(raw(self.cfg), before); continue
            for fin in (None, 8.0 if back == 7.0 else 3.0):
                r = self.on(fin); self.assertEqual(r['changed'], ['brake_zones.json', 'speed_adapt.json'])
                s = self.st(); self.assertEqual((s['on'], s['consistent'], s['back_m'], s['finish_m']), (True, True, back, fin))
                self.assertTrue(C.state(self.cfg, self.ref)['consistent'])                 # the start position is still read correctly
                self.assertEqual(C.state(self.cfg, self.ref)['back_m'], back)
                r = self.off(); self.assertEqual(r['changed'], ['speed_adapt.json', 'brake_zones.json'])
                self.assertEqual(raw(self.cfg), before, (back, fin))
                self.assertEqual(self.st()['on'], False)
        self.assertFalse(list(self.cfg.glob('*.tmp')))
        self.start(0.0); self.assertEqual(raw(self.cfg), raw(CAR))                     # and back to the car's files exactly

    def test_only_sprint_keys_change(self):
        self.start(7.0); old = load(self.cfg); self.on(); new = load(self.cfg)
        for n in ('lap_pos.json', 'lap_ff.json', 'start_bias.json'): self.assertEqual(old[n], new[n])
        sa0, sa1 = old['speed_adapt.json'], new['speed_adapt.json']
        self.assertEqual({k for k in set(sa0) | set(sa1) if sa0.get(k) != sa1.get(k)}, {'hard_cap', 'max_pwm', 'up_rate', 'sprint_mode'})
        self.assertEqual((sa1['hard_cap'], sa1['max_pwm'], sa1['up_rate']), (100.0, 100.0, 60.0))
        self.assertEqual(sa1['sprint_mode']['orig'], {'hard_cap': 50, 'max_pwm': 50.0, 'up_rate': 30.0})
        bz0, bz1 = old['brake_zones.json'], new['brake_zones.json']
        self.assertEqual(bz1['zones'][:len(bz0['zones'])], bz0['zones']); self.assertEqual({k: v for k, v in bz1.items() if k != 'zones'}, {k: v for k, v in bz0.items() if k != 'zones'})
        self.assertEqual(len(bz1['zones']) - len(bz0['zones']), 1)
        self.assertIs(new['start_bias.json']['enabled'], False)                          # back 7 > 2.5: the start left bias is off

    def test_refusals_in_module(self):
        self.start(7.0); self.on()
        with self.assertRaises(C.Refused) as e: self.on()
        self.assertEqual(e.exception.code, 'SPRINT_ALREADY_ON')
        self.off(); r = self.off(); self.assertEqual((r['changed'], r['backup']), ([], None))   # off when off writes nothing
        with self.assertRaises(C.Refused) as e: self.on(9.5)
        self.assertEqual(e.exception.code, 'SPRINT_FINISH_RANGE')

    def test_start_inconsistent_blocks_on(self):
        self.start(7.0)
        p = self.cfg / 'brake_zones.json'; o = json.loads(p.read_text(encoding='utf8')); o['zones'][0]['to_m'] = 99.0; p.write_text(json.dumps(o, indent=1), encoding='utf8')
        with self.assertRaises(C.Refused) as e: self.on()
        self.assertEqual(e.exception.code, 'SPRINT_START_BAD')


class Interrupted(Base):
    def test_write_order_and_failure_restores(self):
        self.start(7.0); before = raw(self.cfg); order = []
        def rec(a, b): order.append(Path(b).name); os.replace(a, b)
        C._replace = rec; self.on(); self.assertEqual(order, ['brake_zones.json', 'speed_adapt.json'])
        on_bytes = raw(self.cfg); order.clear(); self.off(); self.assertEqual(order, ['speed_adapt.json', 'brake_zones.json'])
        for k in (1, 2):                                                                    # ON fails at the first / second replace
            n = [0]
            def bad(a, b):
                n[0] += 1
                if n[0] == k: raise OSError('disk full (test)')
                os.replace(a, b)
            C._replace = bad
            with self.assertRaises(C.Refused) as e: self.on()
            self.assertEqual(e.exception.code, 'SPRINT_WRITE_FAILED'); self.assertEqual(raw(self.cfg), before, k)
            self.assertFalse(list(self.cfg.glob('*.tmp')))
        C._replace = os.replace; self.on(); self.assertEqual(raw(self.cfg), on_bytes)
        n = [0]
        def bad2(a, b):
            n[0] += 1
            if n[0] == 2: raise OSError('power (test)')
            os.replace(a, b)
        C._replace = bad2
        with self.assertRaises(C.Refused): self.off()
        self.assertEqual(raw(self.cfg), on_bytes)                                           # OFF failing half-way puts the sprint files back whole

    def test_power_loss_states_are_partial_and_off_cleans(self):
        self.start(7.0); before = raw(self.cfg); self.on(); on_bytes = raw(self.cfg)
        # power loss after the first ON replace: brake_zones has the sprint zone, speed_adapt still normal (harmless state)
        (self.cfg / 'speed_adapt.json').write_bytes(before['speed_adapt.json'])
        s = self.st(); self.assertEqual((s['on'], s['partial'], s['consistent'], s['reason']), (True, True, False, 'SPRINT_PARTIAL'))
        self.off(); self.assertEqual(raw(self.cfg), before)
        # the other half (cannot happen with the write order, e.g. a hand edit): speed_adapt 100 without the after-cap
        (self.cfg / 'speed_adapt.json').write_bytes(on_bytes['speed_adapt.json'])
        s = self.st(); self.assertEqual((s['partial'], s['reason']), (True, 'SPRINT_PARTIAL'))
        with self.assertRaises(C.Refused) as e: self.on()
        self.assertEqual(e.exception.code, 'SPRINT_PARTIAL')
        self.off(); self.assertEqual(raw(self.cfg), before)

    def test_start_changed_by_hand_is_inconsistent(self):
        self.start(7.0); self.on()
        p = self.cfg / 'lap_pos.json'; o = json.loads(p.read_text(encoding='utf8')); o['start_s'] = -C.d_of(6.0); p.write_text(json.dumps(o, indent=1) + '\n', encoding='utf8')
        s = self.st(); self.assertEqual((s['on'], s['consistent'], s['reason']), (True, False, 'SPRINT_INCONSISTENT'))


@unittest.skipUnless(HAVE, 'snapshot 20261123d not found')
class Http(unittest.TestCase):
    """the real Handler with the car control replaced by a function (no network to a car)."""
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(); t = Path(cls.tmp.name)
        cls.keep = (S.START_CFG_DIR, S.START_REF, S.LEVEL_BACKUPS, S.SERVICE, S.ACCESS, S.upstream, S.TOKEN)
        S.START_CFG_DIR = t / 'web'; S.START_CFG_DIR.mkdir(); S.START_REF = t / 'start_ref.json'; S.LEVEL_BACKUPS = t / 'backups'
        S.SERVICE = t / 'service.json'; S.SERVICE.write_text('{"login": false}', encoding='utf8'); S.ACCESS = t / 'access.log'; S.TOKEN = t / 'token'; S.TOKEN.write_text('x')
        cls.pid = dict(state='ready', run=None); cls.sent = []
        def up(path, body=None, timeout=4.):
            if body is not None: cls.sent.append(path)
            return 200, cls.pid
        S.upstream = up
        cls.svc = ThreadingHTTPServer(('127.0.0.1', 0), S.Handler); cls.base = 'http://127.0.0.1:%d' % cls.svc.server_address[1]
        threading.Thread(target=cls.svc.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.svc.shutdown(); (S.START_CFG_DIR, S.START_REF, S.LEVEL_BACKUPS, S.SERVICE, S.ACCESS, S.upstream, S.TOKEN) = cls.keep; cls.tmp.cleanup()

    def setUp(self):
        for n in SP.ALL: (S.START_CFG_DIR / n).write_bytes((CAR / n).read_bytes())
        if S.START_REF.exists(): S.START_REF.unlink()
        self.pid['run'] = None; self.sent.clear(); S.RACE = S.RaceGo(); S._start_cache[0] = None; S._sprint_cache[0] = None

    def call(self, path, body=None):
        h = {} if body is None else {'Content-Type': 'application/json', 'X-Orin-Control': '1'}
        req = urllib.request.Request(self.base + path, data=None if body is None else json.dumps(body).encode(), headers=h)
        try:
            with urllib.request.urlopen(req, timeout=5) as r: return r.status, json.loads(r.read()) if 'json' in r.headers.get('Content-Type', '') else r.read()
        except urllib.error.HTTPError as e: return e.code, json.loads(e.read())

    def test_flow(self):
        self.assertEqual(self.call('/sprint.js')[0], 200)
        code, d = self.call('/api/sprint'); self.assertEqual((code, d['sprint']['on'], d['sprint']['can_on']), (200, False, False))
        self.call('/api/start/rebuild-ref', {})
        self.assertEqual(self.call('/api/sprint/set', dict(on=True))[1]['reason'], 'SPRINT_BACK_TOO_NEAR')
        self.assertEqual(self.call('/api/start/set', dict(back_m=7.0))[0], 200)
        normal = raw(S.START_CFG_DIR)
        code, d = self.call('/api/sprint/set', dict(on=True, finish_m=None))
        self.assertEqual((code, d['on'], d['back_m'], d['brake_from_m'], d['changed']), (200, True, 7.0, 6.75, ['brake_zones.json', 'speed_adapt.json']))
        code, d = self.call('/api/pid/status'); self.assertEqual((d['service'], d['sprint']['on'], d['sprint']['consistent'], d['start']['back_m'], d['start']['consistent']), ('mobile-service-r7', True, True, 7.0, True))
        # while on: start position, rebuild and race mode are refused; a start is allowed (it is the sprint run)
        self.assertEqual(self.call('/api/start/set', dict(back_m=6.0))[1]['reason'], 'SPRINT_ON')
        self.assertEqual(self.call('/api/start/rebuild-ref', {})[1]['reason'], 'SPRINT_ON')
        self.assertEqual(self.call('/api/race/arm', {})[1]['reason'], 'SPRINT_ON')
        self.assertEqual(self.call('/api/pid/start', {})[0], 200); self.assertIn('/api/pid/start', self.sent)
        for st in ('running', 'paused'):
            self.pid['run'] = dict(id='r', state=st); self.assertEqual(self.call('/api/sprint/set', dict(on=False))[1]['reason'], 'RUN_ACTIVE')
        self.pid['run'] = None
        self.assertEqual(self.call('/api/sprint/set', dict(on='yes'))[1]['reason'], 'SPRINT_BAD_INPUT')
        code, d = self.call('/api/sprint/set', dict(on=False)); self.assertEqual((code, d['on']), (200, False))
        self.assertEqual(raw(S.START_CFG_DIR), normal)
        self.assertIn('sprint-cfg ON', S.ACCESS.read_text(encoding='utf8'))

    def test_partial_blocks_start_and_race_blocks_sprint(self):
        self.call('/api/start/rebuild-ref', {}); self.call('/api/start/set', dict(back_m=7.0)); normal = raw(S.START_CFG_DIR)
        self.call('/api/sprint/set', dict(on=True)); on = raw(S.START_CFG_DIR)
        (S.START_CFG_DIR / 'brake_zones.json').write_bytes(normal['brake_zones.json'])     # speed_adapt 100 without the after-cap
        code, d = self.call('/api/pid/start', {}); self.assertEqual((code, d['reason']), (409, 'SPRINT_PARTIAL')); self.assertNotIn('/api/pid/start', self.sent)
        self.assertEqual(self.call('/api/sprint/set', dict(on=False))[0], 200); self.assertEqual(raw(S.START_CFG_DIR), normal)
        self.assertEqual(self.call('/api/pid/start', {})[0], 200)                          # sprint off: the start is passed through as before
        S.RACE.armed = True
        self.assertEqual(self.call('/api/sprint/set', dict(on=True))[1]['reason'], 'RACE_ARMED')
        S.RACE.armed = False
        def down(path, body=None, timeout=4.): raise RuntimeError('dashboard unreachable')
        keep = S.upstream; S.upstream = down
        try: self.assertEqual(self.call('/api/sprint/set', dict(on=True))[1]['reason'], 'CAR_UNREACHABLE')
        finally: S.upstream = keep
        self.assertEqual(raw(S.START_CFG_DIR), normal)


if __name__ == '__main__':
    unittest.main()
