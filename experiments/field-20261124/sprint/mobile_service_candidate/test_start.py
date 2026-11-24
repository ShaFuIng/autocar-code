"""mobile-service-r6 (2026-11-23): start position (start_cfg.py + the server routes). Laptop only, no car: the car's four files are copied
to a temporary folder. python -m unittest -v test_start"""
import copy, json, os, subprocess, sys, tempfile, threading, unittest, urllib.request, urllib.error
from http.server import ThreadingHTTPServer
from pathlib import Path
import start_cfg as C
import server as S

EXP = next((c for q in Path(__file__).resolve().parents for c in (q / 'laptop_workbench/data/experiments', q) if (c / 'field-20261123/startcfg').is_dir()),
           Path(__file__).resolve().parents[1] / 'laptop_workbench/data/experiments')   # r7 candidate: also found from field-20261124/sprint/mobile_service_candidate
FLAG = EXP / 'field-20261123/startcfg/base_flag'              # flag-spot files (lap_pos without skip_after_by.C1s)
RULE = EXP / 'field-20261123/startcfg/mkstart_rule.py'        # the reference implementation used on the laptop
CAR = EXP / 'orin-web-snapshot-20261123b'                     # the car's files of 2026-11-23 (back 0, C1s 9.0, CRLF lap_pos, one-line start_bias)
HAVE = FLAG.is_dir() and RULE.is_file() and CAR.is_dir()


def load(d): return {n: json.loads((Path(d) / n).read_text(encoding='utf8')) for n in C.FILES}


def raw(d): return {n: (Path(d) / n).read_bytes() for n in C.FILES}


@unittest.skipUnless(HAVE, 'laptop experiment folders not found')
class Base(unittest.TestCase):
    src = CAR

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); t = Path(self.tmp.name); self.cfg = t / 'web'; self.cfg.mkdir()
        for n in C.FILES: (self.cfg / n).write_bytes((Path(self.src) / n).read_bytes())
        self.ref = t / 'start_ref.json'; self.bk = t / 'backups'; self.log = []

    def tearDown(self): C._replace = os.replace; self.tmp.cleanup()

    def edit(self, name, fn):
        p = self.cfg / name; o = json.loads(p.read_text(encoding='utf8')); fn(o); p.write_text(json.dumps(o, ensure_ascii=False, indent=1) + '\n', encoding='utf8')

    def st(self): return C.state(self.cfg, self.ref)

    def set(self, back): return C.apply(self.cfg, self.ref, self.bk, back, log=self.log.append)


class Rule(unittest.TestCase):
    def test_d_and_grid(self):
        self.assertEqual([C.d_of(b) for b in (0., .1, .2, 2.5, 2.6, 6., 7.)], [3.44, 3.29, 3.39, 5.69, 5.79, 9.19, 10.19])
        for i in range(71):   # every grid value maps to exactly one start_s and back
            self.assertEqual(C.back_of_start_s(-C.d_of(i / 10.)), i / 10.)
        self.assertIsNone(C.back_of_start_s(-4.0)); self.assertIsNone(C.back_of_start_s(None)); self.assertIsNone(C.back_of_start_s(True))

    def test_input_validation(self):
        for v, want in ((0, 0.), (0.0, 0.), (2.5, 2.5), (7, 7.), (6.000000001, 6.), (-0.0, 0.)):
            self.assertEqual(C.parse_back(v), (want, None), v)
        for v, code in (('2.5', 'START_BAD_INPUT'), (None, 'START_BAD_INPUT'), (True, 'START_BAD_INPUT'), (float('nan'), 'START_BAD_INPUT'), (float('inf'), 'START_BAD_INPUT'),
                        ([1], 'START_BAD_INPUT'), (-0.1, 'START_RANGE'), (7.1, 'START_RANGE'), (0.25, 'START_STEP'), (2.55, 'START_STEP')):
            self.assertEqual(C.parse_back(v), (None, code), v)


class MatchesRule(Base):
    """same JSON as mkstart_rule.py for back 0 / 2.5 / 2.6 / 6.0, starting from the flag files."""
    src = FLAG

    def rule(self, back):
        out = Path(self.tmp.name) / ('rule%.1f' % back)
        r = subprocess.run([sys.executable, str(RULE), '--back', str(back), '--base', str(FLAG), '--out', str(out)], capture_output=True, text=True, encoding='utf8', errors='replace',
                           env=dict(os.environ, PYTHONIOENCODING='utf8'))
        self.assertEqual(r.returncode, 0, r.stderr); return load(out)

    def test_same_json(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk)
        self.assertEqual(self.st()['reason'], 'START_NEEDS_APPLY')          # the flag files lack skip_after_by.C1s: set adds it
        for back in (6.0, 0.0, 2.6, 2.5):
            self.set(back); mine, want = load(self.cfg), self.rule(back)
            want['start_bias.json'].pop('note_far_20261123', None)             # mkstart_rule adds a note key; the phone service changes only 'enabled'
            self.assertEqual(mine, want, back)
            s = self.st(); self.assertEqual((s['back_m'], s['consistent'], s['left_bias_on']), (back, True, back <= 2.5))
        for d in ('back0.0', 'back2.5', 'back2.6', 'back6.0'):                 # the files generated on 11-23
            self.set(float(d[4:])); want = load(FLAG.parent / d); want['start_bias.json'].pop('note_far_20261123', None)
            self.assertEqual(load(self.cfg), want, d)

    def test_near_points_differ_only_in_hold_s(self):
        """back 0.1 / 0.2 give D < 3.44: mkstart (under mkstart_rule) also shortens start_bias.hold_s there; the user's rule (task item 5) does not."""
        C.rebuild_ref(self.cfg, self.ref, self.bk)
        for back in (0.1, 0.2):
            self.set(back); mine, want = load(self.cfg), self.rule(back)
            self.assertNotEqual(mine['start_bias.json']['hold_s'], want['start_bias.json']['hold_s'])
            want['start_bias.json']['hold_s'] = mine['start_bias.json']['hold_s']; self.assertEqual(mine, want, back)


class RoundTrip(Base):
    def test_back_and_forth_is_byte_identical(self):
        before = raw(self.cfg); C.rebuild_ref(self.cfg, self.ref, self.bk)
        self.assertTrue(self.st()['consistent'])
        for back in (6.0, 2.5, 0.0): self.set(back)
        self.assertEqual(raw(self.cfg), before)
        self.assertEqual(len(list(self.bk.glob('startcfg_*'))), 3)
        b = sorted(self.bk.glob('startcfg_*'))[0]
        self.assertEqual({n: (b / n).read_bytes() for n in C.FILES}, before)   # the first backup holds the originals
        self.assertTrue(raw(self.cfg)['lap_pos.json'].count(b'\r\n') > 100)    # CRLF kept
        self.assertFalse(list(self.cfg.glob('*.tmp')))

    def test_same_value_writes_nothing(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk); before = raw(self.cfg)
        r = self.set(0)
        self.assertEqual((r['backup'], r['changed']), (None, [])); self.assertEqual(raw(self.cfg), before); self.assertFalse(self.bk.exists())

    def test_only_rule_keys_change(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk); old = load(self.cfg); r = self.set(6.0); new = load(self.cfg)
        self.assertEqual(len(r['changed']), 13)
        for n in C.FILES:
            for line in C.changes(old[n], new[n]): self.assertTrue(C.allowed(n, line.split(':', 1)[0], {'brake_zones.json': dict(obj=old['brake_zones.json'])}), line)
        z = new['brake_zones.json']['zones']
        self.assertEqual([(z[i]['from_m'], z[i]['to_m']) for i in (0, 2, 5, 7, 9)], [(7.55, 9.25), (22.65, 24.25), (29.25, 35.25), (54.25, 56.95), (50.35, 52.75)])
        self.assertEqual([z[i]['from_m'] for i in (10, 11, 12, 13)], [999.0] * 4)   # dummy start pieces of lap zones stay
        self.assertEqual((new['lap_pos.json']['start_s'], new['lap_ff.json']['curves']['c1']['start']['gap_m'], new['start_bias.json']['enabled']), (-9.19, 8.95, False))


class OtherEdits(Base):
    def test_other_tuning_survives(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk)
        def bz(o): o['zones'][1]['cap_pwm'] = 41; o['zones'][0]['cap_pwm'] = 33; o['zones'].append(dict(name='new lap zone', lap_anchor='C1e', lap_from_m=1., lap_to_m=2., anchor='start', from_m=999., to_m=999.5, cap_pwm=40))
        def lf(o): o['curves']['c2']['band'] = dict(test='changed'); o['curves']['c1']['start']['win_m'] = [-1.0, 0.9]
        def lp(o): o['skip_after_by']['C2e'] = 4.5
        self.edit('brake_zones.json', bz); self.edit('lap_ff.json', lf); self.edit('lap_pos.json', lp)
        self.assertTrue(self.st()['consistent'])
        for back in (6.0, 0.0, 3.3):
            self.set(back); o = load(self.cfg)
            self.assertEqual((o['brake_zones.json']['zones'][1]['cap_pwm'], o['brake_zones.json']['zones'][0]['cap_pwm'], o['brake_zones.json']['zones'][-1]['from_m']), (41, 33, 999.))
            self.assertEqual((o['lap_ff.json']['curves']['c2']['band'], o['lap_ff.json']['curves']['c1']['start']['win_m']), (dict(test='changed'), [-1.0, 0.9]))
            self.assertEqual(o['lap_pos.json']['skip_after_by'], {'C2e': 4.5, 'C1s': 9.0})


class Inconsistent(Base):
    def unchanged(self, before):
        self.assertEqual(raw(self.cfg), before)

    def test_no_ref(self):
        before = raw(self.cfg); self.assertEqual(self.st()['reason'], 'START_NO_REF')
        with self.assertRaises(C.Refused) as e: self.set(2.5)
        self.assertEqual(e.exception.code, 'START_NO_REF'); self.unchanged(before)
        self.ref.write_text('{"zones": 3}', encoding='utf8'); self.assertEqual(self.st()['reason'], 'START_NO_REF')

    def test_tuned_lap1_zone_is_refused_then_kept_after_rebuild(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk); self.set(2.5)
        self.edit('brake_zones.json', lambda o: o['zones'][0].update(to_m=6.0))   # someone lengthened the lap-1 curve-1 zone at back 2.5 (was 5.75)
        s = self.st(); self.assertEqual((s['reason'], s['back_m']), ('START_INCONSISTENT', 2.5))
        self.assertTrue(any('to_m 現在 6.0' in p and '5.75' in p for p in s['problems']), s['problems'])
        before = raw(self.cfg)
        with self.assertRaises(C.Refused) as e: self.set(0)
        self.assertEqual(e.exception.code, 'START_INCONSISTENT'); self.unchanged(before)
        r = C.rebuild_ref(self.cfg, self.ref, self.bk)                         # explicit: the present files are right
        self.assertEqual(r['ref']['zones'][0]['to_m'], 3.75); self.assertEqual(r['problems'], [])
        self.assertEqual(len(list(self.bk.glob('startref_*'))), 1)            # the old reference was kept
        self.set(0); self.assertEqual(load(self.cfg)['brake_zones.json']['zones'][0]['to_m'], 3.75)   # the lengthening survives the move

    def test_start_s_alone_or_unknown(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk)
        self.edit('lap_pos.json', lambda o: o.update(start_s=-9.19))          # start_s says 6 m, the zones still say 0 m
        s = self.st(); self.assertEqual((s['reason'], s['back_m']), ('START_INCONSISTENT', 6.0))
        self.assertTrue(len(s['problems']) >= 11)
        self.edit('lap_pos.json', lambda o: o.update(start_s=-4.0)); before = raw(self.cfg)
        self.assertEqual(self.st()['reason'], 'START_S_UNKNOWN')
        for f in (lambda: self.set(1.0), lambda: C.rebuild_ref(self.cfg, self.ref, self.bk)):
            with self.assertRaises(C.Refused) as e: f()
            self.assertEqual(e.exception.code, 'START_S_UNKNOWN')
        self.unchanged(before)

    def test_new_start_zone_and_missing_gap(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk)
        self.edit('brake_zones.json', lambda o: o['zones'].append(dict(name='extra start zone', anchor='start', from_m=5.0, to_m=6.0, cap_pwm=40)))
        s = self.st(); self.assertEqual(s['reason'], 'START_INCONSISTENT'); self.assertTrue(any('多了' in p for p in s['problems']))
        self.edit('brake_zones.json', lambda o: o['zones'].pop())
        self.edit('lap_ff.json', lambda o: o['curves']['c1'].pop('start'))
        self.assertEqual(self.st()['reason'], 'START_INCONSISTENT')
        self.edit('lap_ff.json', lambda o: o.update(curves=[1]))
        self.assertEqual(self.st()['reason'], 'START_FILE_ERROR')
        with self.assertRaises(C.Refused) as e: self.set(1.0)
        self.assertEqual(e.exception.code, 'START_FILE_ERROR')

    def test_absolute_keys_are_repaired_by_set(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk)
        self.edit('start_bias.json', lambda o: o.update(enabled=False))
        s = self.st(); self.assertEqual((s['reason'], s['consistent']), ('START_NEEDS_APPLY', False))
        self.set(0); self.assertTrue(self.st()['consistent']); self.assertIs(load(self.cfg)['start_bias.json']['enabled'], True)

    def test_missing_or_broken_file(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk); (self.cfg / 'start_bias.json').unlink()
        self.assertEqual(self.st()['reason'], 'START_FILE_ERROR')
        (self.cfg / 'start_bias.json').write_text('{broken', encoding='utf8')
        with self.assertRaises(C.Refused) as e: self.set(1.0)
        self.assertEqual(e.exception.code, 'START_FILE_ERROR')


class Force(Base):   # r6a (2026-11-23): a write cut by a power loss is repaired from the reference
    def test_half_written_set_is_repaired_by_force(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk); start = raw(self.cfg)
        self.set(6.0); six = raw(self.cfg)
        for n in ('lap_ff.json', 'start_bias.json'): (self.cfg / n).write_bytes(start[n])       # power loss after two of the four replaces
        self.assertEqual(self.st()['reason'], 'START_INCONSISTENT')
        for back in (6.0, 2.5):
            with self.assertRaises(C.Refused) as e: self.set(back)                              # typing the value again is still refused
            self.assertEqual(e.exception.code, 'START_INCONSISTENT')
        mixed = raw(self.cfg)
        r = C.apply(self.cfg, self.ref, self.bk, 6.0, log=self.log.append, force=True)
        self.assertEqual(raw(self.cfg), six); self.assertTrue(self.st()['consistent']); self.assertEqual(self.st()['back_m'], 6.0)
        self.assertEqual({n: (self.bk / r['backup'] / n).read_bytes() for n in C.FILES}, mixed)  # the mixed files were backed up first
        self.assertIn('FORCED', self.log[-1])
        for n in ('lap_ff.json', 'start_bias.json'): (self.cfg / n).write_bytes(start[n])
        C.apply(self.cfg, self.ref, self.bk, 0.0, force=True); self.assertEqual(raw(self.cfg), start)   # or straight back to the flag spot

    def test_force_needs_a_reference_and_valid_input(self):
        with self.assertRaises(C.Refused) as e: C.apply(self.cfg, self.ref, self.bk, 2.5, force=True)
        self.assertEqual(e.exception.code, 'START_NO_REF')
        C.rebuild_ref(self.cfg, self.ref, self.bk); before = raw(self.cfg)
        for bad in ('x', 7.1, 2.55):
            with self.assertRaises(C.Refused) as e: C.apply(self.cfg, self.ref, self.bk, bad, force=True)
            self.assertEqual(e.exception.code, C.parse_back(bad)[1])                            # the same input refusal as without force
        self.assertEqual(raw(self.cfg), before)

    def test_force_keeps_other_tuning_and_is_the_plain_set_when_consistent(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk)
        self.edit('brake_zones.json', lambda o: o['zones'][3].update(cap_pwm=41))
        self.edit('lap_ff.json', lambda o: o['curves']['c1']['start'].update(gap_m=4.0))       # broken by hand
        C.apply(self.cfg, self.ref, self.bk, 2.5, force=True)
        self.assertEqual(load(self.cfg)['brake_zones.json']['zones'][3]['cap_pwm'], 41)
        self.assertEqual(load(self.cfg)['lap_ff.json']['curves']['c1']['start']['gap_m'], round(3.2 + C.delta_of(2.5), 3))
        a = raw(self.cfg); C.apply(self.cfg, self.ref, self.bk, 2.5, force=True); self.assertEqual(raw(self.cfg), a)


class WriteFailure(Base):
    def test_failing_replace_restores(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk); before = raw(self.cfg); n = [0]
        def bad(a, b):
            n[0] += 1
            if n[0] == 3: raise OSError('disk full (test)')
            os.replace(a, b)
        C._replace = bad
        with self.assertRaises(C.Refused) as e: self.set(6.0)
        self.assertEqual(e.exception.code, 'START_WRITE_FAILED'); self.assertEqual(raw(self.cfg), before)
        self.assertFalse(list(self.cfg.glob('*.tmp'))); self.assertTrue(any('FAILED' in l for l in self.log))
        self.assertTrue(list(self.bk.glob('startcfg_*')))                     # the backup was made before the first write

    def test_bad_read_back_restores(self):
        C.rebuild_ref(self.cfg, self.ref, self.bk); before = raw(self.cfg)
        def corrupt(a, b):
            os.replace(a, b)
            if Path(b).name == 'start_bias.json': Path(b).write_bytes(Path(b).read_bytes() + b' ')
        C._replace = corrupt
        with self.assertRaises(C.Refused) as e: self.set(6.0)
        self.assertEqual(e.exception.code, 'START_WRITE_FAILED'); self.assertEqual(raw(self.cfg), before)


@unittest.skipUnless(HAVE, 'laptop experiment folders not found')
class Http(unittest.TestCase):
    """the real Handler with the car control replaced by a function (no network to a car)."""
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(); t = Path(cls.tmp.name)
        cls.keep = (S.START_CFG_DIR, S.START_REF, S.LEVEL_BACKUPS, S.SERVICE, S.ACCESS, S.upstream, S.TOKEN)
        S.START_CFG_DIR = t / 'web'; S.START_CFG_DIR.mkdir(); S.START_REF = t / 'start_ref.json'; S.LEVEL_BACKUPS = t / 'backups'
        S.SERVICE = t / 'service.json'; S.SERVICE.write_text('{"login": false}', encoding='utf8'); S.ACCESS = t / 'access.log'; S.TOKEN = t / 'token'; S.TOKEN.write_text('x')
        cls.pid = dict(state='ready', run=None); S.upstream = lambda path, body=None, timeout=4.: (200, cls.pid)
        cls.svc = ThreadingHTTPServer(('127.0.0.1', 0), S.Handler); cls.base = 'http://127.0.0.1:%d' % cls.svc.server_address[1]
        threading.Thread(target=cls.svc.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.svc.shutdown(); (S.START_CFG_DIR, S.START_REF, S.LEVEL_BACKUPS, S.SERVICE, S.ACCESS, S.upstream, S.TOKEN) = cls.keep; cls.tmp.cleanup()

    def setUp(self):
        for n in C.FILES: (S.START_CFG_DIR / n).write_bytes((CAR / n).read_bytes())
        if S.START_REF.exists(): S.START_REF.unlink()
        self.pid['run'] = None; S.RACE = S.RaceGo(); S._start_cache[0] = None

    def call(self, path, body=None):
        h = {} if body is None else {'Content-Type': 'application/json', 'X-Orin-Control': '1'}
        req = urllib.request.Request(self.base + path, data=None if body is None else json.dumps(body).encode(), headers=h)
        try:
            with urllib.request.urlopen(req, timeout=5) as r: return r.status, json.loads(r.read()) if 'json' in r.headers.get('Content-Type', '') else r.read()
        except urllib.error.HTTPError as e: return e.code, json.loads(e.read())

    def test_flow(self):
        self.assertEqual(self.call('/start.js')[0], 200)
        code, d = self.call('/api/start'); self.assertEqual((code, d['start']['reason'], d['start']['back_m']), (200, 'START_NO_REF', 0.0))
        self.assertEqual(self.call('/api/start/set', dict(back_m=2.5))[1]['reason'], 'START_NO_REF')
        code, d = self.call('/api/start/rebuild-ref', {}); self.assertEqual((code, d['back_m'], d['start']['consistent']), (200, 0.0, True))
        code, d = self.call('/api/start/set', dict(back_m=2.5))
        self.assertEqual((code, d['back_m'], d['D'], d['left_bias_on'], len(d['changed'])), (200, 2.5, 5.69, True, 12)); self.assertTrue(d['backup'].startswith('startcfg_'))
        code, d = self.call('/api/pid/status'); self.assertEqual((code, d['service'], d['start']['back_m'], d['start']['consistent']), (200, 'mobile-service-r7', 2.5, True))
        for v, want in (('abc', 'START_BAD_INPUT'), (None, 'START_BAD_INPUT'), (7.5, 'START_RANGE'), (-1, 'START_RANGE'), (1.25, 'START_STEP')):
            self.assertEqual(self.call('/api/start/set', dict(back_m=v)), (409, dict(ok=False, reason=want, problems=[])), v)
        self.assertIn('start-cfg set back 2.5', S.ACCESS.read_text(encoding='utf8'))

    def test_refusals(self):
        self.call('/api/start/rebuild-ref', {}); before = raw(S.START_CFG_DIR)
        for st in ('running', 'paused'):
            self.pid['run'] = dict(id='r', state=st); self.assertEqual(self.call('/api/start/set', dict(back_m=3.0))[1]['reason'], 'RUN_ACTIVE')
        self.pid['run'] = None; S.RACE.armed = True
        self.assertEqual(self.call('/api/start/set', dict(back_m=3.0))[1]['reason'], 'RACE_ARMED')
        self.assertEqual(self.call('/api/start/rebuild-ref', {})[1]['reason'], 'RACE_ARMED')
        S.RACE.armed = False
        def down(path, body=None, timeout=4.): raise RuntimeError('dashboard unreachable')
        keep = S.upstream; S.upstream = down
        try: self.assertEqual(self.call('/api/start/set', dict(back_m=3.0))[1]['reason'], 'CAR_UNREACHABLE')   # cannot tell whether a run is active
        finally: S.upstream = keep
        self.assertEqual(raw(S.START_CFG_DIR), before)


if __name__ == '__main__':
    unittest.main()
