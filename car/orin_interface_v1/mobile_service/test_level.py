"""mobile-service-r4 (2026-11-22): flat-ground IMU baseline (imu_level). No car, no HTTP: the functions are called directly."""
import json, tempfile, unittest
from pathlib import Path
import server as S


def still(n=20, pitch=0.30, gyro=0.05):
    return [dict(t=100. + .02 * i, pitch_deg=pitch + (.01 if i % 2 else -.01), roll_deg=1.2, gyro_dps=[gyro, -gyro, .0]) for i in range(n)]


class Level(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); d = Path(self.tmp.name)
        self.keep = (S.BOOST, S.LEVEL_BACKUPS, S.SERVICE, S.ACCESS, S.upstream, S.IMU)
        S.BOOST = d / 'imu_boost.json'; S.LEVEL_BACKUPS = d / 'backups'; S.SERVICE = d / 'service.json'; S.ACCESS = d / 'access.log'
        self.cfg = dict(enabled=True, uphill_pwm=25, on_deg=5.0, flat_pitch_deg=1.1, note='x', pwm_rate=10.0)
        S.BOOST.write_text(json.dumps(self.cfg, indent=1), encoding='utf8')
        self.pid = dict(run=None); S.upstream = lambda path, body=None, timeout=4.: (200, self.pid)
        S.IMU = S.ImuWatch(); S.IMU.state = 'ok'

    def tearDown(self):
        S.BOOST, S.LEVEL_BACKUPS, S.SERVICE, S.ACCESS, S.upstream, S.IMU = self.keep; self.tmp.cleanup()

    def read(self): return json.loads(S.BOOST.read_text(encoding='utf8'))

    def test_sets_only_the_baseline(self):
        ok, code, d = S.imu_level('test', still(pitch=0.30))
        self.assertEqual((ok, code), (True, None)); self.assertEqual(d, dict(static_pitch_deg=0.3, old_flat_pitch_deg=1.1, flat_pitch_deg=1.1))
        ok, code, d = S.imu_level('test', still(pitch=1.00))
        self.assertEqual(d['flat_pitch_deg'], 1.8)
        new = self.read(); self.assertEqual(new, dict(self.cfg, flat_pitch_deg=1.8))                  # every other key untouched
        b = sorted(S.LEVEL_BACKUPS.glob('imulevel_*/imu_boost.json')); self.assertTrue(b)
        self.assertEqual(json.loads(b[0].read_text(encoding='utf8'))['flat_pitch_deg'], 1.1)          # the old file was kept
        self.assertIn('repair imu level', S.ACCESS.read_text(encoding='utf8'))

    def test_offset_from_service_json(self):
        S.SERVICE.write_text(json.dumps(dict(imu_level_offset_deg=1.1)), encoding='utf8')
        self.assertEqual(S.imu_level('test', still(pitch=0.95))[2]['flat_pitch_deg'], 2.05)          # the 2026-11-21 pair: 0.95 -> 2.05

    def test_refusals_leave_the_file(self):
        before = S.BOOST.read_text(encoding='utf8')
        moving = still(); moving[5]['gyro_dps'] = [4.0, 0, 0]
        wobble = [dict(d, pitch_deg=d['pitch_deg'] + (.5 if i % 2 else -.5)) for i, d in enumerate(still())]
        for samples, want in ((moving, 'IMU_NOT_STILL'), (wobble, 'IMU_NOT_STILL'), (still(pitch=-15.), 'IMU_NOT_FLAT'), (still(pitch=5.5), 'IMU_LEVEL_RANGE'),
                              (still(n=5), 'IMU_FAULT')):
            ok, code, _ = S.imu_level('test', samples); self.assertEqual((ok, code), (False, want), want)
        for state in ('running', 'paused'):
            self.pid['run'] = dict(id='r', state=state); self.assertEqual(S.imu_level('test', still())[:2], (False, 'RUN_ACTIVE'))
        self.pid['run'] = dict(id='r', state='ended'); S.IMU.state = 'frozen'
        self.assertEqual(S.imu_level('test', still())[:2], (False, 'IMU_FAULT'))
        self.assertEqual(S.BOOST.read_text(encoding='utf8'), before); self.assertFalse(S.LEVEL_BACKUPS.exists())

    def test_level_info_for_the_page(self):
        self.assertEqual(S.level_info(0.28), dict(flat_pitch_deg=1.1, calibrated_static_deg=0.3, diff_deg=-0.02))
        self.assertEqual(S.level_info(1.69)['diff_deg'], 1.39)
        self.assertIsNone(S.level_info(None)['diff_deg'])
        S.IMU.last = dict(pitch_deg=0.9, roll_deg=1.0, t=0.); self.assertEqual(S.imu_now()['level']['diff_deg'], 0.6)
        S.imu_level('test', still(pitch=0.90)); self.assertEqual(S.imu_now()['level'], dict(flat_pitch_deg=1.7, calibrated_static_deg=0.9, diff_deg=0.0))
        S.BOOST.unlink(); self.assertIsNone(S.level_info(0.3))

    def test_samples_are_distinct_time_stamps(self):
        feed = iter([dict(t=1., pitch_deg=.3), dict(t=1., pitch_deg=.3), dict(t=1.02, pitch_deg=.31), ValueError(), dict(t=1.04, pitch_deg=.29)])
        clock = iter([0., .1, .2, .3, .4, .5, 9.])

        def read():
            v = next(feed)
            if isinstance(v, Exception): raise v
            return v
        out = S.level_samples(1., read=read, sleep=lambda s: None, clock=lambda: next(clock))
        self.assertEqual([d['t'] for d in out], [1., 1.02, 1.04])


if __name__ == '__main__':
    unittest.main()
