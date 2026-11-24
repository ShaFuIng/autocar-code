"""regression for run 47abbf59 (2026-11-21): after a 0.5 s coast + pause, the restored latest frame must not look older than max_gap."""
import unittest
import test_pause_unit as U
PR, PC, CLOCK = U.PR, U.PC, U.CLOCK

class C(unittest.TestCase):
    def setUp(self): PC._P = 0.; CLOCK[0] = 1000.
    def age_after_resume(self, coast_frames):
        lp = U.LP(); PR.json  # noqa
        cfg = dict(PR.DEFAULTS, coast_max_m=2.0); PR.load_config = lambda path=None: cfg
        PR.pause(lp, {'run_id': 'r1'})
        enc = 1000
        for _ in range(coast_frames): enc += 100; U.held(lp, 1, enc=enc)   # 16 frames = 0.53 s of coasting, 0.36 m
        U.held(lp, 400)   # 13 s standing still
        self.assertTrue(lp.status()['pause']['can_resume'], lp.status()['pause'])
        PR.resume(lp, {'run_id': 'r1'}); CLOCK[0] += .009; PR.loop_top(lp, None, None)
        return PC.monotonic() - lp.latest['capture_at'], lp.run['parameters']['max_gap'], lp.latest['frame_seq']
    def test_coast(self):
        age, gap, fs = self.age_after_resume(16); print('coast 16 frames: age %.3f s (max_gap %.2f)' % (age, gap))
        self.assertLess(age, .4); self.assertGreaterEqual(age, 0.); self.assertEqual(fs, 10)
    def test_no_coast(self):
        age, gap, fs = self.age_after_resume(0); print('no coast: age %.3f s' % age)
        self.assertLess(age, .4); self.assertGreaterEqual(age, 0.)
if __name__ == '__main__': unittest.main()
