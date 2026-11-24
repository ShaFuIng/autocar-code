"""unit tests of land4.py (land4-r1). Run: python -B tests/test_land4.py   (from the landing4 folder)"""
import json, os, sys, tempfile, unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
for p in (HERE.parent / 'candidate', HERE.parent / 'cand'):
    if (p / 'land4.py').exists(): sys.path.insert(0, str(p)); break
import land4

C0, LO, HI = 1475., 780., 2050.
M = land4.M_PER_COUNT


def cfg(slope=True, land=True, **kw):
    raw = dict(enabled=True, theta=[[-1.7, 0.0], [0.0, -11.0], [0.4, -13.0], [1.0, -39.0], [1.5, -51.0], [3.0, -79.0]],
               kappa=[[-1.7, 0.0], [-1.5, 7.0], [-0.1, 7.0], [0.2, 5.0], [0.6, 42.0], [0.8, 48.0], [1.2, 28.0], [1.5, 11.0], [3.0, 26.0]],
               e_ref=[[-1.6, 0.1], [-0.05, 0.1], [0.0, 0.16], [0.6, 0.83], [1.3, 0.09], [3.0, 0.38]], slope=dict(enabled=slope), land=dict(enabled=land))
    raw.update(kw)
    return land4.merge(raw)


class Car:
    """feeds a Land4 with a scripted pass: upper straight -> slope -> landing; v constant, 30 frames/s, MCU status every 0.2 s."""
    def __init__(self, L, v=1.1, yaw0=100.0):
        self.L = L; self.v = v; self.t = 50.0; self.s = 0.; self.yaw = yaw0; self.land_t = None; self.enc = None; self.ms = None; self.next = self.t

    def frame(self, upper=False, down_tail=False, landed=False, chain=C0, near=0.1, valid=True, yaw=None, gz=0., imu=True, last_sent=C0):
        self.t += 0.035; self.s += self.v * 0.035
        if self.t >= self.next: self.enc = int(self.s / M) + 5000; self.ms = int(self.t * 1000); self.next += 0.2
        if yaw is not None: self.yaw = yaw
        if landed and self.land_t is None: self.land_t = self.t
        return self.L.step(self.t, valid, near, self.yaw if imu else None, gz if imu else None, self.enc, self.ms, self.v, upper, down_tail, self.land_t if landed else None, chain, C0, LO, HI, last_sent)

    def upper(self, n=45, yaw=100.0):
        for _ in range(n): r = self.frame(upper=True, yaw=yaw)
        return r


class T(unittest.TestCase):
    def test_01_config_off(self):
        self.assertIsNone(land4.merge(None)); self.assertIsNone(land4.merge({}))
        self.assertIsNone(land4.merge(dict(enabled=False, slope=dict(enabled=True))))
        self.assertIsNone(land4.merge(dict(enabled=True)))                                   # no stage enabled, no track_only
        self.assertIsNone(land4.merge(dict(enabled=True, slope=dict(enabled=False), land=dict(enabled=False))))
        self.assertIsNotNone(land4.merge(dict(enabled=True, track_only=True)))
        self.assertIsNotNone(land4.merge(dict(enabled=True, slope=dict(enabled=True))))
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(land4.load_config(os.path.join(d, 'absent.json')))
            p = os.path.join(d, 'bad.json'); open(p, 'w').write('{not json'); self.assertIsNone(land4.load_config(p))
            self.assertIsNone(land4.Land4.from_file(os.path.join(d, 'absent.json')))

    def test_02_idle_untouched(self):
        L = land4.Land4(cfg()); c = Car(L)
        for _ in range(60):
            st, info = c.frame(upper=False, chain=1612.3)
            self.assertEqual(st, 1612.3); self.assertIsNone(info)
        st, info = c.upper(); self.assertEqual(st, C0); self.assertIsNone(info); self.assertEqual(L.state, 'IDLE')
        self.assertFalse(L.wants_imu(False, False)); self.assertTrue(L.wants_imu(True, False)); self.assertTrue(L.wants_imu(False, True))

    def test_03_no_reference_no_action(self):
        L = land4.Land4(cfg()); c = Car(L)
        for k in range(120):
            st, info = c.frame(down_tail=True, landed=k > 60, chain=1700.)
            self.assertEqual(st, 1700.)
            if info: self.assertIn(info['state'], ('OFF',)); self.assertEqual(info['reason'], 'NO_REF_SAMPLES')
        L = land4.Land4(cfg()); c = Car(L)
        for k in range(45): c.frame(upper=True, yaw=100. + (9. if k % 2 else 0.))            # heading spread 9 deg > ref_spread_deg
        st, info = c.frame(down_tail=True, chain=1700.); self.assertEqual(st, 1700.); self.assertEqual(info['reason'], 'REF_SPREAD')

    def test_04_slope_heading_sign(self):
        res = {}
        for dy in (3., -3., 0.):
            L = land4.Land4(cfg()); c = Car(L); c.upper()
            for _ in range(12): st, info = c.frame(down_tail=True, yaw=100. + dy, chain=C0)   # u ~ -2.2..-1.8: target 0, camera on its reference
            self.assertEqual(info['state'], 'SLOPE'); self.assertAlmostEqual(info['psi'], dy, places=1)
            self.assertAlmostEqual(info['head_us'], 10. * dy, delta=1.); res[dy] = st
        self.assertGreater(res[3.], res[0.] + 20.)        # pointing left of the reference -> steer right
        self.assertLess(res[-3.], res[0.] - 20.)
        self.assertLess(abs(res[0.] - C0), 60.)            # only the small slope feed-forward

    def test_05_mouth_spike_ignored(self):
        L = land4.Land4(cfg()); c = Car(L); c.upper(); outs = []
        for k in range(15):
            ch = 2049. if k == 1 else (1775. if 2 <= k <= 8 else C0)                          # camera chain: false +1.00 frame, then the 0.3 s jump-guard hold
            st, info = c.frame(down_tail=True, chain=ch, near=1.0 if k == 1 else -0.2); outs.append(st)
        self.assertLess(max(outs), C0 + 70.); self.assertGreater(min(outs), C0 - 40.)          # the chain asked 2049 / 1775; the law stays near the centre (slope feed-forward <= 50 us)
        self.assertLess(max(abs(b_ - a_) for a_, b_ in zip(outs, outs[1:])), 41.)

    def test_06_camera_single_frame_and_limit(self):
        L = land4.Land4(cfg()); c = Car(L); c.upper(); cams = []
        for k in range(45):
            st, info = c.frame(down_tail=True, near=1.0 if k == 30 else 0.1); cams.append(info['cam_deg'])
        self.assertLess(max(abs(x) for x in cams), 0.2)                                       # one frame at +1.0 does not pass the median of 3
        L = land4.Land4(cfg()); c = Car(L); c.upper()
        for k in range(55): st, info = c.frame(down_tail=True, near=1.0)
        self.assertAlmostEqual(info['cam_deg'], -5.0, places=2)                               # persistent error: target shifted right, limited to cam_lim
        L = land4.Land4(cfg()); c = Car(L); c.upper()
        for k in range(8): st, info = c.frame(down_tail=True, near=1.0)
        self.assertEqual(info['cam_deg'], 0.0)                                                # no camera term before cam_from_m

    def run_pass(self, L, yaw_fn=None, landed_at=57, n=110, chain=C0, **kw):
        c = Car(L); c.upper(); rows = []
        for k in range(n):
            y = 100. if yaw_fn is None else yaw_fn(k)
            st, info = c.frame(down_tail=True, landed=k >= landed_at, yaw=y, chain=chain, **kw); rows.append((st, info))
        return rows

    def test_07_landing_limits_and_step(self):
        L = land4.Land4(cfg())
        rows = self.run_pass(L, yaw_fn=lambda k: 100. - 11. * min(1., max(0., (k - 14) / 43.)))   # follows the slope ramp, then keeps -11 (lags the table after the landing)
        prev = None
        for st, info in rows:
            if info is None or info.get('state') not in ('SLOPE', 'LAND'): continue
            self.assertLessEqual(st, C0 + 260. + 1e-6); self.assertGreaterEqual(st, C0 - 150. - 1e-6)
            if prev is not None and info.get('w', 1.) == 1.: self.assertLessEqual(abs(st - prev), 60. * 3 + 1e-6)
            prev = st
        land = [(st, i) for st, i in rows if i and i.get('state') == 'LAND' and i.get('w') == 1.]
        self.assertTrue(land); self.assertGreater(max(st for st, _ in land), C0 + 200.)       # feed-forward + heading lag push right after the landing
        self.assertAlmostEqual(land[0][1]['x_m'], 0.0, delta=0.06)

    def test_08_hand_over_and_done(self):
        L = land4.Land4(cfg()); rows = self.run_pass(L, n=140, chain=1600.)
        states = [i.get('state') if i else None for _, i in rows]
        self.assertIn('END', states); k = states.index('END'); self.assertEqual(rows[k][1]['reason'], 'HANDED_OVER'); self.assertEqual(rows[k][0], 1600.)
        self.assertTrue(all(s is None for s in states[k + 1:])); self.assertTrue(all(st == 1600. for st, _ in rows[k + 1:]))
        ws = [i['w'] for _, i in rows if i and i.get('state') == 'LAND']
        self.assertEqual(ws[0], 1.0); self.assertLess(ws[-1], 0.5); self.assertTrue(all(a >= b for a, b in zip(ws, ws[1:])))
        self.assertGreater(rows[k][1]['x_m'], 1.5); self.assertLess(rows[k][1]['x_m'], 1.62)
        self.assertEqual(L.state, 'DONE')
        c = Car(L); st, info = c.frame(down_tail=False, chain=1500.); self.assertEqual(L.state, 'IDLE'); self.assertIsNone(info)

    def test_09_despike(self):
        def yf(k): return 100. - 0.4 * k + (5. if k >= 40 else 0.)                            # steady right turn + one +5 deg jump (gyro clipped at the impact)
        L = land4.Land4(cfg()); rows = self.run_pass(L, yaw_fn=yf, gz=-0.4 / 0.035)
        psis = [i['psi'] for _, i in rows if i and 'psi' in i]
        self.assertLess(max(abs(b - a) for a, b in zip(psis, psis[1:])), 1.0)                 # no jump left in the heading used by the law
        self.assertAlmostEqual(L.glitch, 5.0, delta=0.05); self.assertEqual(L.glitches, 1)
        L = land4.Land4(cfg(spike_on=False)); rows = self.run_pass(L, yaw_fn=yf, gz=-0.4 / 0.035)
        psis = [i['psi'] for _, i in rows if i and 'psi' in i]
        self.assertGreater(max(abs(b - a) for a, b in zip(psis, psis[1:])), 4.0)

    def test_09b_despike_cannot_run_away(self):
        # the yaw really turns at -110 dps while the gyro reading passed in stays 0 (stale): every frame looks like a 3.85 deg excess.
        L = land4.Land4(cfg()); rows = self.run_pass(L, yaw_fn=lambda k: 100. - 3.85 * max(0, k - 20), gz=0.)
        self.assertLessEqual(abs(L.glitch), 12.0 + 1e-6)                                      # total removed is bounded (glitch_max_deg)
        self.assertLessEqual(L.glitches, 4)                                                   # never two frames in a row, then the bound stops it
        L = land4.Land4(cfg()); rows = self.run_pass(L, yaw_fn=lambda k: 100. + (5. if k >= 40 else 0.), gz=250.9)   # clipped readings are not used as the prediction
        self.assertEqual(L.glitches, 1); self.assertAlmostEqual(L.glitch, 5.0, delta=0.05)

    def test_10_stage_switches(self):
        L = land4.Land4(cfg(slope=True, land=False)); rows = self.run_pass(L, chain=1700., n=110)
        ld = [(st, i) for st, i in rows if i and i.get('state') == 'LAND']
        self.assertTrue(all('shadow_us' in i for _, i in ld))
        outs = [st for st, _ in ld]
        self.assertLess(outs[0], 1700.); self.assertEqual(outs[-1], 1700.)                    # rate-limited return to the chain, then the chain itself
        self.assertTrue(all(b - a <= 40. * 3 + 1e-6 for a, b in zip(outs, outs[1:])))
        L = land4.Land4(cfg(slope=False, land=True)); rows = self.run_pass(L, chain=1490., n=110)
        for st, i in rows:
            if i and i.get('state') == 'SLOPE': self.assertEqual(st, 1490.); self.assertIn('shadow_us', i); self.assertFalse(i['applied'])
        self.assertTrue(any(i and i.get('state') == 'LAND' and i['applied'] for _, i in rows))
        L = land4.Land4(land4.merge(dict(enabled=True, track_only=True))); rows = self.run_pass(L, chain=1533., n=150)
        self.assertTrue(all(st == 1533. for st, _ in rows)); self.assertTrue(any(i and 'shadow_us' in i for _, i in rows))

    def test_11_lost_frames_and_no_imu(self):
        L = land4.Land4(cfg()); c = Car(L); c.upper()
        for k in range(20): st, info = c.frame(down_tail=True, yaw=102., near=0.1)
        ef = L.ef
        for k in range(10):
            st, info = c.frame(down_tail=True, yaw=102., valid=False, near=None, chain=1300.)
            self.assertEqual(info['state'], 'SLOPE'); self.assertGreater(st, C0)              # gyro law on lost frames, not the held command
        self.assertEqual(L.ef, ef)                                                            # camera filter untouched by lost frames (within cam_hold_s)
        st, info = c.frame(down_tail=True, imu=False, chain=1444.)
        self.assertEqual(st, 1444.); self.assertEqual(info['reason'], 'NO_IMU')

    def test_12_odometer(self):
        o = land4.Odo(); t = 10.; s = 0.; v = 1.2; ds = []
        for k in range(60):
            t += 0.035; s += v * 0.035
            if k % 6 == 0: o.feed(t, int(s / M), int(t * 1000))
            else: o.feed(t, o.enc, o.ms)
            ds.append(o.dist(t))
        self.assertAlmostEqual(o.speed(), v, delta=0.02)
        inc = [b - a for a, b in zip(ds[12:], ds[13:])]
        self.assertGreater(min(inc), -0.012); self.assertLess(max(inc), 0.09)                 # smooth between the 5 Hz updates (no 0.25 m steps)
        self.assertAlmostEqual(ds[-1] - ds[12], v * 0.035 * 47, delta=0.05)

    def test_13_window_closed(self):
        L = land4.Land4(cfg()); c = Car(L); c.upper()
        for k in range(10): c.frame(down_tail=True)
        st, info = c.frame(down_tail=False, chain=1500.)
        self.assertEqual(st, 1500.); self.assertEqual(info['reason'], 'WINDOW_CLOSED'); self.assertEqual(L.state, 'IDLE')

    def test_14_first_command_starts_from_last_sent(self):
        L = land4.Land4(cfg()); c = Car(L); c.upper()
        st, info = c.frame(down_tail=True, yaw=110., chain=2049., last_sent=1480.)             # 10 deg left of the reference at the mouth
        self.assertLessEqual(st, 1480. + 40. * 3 + 1e-6); self.assertGreater(st, 1480.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
