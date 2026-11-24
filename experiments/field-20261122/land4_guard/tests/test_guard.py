"""land4-r2 landing guard (land_min_m / land_min_s). Run: python -B tests/test_guard.py (from land4_guard)."""
import importlib.util, sys, unittest
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / 'candidate'))
import land4 as NEW
import test_land4 as T0
spec = importlib.util.spec_from_file_location('land4_old', HERE.parent / 'base' / 'land4.py'); OLD = importlib.util.module_from_spec(spec); spec.loader.exec_module(OLD)
C0 = T0.C0


def run(mod, bounce=(), landed_at=57, n=110, dead_odo=False, **kw):
    saved = T0.land4; T0.land4 = mod
    try:
        L = mod.Land4(T0.cfg(**kw)); c = T0.Car(L); c.upper(); rows = []
        for k in range(n):
            if dead_odo: c.enc = None; c.ms = None; c.next = 1e9
            lv = (k in bounce) or k >= landed_at
            if lv and c.land_t is None: c.land_t = c.t
            st, info = L.step(*(lambda t: (t,))(0)) if False else c.frame(down_tail=True, landed=lv, yaw=100. - 11. * min(1., max(0., (k - 14) / 43.)))
            rows.append((round(st, 6), None if info is None else info.get('state'), None if info is None else info.get('x_m')))
        return rows, L
    finally: T0.land4 = saved


class G(unittest.TestCase):
    def test_keys_absent_is_r1(self):
        for b in ((), (1,), (1, 2), (20,)):
            self.assertEqual(run(OLD, bounce=b)[0], run(NEW, bounce=b)[0], b)
        r, _ = run(NEW, bounce=(1,)); self.assertEqual(r[1][1], 'LAND')            # the 54cb failure reproduced: LAND one frame after the mouth

    def test_guard_ignores_the_mouth_bounce(self):
        clean, _ = run(NEW, land_min_m=1.2, land_min_s=1.0)
        for b in ((1,), (1, 2), (0, 3), (5, 6, 7)):
            r, L = run(NEW, bounce=b, land_min_m=1.2, land_min_s=1.0)
            self.assertEqual(r, clean, b); self.assertEqual(L.early, len(b))
        st = [s for _, s, _ in clean]; self.assertTrue(all(s == 'SLOPE' for s in st[:57])); self.assertEqual(st[57], 'LAND')
        self.assertEqual(clean, run(NEW)[0])                                       # a normal pass: the guard changes nothing

    def test_time_alone_with_a_dead_odometer(self):
        r, _ = run(NEW, dead_odo=True, land_min_m=1.2, land_min_s=1.0)
        self.assertEqual(r, run(NEW, dead_odo=True)[0])                            # landing at frame 57 = 2.0 s on the slope: accepted by time
        r, _ = run(NEW, landed_at=20, land_min_m=1.2, land_min_s=1.0)              # 0.7 s / 0.77 m: refused until 1.0 s (frame 28) or 1.2 m (frame 31)
        k = [i for i, x in enumerate(r) if x[1] == 'LAND'][0]; self.assertIn(k, (28, 29, 30, 31))


if __name__ == '__main__': unittest.main()
