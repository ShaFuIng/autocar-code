import json, tempfile, unittest
from pathlib import Path
import start_bias as B


class T(unittest.TestCase):
    def cfg(self, **k):
        d = tempfile.mkdtemp(); p = Path(d) / 'start_bias.json'; p.write_text(json.dumps(k), encoding='utf8'); return B.load_config(p)

    def test_load(self):
        self.assertIsNone(B.load_config(Path(tempfile.mkdtemp()) / 'none.json'))
        self.assertIsNone(self.cfg(enabled=False, bias_us=-35, hold_s=.9))
        self.assertIsNone(self.cfg(enabled=True, bias_us=-350, hold_s=.9))
        self.assertIsNone(self.cfg(enabled=True, bias_us=-35, hold_s=9))
        self.assertIsNone(self.cfg(enabled=True, bias_us=True, hold_s=.9))
        self.assertIsNone(self.cfg(enabled=True, bias_us=0, hold_s=.9))
        self.assertEqual(self.cfg(enabled=True, bias_us=-35, hold_s=.9, fade_s=.6), dict(bias_us=-35., hold_s=.9, fade_s=.6))
        self.assertEqual(B.load_config(Path(__file__).with_name('on_start_bias') / 'start_bias.json'), dict(bias_us=-35., hold_s=.9, fade_s=.6))

    def test_profile(self):
        c = dict(bias_us=-35., hold_s=.9, fade_s=.6)
        self.assertEqual(B.bias(c, -.1), (0., False)); self.assertEqual(B.bias(c, 0.), (-35., False)); self.assertEqual(B.bias(c, .89), (-35., False))
        b, d = B.bias(c, 1.2); self.assertAlmostEqual(b, -17.5); self.assertFalse(d)
        self.assertEqual(B.bias(c, 1.5), (0., True)); self.assertEqual(B.bias(c, 60.), (0., True))
        self.assertEqual(B.bias(dict(c, fade_s=0.), .9), (0., True))

    def test_apply(self):
        c = dict(bias_us=-35., hold_s=.9, fade_s=.6)
        st, inf, done = B.apply(c, .3, 1490., 780., 2050.); self.assertEqual((st, done), (1455., False)); self.assertEqual(inf['bias_us'], -35.); self.assertEqual(inf['pid_steer_us'], 1490.)
        self.assertEqual(B.apply(c, .3, 790., 780., 2050.)[0], 780.)          # clipped to the servo range
        self.assertEqual(B.apply(c, 2., 1490., 780., 2050.), (1490., None, True))
        self.assertEqual(B.apply(c, -.2, 1490., 780., 2050.), (1490., None, False))


if __name__ == '__main__':
    unittest.main()
