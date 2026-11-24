import unittest
from visual_pid import PID,DEFAULTS,validate,path_features
class DirectionTests(unittest.TestCase):
 def test_disabled_identical(self):
  a=PID(DEFAULTS);b=PID(DEFAULTS)
  for i in range(6):
   self.assertAlmostEqual(a.step(i*.1,.2).get('steer_us') or 0,b.step_path(i*.1,[[767.4,0],[767.4,719]]).get('steer_us') or 0)
 def test_direction_sign(self):
  p=dict(DEFAULTS,direction_enabled=1,direction_far_row=.7,direction_weight=.5)
  for sign in [-1,1]:
   f=path_features([[639.5+sign*100,.7*719],[639.5,.75*719]],p)
   self.assertGreater(f['control_error']*sign,0)
   self.assertFalse(f['ground_geometry_applied'])
 def test_missing_and_reset(self):
  c=PID(dict(DEFAULTS,direction_enabled=1));path=[[800,480],[700,590]]
  c.step_path(0,path);self.assertTrue(c.step_path(.1,path)['valid'])
  self.assertEqual(c.step_path(.2,[[700,520],[700,590]])['pwm'],0)
  self.assertEqual(c.step_path(.3,path)['reason'],'WARMUP')
 def test_validation(self):
  for extra in [dict(direction_far_row=.75),dict(direction_weight=float('nan')),dict(direction_enabled=2)]:
   p=dict(DEFAULTS,direction_enabled=1);p.update(extra)
   with self.assertRaises(ValueError):validate(p)
if __name__=='__main__':unittest.main()
