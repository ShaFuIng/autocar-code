import unittest
from visual_pid import PID,DEFAULTS,BACKLASH_DEFAULTS,validate

class BacklashTests(unittest.TestCase):
    def controller(self,**changes):
        return PID(dict(DEFAULTS,**dict(BACKLASH_DEFAULTS,kp=100,kd=0,steer_rate=800,
            backlash_enabled=1,backlash_left_us=40,backlash_right_us=60,
            backlash_threshold_us=10,backlash_rate=800,**changes)))

    def test_disabled_exact_baseline(self):
        a,b=PID(DEFAULTS),PID(dict(DEFAULTS,**BACKLASH_DEFAULTS))
        for i,e in enumerate([0,.2,.4,.1,-.2,-.5,.2]):
            self.assertEqual(a.step(i*.1,e)['steer_us'],b.step(i*.1,e)['steer_us'])

    def test_direction_memory_not_center_sign(self):
        c=self.controller();c.step(0,.4);c.step(.1,.4)
        r=c.step(.2,.2)  # Still right of center, but command moved left.
        self.assertEqual(r['backlash_direction'],-1)
        self.assertEqual(r['backlash_offset_us'],-40)
        self.assertEqual(r['backlash_reversals'],1)
        for i in range(3,20):r=c.step(i*.1,.2)
        self.assertEqual(r['backlash_reversals'],1)
        self.assertEqual(r['steer_us'],1280)
        r=c.step(2,.4)
        self.assertEqual(r['backlash_offset_us'],60)
        self.assertEqual(r['backlash_reversals'],2)

    def test_noise_no_repeated_switch(self):
        c=self.controller();c.step(0,.4);c.step(.1,.4)
        for i,e in enumerate([.39,.38,.4,.37,.4],2):
            self.assertEqual(c.step(i*.1,e)['backlash_reversals'],0)

    def test_speed_limit_travel_limit_reset(self):
        p=dict(DEFAULTS,**BACKLASH_DEFAULTS)
        p.update(backlash_enabled=1,backlash_left_us=200,backlash_right_us=200,
                 backlash_rate=50,kp=600,kd=0,steer_rate=800,min_us=1250,max_us=1350)
        c=PID(p);previous=1300
        for i,e in enumerate([0,.5,.5,-.5,-.5,.5,.5]):
            r=c.step(i*.1,e)
            if r['valid']:
                self.assertLessEqual(abs(r['steer_us']-previous),5.00001)
                self.assertTrue(1250<=r['steer_us']<=1350)
                previous=r['steer_us']
        r=c.step(.7,None);self.assertEqual(r['pwm'],0)
        self.assertEqual(c.bl_reversals,0)
        self.assertEqual(c.bl_offset,0)
        self.assertEqual(c.step(.8,.3)['reason'],'WARMUP')

    def test_no_drift_after_many_reversals(self):
        c=self.controller();c.step(0,0)
        for i in range(1,101):
            r=c.step(i*.1,.4 if i%2 else -.4)
            self.assertIn(r['backlash_offset_us'],(0,-40,60))
            self.assertTrue(1220<=r['steer_us']<=1400)

    def test_invalid_config(self):
        for k,v in [('backlash_enabled',.5),('backlash_left_us',float('nan')),
                    ('backlash_right_us',201),('backlash_rate',0),('backlash_threshold_us',0)]:
            with self.assertRaises(ValueError):validate(dict(DEFAULTS,**{k:v}))

if __name__=='__main__':unittest.main()
