import unittest
from visual_pid import DEFAULTS,PID,path_error,replay,validate

class VisualPIDTest(unittest.TestCase):
    def test_sign_and_slew(self):
        for error,sign in ((.5,1),(-.5,-1)):
            c=PID(DEFAULTS);self.assertFalse(c.step(0,error)['valid'])
            r=c.step(.1,error)
            self.assertGreater((r['steer_us']-1300)*sign,0)
            self.assertLessEqual(abs(r['steer_us']-1300),30)
            self.assertFalse(r['hardware_output'])
    def test_loss_and_warmup(self):
        c=PID(DEFAULTS);c.step(0,.2);c.step(.1,.2)
        self.assertEqual(c.step(.2,None)['pwm'],0)
        self.assertEqual(c.step(.3,.2)['reason'],'WARMUP')
        self.assertFalse(c.step(2,.2)['valid'])
    def test_antiwindup(self):
        c=PID(dict(DEFAULTS,kp=600,ki=200,steer_rate=50))
        for i in range(20):c.step(i*.1,1)
        self.assertEqual(c.i,0)
    def test_invalid_and_geometry(self):
        for v in (float('nan'),float('inf'),True,-1):
            with self.assertRaises(ValueError):validate(dict(DEFAULTS,kp=v))
        self.assertIsNone(path_error([[100,50],[200,100]],.75))
        self.assertAlmostEqual(path_error([[639.5,0],[639.5,719]],.75),0)
        self.assertGreater(path_error([[900,0],[900,719]],.75),0)
    def test_replay(self):
        r={'id':'abc','source':'file','models':{'v4':[{'frame':i,'capture_s':i*.1,'A':{'path':[[800,0],[800,719]]}} for i in range(5)]}}
        result=replay(r,'v4','A',DEFAULTS)
        self.assertEqual(result['valid_frames'],4)
        self.assertFalse(result['imu_used'])
        self.assertFalse(result['hardware_output'])
    def test_reserved_imu_does_not_change_output(self):
        a,b=PID(DEFAULTS),PID(DEFAULTS)
        for i in range(4):
            t=i*.1
            self.assertEqual(a.step(t,.3),b.step(t,.3,{'sample_t':t,'yaw_rate_dps':100,'valid':True}))

if __name__=='__main__':unittest.main()
