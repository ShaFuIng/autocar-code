import unittest
from visual_pid import PID, DEFAULTS, BACKLASH_DEFAULTS, validate
from test_pid_workspace import FieldTests
from server import App

class InitialTests(unittest.TestCase):
    def make(self, **changes):
        p=dict(DEFAULTS, **BACKLASH_DEFAULTS)
        p.update(kp=100,kd=0,steer_rate=800,backlash_initial_enabled=1,
                 backlash_initial_left_us=30,backlash_initial_right_us=40,
                 backlash_initial_rate=800,backlash_rate=800)
        p.update(changes)
        return PID(p)

    def test_first_direction_once_and_reverse_clears(self):
        for sign,offset in [(-1,-30),(1,40)]:
            c=self.make();c.step(0,0)
            self.assertEqual(c.step(.1,sign*.05)['backlash_offset_us'],0)
            r=c.step(.2,sign*.2)
            self.assertEqual(r['backlash_offset_us'],offset)
            self.assertEqual(r['backlash_source'],'initial')
            for i in range(3,8):
                r=c.step(i*.1,sign*.3)
                self.assertEqual(r['backlash_initial_count'],1)
                self.assertEqual(r['backlash_offset_us'],offset)
            r=c.step(.8,-sign*.2)
            self.assertEqual(r['backlash_offset_us'],0)
            self.assertEqual(r['backlash_source'],'none')
            self.assertEqual(c.step(.9,sign*.3)['backlash_initial_count'],1)

    def test_reversal_replaces_initial_not_adds(self):
        c=self.make(backlash_enabled=1,backlash_left_us=60,backlash_right_us=70)
        c.step(0,0);self.assertEqual(c.step(.1,.2)['backlash_offset_us'],40)
        r=c.step(.2,-.2)
        self.assertEqual(r['backlash_offset_us'],-60)
        self.assertEqual(r['backlash_source'],'reversal')
        self.assertEqual(c.step(.3,.2)['backlash_offset_us'],70)

    def test_reset_rate_limits_and_invalid(self):
        c=self.make(backlash_initial_rate=50,min_us=1280,max_us=1320)
        c.step(0,0);last=1300
        for i in range(1,8):
            r=c.step(i*.1,.5)
            self.assertLessEqual(abs(r['steer_us']-last),5.00001)
            self.assertTrue(1280<=r['steer_us']<=1320)
            last=r['steer_us']
        c.step(.8,None);self.assertEqual(c.bl_initial_count,0)
        c.step(.9,0)
        self.assertEqual(c.step(1,-.3)['backlash_source'],'initial')
        for k,v in [('backlash_initial_enabled',.5),('backlash_initial_left_us',201),
                    ('backlash_initial_right_us',float('nan')),('backlash_initial_rate',0),
                    ('backlash_initial_threshold_us',0)]:
            with self.assertRaises(ValueError):validate(dict(DEFAULTS,**{k:v}))

class CapabilityTests(FieldTests):
    def test_v1_rejects_initial_enabled_without_start(self):
        self.store.save('visual-pid',0,dict(parameters=dict(DEFAULTS,backlash_initial_enabled=1)))
        self.app.integration.request.return_value={'backlash_version':1}
        with self.assertRaises(ValueError):App.post(self.app,'/api/pid/live/start',dict(parameter_revision=1))
        self.app.integration.request.assert_called_once_with('/api/pid/status')

    def test_v1_strips_disabled_new_keys_preserves_reversal(self):
        p=dict(DEFAULTS,backlash_enabled=1,backlash_right_us=40,backlash_initial_enabled=0)
        self.store.save('visual-pid',0,dict(parameters=p))
        self.app.integration.request.return_value={'backlash_version':1}
        App.post(self.app,'/api/pid/live/start',dict(parameter_revision=1))
        sent=self.app.integration.request.call_args.args[1]['parameters']
        self.assertNotIn('backlash_initial_enabled',sent)
        self.assertEqual(sent['backlash_right_us'],40)

    def test_v2_preserves_initial(self):
        p=dict(DEFAULTS,backlash_initial_enabled=1,backlash_initial_right_us=40)
        self.store.save('visual-pid',0,dict(parameters=p))
        self.app.integration.request.return_value={'backlash_version':2}
        App.post(self.app,'/api/pid/live/start',dict(parameter_revision=1))
        self.assertEqual(self.app.integration.request.call_args.args[1]['parameters'],p)

if __name__=='__main__':unittest.main()
