import copy,json,tempfile,time,unittest,uuid
from pathlib import Path
from field_trials import Trials,pure_pursuit

class Store:
    def __init__(self,root):
        self.root=Path(root);self.rev=1
        self.parameters=dict(pwm=15,steer_rate=300,lookahead_min=.8,lookahead_max=1.2,lookahead_time=1,curve_scale=.8)
    def get(self,key):return dict(key=key,rev=self.rev,data={'parameters':copy.deepcopy(self.parameters)} if key=='tracking-proposal' else {})

class FieldTrialsTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.store=Store(self.tmp.name)
        def forbidden():raise AssertionError('Synthetic mode must never contact hardware')
        self.m=Trials(self.store,forbidden,forbidden)
    def tearDown(self):
        self.m.cancel.set()
        for _ in range(40):
            if not self.m.active:break
            time.sleep(.05)
        self.tmp.cleanup()
    def payload(self,**kw):return dict(request_id=uuid.uuid4().hex,mode='synthetic',seconds=1,revision=1,segment='unit',scenario='left',**kw)
    def done(self,id):
        for _ in range(80):
            d=self.m.detail(id)
            if d['status']!='recording':return d
            time.sleep(.05)
        self.fail('timeout')
    def test_pp_sign_and_rate(self):
        a=pure_pursuit([[1,.5]],.8,.2,100,.2);b=pure_pursuit([[1,-.5]],.8,.2,100,.2)
        self.assertLess(a['proposal_us'],1300);self.assertGreater(b['proposal_us'],1300)
        self.assertLessEqual(abs(a['proposal_us']-1300),20);self.assertFalse(a['hardware_output'])
    def test_path_loss_and_unknown_center(self):
        self.assertEqual(pure_pursuit([],.8,.2,300,.2)['pwm_scale'],0)
        self.assertFalse(pure_pursuit([[1,.001]],.8,.2,300,.2)['valid'])
        self.assertFalse(pure_pursuit([[.1,.1]],.05,.2,300,.2)['valid'])
    def test_frozen_snapshot_dedup_and_completion(self):
        p=self.payload();r=self.m.start(p);self.assertEqual(self.m.start(p)['id'],r['id'])
        self.store.parameters['pwm']=20
        with self.assertRaises(ValueError):self.m.start(self.payload())
        d=self.done(r['id']);self.assertEqual(d['status'],'completed');self.assertEqual(d['parameter_snapshot']['data']['parameters']['pwm'],15)
        self.assertTrue(d['events']);self.assertFalse(d['hardware_output'])
        self.assertEqual(self.m.start(p)['id'],r['id'])
    def test_invalid_modes_and_versions(self):
        p=self.payload();p['mode']='drive'
        with self.assertRaises(ValueError):self.m.start(p)
        p=self.payload();p['revision']=99
        with self.assertRaises(ValueError):self.m.start(p)
        p=self.payload();p['seconds']=float('nan')
        with self.assertRaises(ValueError):self.m.start(p)
    def test_stale_observer_and_stop_is_record_only(self):
        self.m.read_status=lambda:dict(uart_age_s=9,camera_age_s=0)
        p=self.payload();p.update(mode='observe',record_only_confirmed=True)
        r=self.m.start(p);d=self.done(r['id']);self.assertEqual(d['status'],'failed');self.assertEqual(d['samples'],0)
    def test_observer_image_time_windows(self):
        self.m.read_status=lambda:dict(uart_age_s=0,camera_age_s=.1,vehicle={'state':'LOCKED'})
        self.m.read_camera=lambda:b'\xff\xd8FAKE_JPEG_FOR_TEST'
        p=self.payload();p.update(mode='observe',record_only_confirmed=True)
        d=self.done(self.m.start(p)['id']);e=d['events'][0]
        self.assertTrue(e['exposure_time_unknown']);self.assertGreaterEqual(e['image_received_s'],e['image_request_s']);self.assertIsNone(e['proposal'])
    def test_review_append_and_path_validation(self):
        d=self.done(self.m.start(self.payload())['id'])
        for note in ['first','second']:self.m.review(dict(id=d['id'],t=.2,note=note))
        self.assertEqual(len(self.m.detail(d['id'])['reviews']),2)
        with self.assertRaises(ValueError):self.m.detail('../x')
    def test_restart_does_not_resume(self):
        id=uuid.uuid4().hex;p=Path(self.tmp.name)/'field_trials'/id;p.mkdir()
        self.m.write(p/'meta.json',dict(id=id,status='recording',created=0))
        Trials(self.store)
        self.assertEqual(json.loads((p/'meta.json').read_text())['status'],'interrupted')

if __name__=='__main__':unittest.main()
