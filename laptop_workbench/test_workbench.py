import copy
import json
import tempfile
import time
import unittest
from pathlib import Path
from server import App, Conflict, Simulator, Store, ROOT

TEST_ROOT=ROOT/'data'/'test_tmp'
TEST_ROOT.mkdir(parents=True,exist_ok=True)


class Tests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=TEST_ROOT)
        self.store = Store(self.tmp.name)
        self.now = 100.
        self.sim = Simulator(self.store, lambda: self.now)
        self.sim.configure({'action':'authorize','enabled':True})

    def tearDown(self):
        self.store.db.close()
        self.tmp.cleanup()

    def request(self, **kw):
        return dict(id='task-00000001',owner='owner-0000001',mode='fixed',duration_s=10,pwm=15,**kw)

    def test_ten_second_disconnect(self):
        self.sim.start(self.request())
        for _ in range(99):
            self.now += .1
            self.sim.tick()
        self.assertIsNotNone(self.sim.active)
        self.now = 110.001
        self.sim.tick()
        task = self.store.tasks()[0]
        self.assertEqual(task['reason'],'DURATION_COMPLETE')
        self.assertAlmostEqual(task['elapsed_s'],10.001)
        self.assertGreater(len(self.store.events(task['id'])),90)

    def test_idempotency_and_changed_payload(self):
        p=self.request()
        self.sim.start(p)
        self.sim.start(copy.deepcopy(p))
        self.assertEqual(len(self.store.tasks()),1)
        with self.assertRaises(Conflict):
            self.sim.start(p|{'pwm':12})
        self.sim.stop()
        self.sim.start(p)
        self.assertIsNone(self.sim.active)

    def test_single_controller(self):
        self.sim.start(self.request())
        with self.assertRaises(Conflict):
            self.sim.start(self.request()|{'id':'task-00000002','owner':'owner-0000002'})
        with self.assertRaises(ValueError):
            self.sim.action({'action':'input','id':'task-00000001','owner':'owner-0000002'})

    def test_remote_timeout(self):
        self.sim.start(self.request()|{'mode':'remote'})
        self.sim.tick()
        self.assertEqual(self.store.events('task-00000001')[-1]['pwm'],0)
        self.now+=.76
        self.sim.tick()
        self.assertEqual(self.store.tasks()[0]['reason'],'REMOTE_INPUT_TIMEOUT')

    def test_fault_estop_no_auto_resume(self):
        for action in ('fault','estop'):
            self.sim.start(self.request()|{'id':'task-00000'+action})
            self.sim.action({'action':action})
            self.assertIsNone(self.sim.active)
            self.sim.action({'action':'clear'})
            self.assertIsNone(self.sim.active)

    def test_vision_failures_and_fixed_independence(self):
        self.sim.start(self.request())
        self.sim.action({'action':'path'})
        self.assertIsNotNone(self.sim.active)
        self.sim.stop()
        for fault in ('path','camera'):
            self.sim.start(self.request()|{'id':'task-0000'+fault,'mode':'vision'})
            self.sim.action({'action':fault})
            self.assertIsNone(self.sim.active)
            self.assertIn('VISION_',self.sim.last_reason)

    def test_executor_restart(self):
        self.sim.start(self.request())
        other=Simulator(self.store,lambda:self.now)
        self.assertFalse(other.authorized)
        self.assertIsNone(other.active)
        self.assertEqual(self.store.tasks()[0]['reason'],'EXECUTOR_RESTART')
        self.assertEqual(other.applied['pwm_limit'],25)

    def test_save_does_not_authorize_or_start(self):
        self.sim.configure({'action':'authorize','enabled':False})
        self.store.save('config',0,{'pwm_limit':100,'authorized':True})
        self.assertFalse(self.sim.authorized)
        self.assertEqual(self.sim.applied['pwm_limit'],25)
        self.assertIsNone(self.sim.active)

    def test_draft_conflict_and_restart_persistence(self):
        self.store.save('test-A',0,{'wheelbase':None})
        with self.assertRaises(Conflict):
            self.store.save('test-A',0,{'wheelbase':0})
        other=Store(self.tmp.name)
        self.assertIsNone(other.get('test-A')['data']['wheelbase'])
        other.db.close()

    def test_limits_and_illegal_values(self):
        for value in (-1,101,None,True,float('nan'),float('inf'),'25'):
            with self.assertRaises(ValueError):
                self.sim.configure({'action':'apply','pwm_limit':value})
        for value in (-1,26,None,True,float('nan')):
            with self.assertRaises(ValueError):
                self.sim.start(self.request()|{'pwm':value})
        for value in (0,121,None,True,float('nan')):
            with self.assertRaises(ValueError):
                self.sim.start(self.request()|{'duration_s':value})
        self.sim.configure({'action':'apply','pwm_limit':100,'config_rev':2})
        self.sim.start(self.request()|{'pwm':100})
        with self.assertRaises(ValueError):
            self.sim.configure({'action':'apply','pwm_limit':0})
        with self.assertRaises(ValueError):
            self.sim.configure({'action':'authorize','enabled':False})

    def test_record_without_authority(self):
        self.sim.authorized=False
        self.sim.start(self.request()|{'mode':'record','pwm':0})
        self.assertEqual(self.sim.active['mode'],'record')

    def test_real_wall_clock_without_browser(self):
        sim=Simulator(self.store)
        sim.authorized=True
        sim.start(self.request()|{'duration_s':.2})
        time.sleep(.23)
        sim.tick()
        self.assertEqual(self.store.tasks()[0]['reason'],'DURATION_COMPLETE')


class DataTests(unittest.TestCase):
    def test_reject_invalid_documents_and_unsaved_configuration(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as tmp:
            app=App(tmp)
            for value in (-1,101,None):
                with self.assertRaises(ValueError):
                    app.post('/api/document',{'key':'config','rev':0,'data':{'pwm_limit':value}})
            with self.assertRaises(Conflict):
                app.post('/api/configure',{'action':'apply','config_rev':10,'pwm_limit':100})
            with self.assertRaises(ValueError):
                app.post('/api/document',{'key':'test-A','rev':0,'data':{'status':'已採用','values':{'width':1}}})
            uid=next(iter(app.replays))
            with self.assertRaises(ValueError):
                app.post('/api/document',{'key':'review-'+uid,'rev':0,'data':{'start':0,'end':999999}})
            app.store.db.close()

    def test_inventory_replay_and_invalidation(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as tmp:
            app=App(tmp)
            self.assertTrue(app.inventory['items'])
            shadow=next(r for r in app.replays.values() if r['kind']=='shadow')
            self.assertTrue(app.replay(shadow['id'])['rows'][0]['A'])
            field=next(r for r in app.replays.values() if r['kind']=='field')
            self.assertGreater(field['rows'][-1]['capture_s'],10)
            app.post('/api/document',{'key':'test-J','rev':0,'data':{'status':'待確認'}})
            app.post('/api/document',{'key':'config','rev':0,'data':{'changed':'camera'}})
            self.assertEqual(app.store.get('test-J')['data']['status'],'改裝後需重測')
            self.assertEqual(app.store.get('test-A')['rev'],0)
            app.store.db.close()


if __name__=='__main__':
    unittest.main(verbosity=2)
