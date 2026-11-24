import json,sys,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
from orin_bridge import Bridge,ROOT

class RemoteTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir=ROOT/'data/test_tmp')
        self.module=types.ModuleType('remote_test');self.module.__file__=str(ROOT/'orin_remote.py')
        fake=types.SimpleNamespace(flock=lambda *a:None,LOCK_EX=1)
        with patch.dict(sys.modules,{'fcntl':fake}),patch.object(Path,'home',return_value=Path(self.tmp.name)):
            exec(compile((ROOT/'orin_remote.py').read_text(encoding='utf-8'),'orin_remote.py','exec'),self.module.__dict__)
        self.m=self.module;self.job='a'*32;self.payload={'job':self.job,'duration_s':1}
        self.m.save(self.m.jid(self.job),dict(job=self.job,state='queued'))
    def tearDown(self):self.tmp.cleanup()
    def test_existing_recording_never_stopped(self):
        with patch.object(self.m,'current',return_value={'id':'existing','status':'recording'}),patch.object(self.m,'request') as request:
            self.m.worker(self.payload);request.assert_not_called()
        self.assertEqual(json.loads(self.m.jid(self.job).read_text())['state'],'failed')
    def test_owned_recording_stops_at_remote_deadline(self):
        vehicle={'state':'LOCKED','motor_a':{'pwm_command':0,'target':0},'motor_b':{'pwm_command':0,'target':0}}
        with patch.object(self.m,'current',side_effect=[{},dict(id='owned',status='recording')]),patch.object(self.m,'get',return_value={'vehicle':vehicle}),patch.object(self.m.time,'monotonic',side_effect=[0,2]),patch.object(self.m,'request',side_effect=[b'{"current":{"id":"owned"}}',b'{"current":{"id":"owned","status":"stopped"}}']) as request:
            self.m.worker(self.payload)
            self.assertEqual([x.args[0] for x in request.call_args_list],['/api/record/start','/api/record/stop'])
        self.assertEqual(json.loads(self.m.jid(self.job).read_text())['state'],'completed')
    def test_replaced_recording_not_stopped(self):
        with patch.object(self.m,'current',side_effect=[{},dict(id='someone_else',status='recording')]),patch.object(self.m,'get',return_value={'vehicle':{'state':'LOCKED'}}),patch.object(self.m.time,'monotonic',side_effect=[0,2]),patch.object(self.m,'request',return_value=b'{"current":{"id":"owned"}}') as request:
            self.m.worker(self.payload);self.assertEqual(request.call_count,1)
        self.assertEqual(json.loads(self.m.jid(self.job).read_text())['state'],'ownership_changed')
    def test_motion_and_path_traversal_rejected(self):
        with self.assertRaises(ValueError):self.m.request('/api/motor/start',True)
        with self.assertRaises(ValueError):self.m.folder({'kind':'record','id':'../token'})
        with self.assertRaises(ValueError):Bridge(self.tmp.name).call('motor')

if __name__=='__main__':unittest.main()
