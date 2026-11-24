import sys,tempfile,threading,time,unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'web'))
from live_pid import LivePID
from visual_pid import DEFAULTS
from test_manual_drive import ManualTests

class LiveTests(unittest.TestCase):
    def test_preview_only_path_cannot_start(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            live,m,now,cmd,p=self.make(f)
            live.path_algorithm='temporal-v2';p['path_algorithm']='temporal-v2'
            with self.assertRaisesRegex(ValueError,'不可啟動實車'):live.start(p)
            self.assertEqual(cmd,[])
            self.assertIsNone(live.run)

    def test_path_mismatch_cannot_start(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            live,m,now,cmd,p=self.make(f);p['path_algorithm']='single-left'
            with self.assertRaisesRegex(ValueError,'預覽不符'):live.start(p)
            self.assertEqual(cmd,[])

    def test_unknown_path_prepare_rejected(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            live,m,now,cmd,p=self.make(f)
            with self.assertRaisesRegex(ValueError,'Unknown path'):
                live.prepare({'path_algorithm':'arbitrary-module'})
            self.assertEqual(cmd,[])

    def test_stop_diagnostic_preserves_invalid_result(self):
        import json
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            live,m,now,cmd,p=self.make(f)
            with patch('manual_drive.threading.Thread.start'):live.start(p)
            live.latest=dict(capture_at=time.monotonic(),valid=False,path=[],quality=0,frame_seq=99)
            live.preview=b'test-preview'
            live.end('PID_INVALID_OR_STALE_PATH')
            rows=[json.loads(line) for line in (m.root/live.run['id']/'pid.jsonl').read_text().splitlines()]
            d=next(r for r in rows if r['kind']=='stop_diagnostic')
            self.assertEqual(d['vision']['frame_seq'],99)
            self.assertFalse(d['vision']['valid'])
            self.assertTrue(m.cancel.is_set())
            self.assertEqual(rows[-1]['reason'],'PID_INVALID_OR_STALE_PATH')
    def make(self,folder):
        m,now,commands,hooks,v,profile=ManualTests().make(folder)
        with patch('live_pid.threading.Thread.start'):
            live=LivePID(m,lambda:(b'',0,0),threading.RLock(),lambda:False)
        live.state='ready';live.consecutive=3;live.row_ratio=.75;live.model_id='v4-default'
        live.latest={'capture_at':time.monotonic(),'weights_sha256':'test'}
        p=dict(DEFAULTS,center_us=1460,min_us=1010,max_us=1810)
        request=dict(parameters=p,owner='a'*32,seconds=3,area_clear=True,parameter_revision=1)
        return live,m,now,commands,request
    def test_start_is_frozen_and_manual_input_rejected(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            live,m,now,cmd,p=self.make(f)
            with patch('manual_drive.threading.Thread.start'):r=live.start(p)
            self.assertEqual(m.current['controller'],'visual_pid');self.assertEqual(cmd,[])
            with self.assertRaises(ValueError):m.input(dict(owner=p['owner'],run_id=r['run_id'],seq=1,direction=1,steer=0))
            with self.assertRaises(ValueError):live.start(p)
            live.end()
    def test_twenty_seconds_accepted_and_deadline_stops(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            live,m,now,cmd,p=self.make(f);p.update(seconds=20,execution_mode='orin_timed')
            with patch('manual_drive.threading.Thread.start'):live.start(p)
            self.assertEqual(live.run['seconds'],20);self.assertEqual(cmd,[])
            live.run['deadline']=0;live.check_safety()
            self.assertEqual(live.run['reason'],'PID_DURATION_COMPLETE')

    def test_watchdogs(self):
        for failure in ('browser','vision','duration'):
            with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
                live,m,now,cmd,p=self.make(f)
                with patch('manual_drive.threading.Thread.start'):live.start(p)
                if failure=='browser':live.browser_at-=2
                if failure=='vision':live.latest['capture_at']-=2
                if failure=='duration':live.run['deadline']=0
                live.check_safety();self.assertTrue(m.cancel.is_set());self.assertEqual(live.run['state'],'stopped')
                with self.assertRaises(ValueError):live.heartbeat(dict(owner=p['owner'],run_id=live.run['id'],seq=1))
    def test_preconditions(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            live,m,now,cmd,p=self.make(f)
            live.consecutive=0
            with self.assertRaises(ValueError):live.start(p)
            live.consecutive=3;p['parameters']['center_us']=1450
            with self.assertRaises(ValueError):live.start(p)
            self.assertEqual(cmd,[])
    def test_timed_task_ignores_browser_loss_but_keeps_deadline(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            live,m,now,cmd,p=self.make(f);p['execution_mode']='orin_timed'
            with patch('manual_drive.threading.Thread.start'):result=live.start(p)
            deadline=live.run['deadline'];live.browser_at-=100
            live.check_safety()
            self.assertEqual(live.run['state'],'running');self.assertFalse(m.cancel.is_set())
            self.assertEqual(result['execution_mode'],'orin_timed')
            # Observation/heartbeat cannot extend the task or change its parameters.
            live.heartbeat(dict(owner=p['owner'],run_id=live.run['id'],seq=1))
            live.status();self.assertEqual(live.run['deadline'],deadline)
            with patch('live_pid.time.monotonic',return_value=deadline+.01):live.check_safety()
            self.assertEqual(live.run['reason'],'PID_DURATION_COMPLETE')
            self.assertTrue(m.cancel.is_set());self.assertEqual(cmd,[])
    def test_timed_task_retains_vision_and_local_pid_watchdogs(self):
        for failure in ('vision','local_input','controller','explicit_stop'):
            with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
                live,m,now,cmd,p=self.make(f);p['execution_mode']='orin_timed'
                with patch('manual_drive.threading.Thread.start'):live.start(p)
                live.browser_at-=100
                if failure=='vision':
                    live.latest['capture_at']-=2;live.check_safety()
                    self.assertEqual(live.run['reason'],'PID_VISION_STALE')
                elif failure=='local_input':
                    now[0]+=1.1
                    with self.assertRaises(ValueError):m._check()
                    live.end('TEST_LOCAL_INPUT_STOP')
                elif failure=='controller':
                    m.current['status']='stopped';live.check_safety()
                    self.assertEqual(live.run['reason'],'VEHICLE_CONTROLLER_STOPPED')
                else:
                    live.end();self.assertEqual(live.run['reason'],'USER_STOP')
                self.assertEqual(live.run['state'],'stopped');self.assertEqual(cmd,[])
    def test_mode_and_duration_reject_invalid_without_actuation(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            live,m,now,cmd,p=self.make(f)
            for changes in ({'execution_mode':'forever'},{'execution_mode':'orin_timed','seconds':21}):
                with self.assertRaises(ValueError):live.start(dict(p,**changes))
            self.assertEqual(cmd,[])
    def test_target_range_sequence_and_duration(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            live,m,now,cmd,p=self.make(f)
            with patch('manual_drive.threading.Thread.start'):r=live.start(p)
            target=dict(owner=p['owner'],run_id=r['run_id'],seq=1,target_us=1500,pwm=10)
            m.pid_input(target)
            with self.assertRaises(ValueError):m.pid_input(target)
            with self.assertRaises(ValueError):m.pid_input(dict(target,seq=2,pwm=26))
            with self.assertRaises(ValueError):m.pid_input(dict(target,seq=2,target_us=2500))
            now[0]+=4
            with self.assertRaises(ValueError):m._check()
            live.end()
    def test_fake_actuator_run_starts_zero_and_stops(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            helper=ManualTests();m,now,commands,hooks,v,profile=helper.make(f)
            p=dict(helper.payload(),seconds=1,accel_seconds=0,decel_seconds=0,right_pct=10,left_pct=10)
            with patch('manual_drive.threading.Thread.start'):r=m.start(p,_pid=True)
            seq=[0]
            def feed():
                seq[0]+=1
                m.pid_input(dict(owner=p['owner'],run_id=r['run_id'],seq=seq[0],target_us=1500,pwm=10))
            hooks[0]=feed
            with patch('manual_drive.time.sleep',lambda dt:now.__setitem__(0,now[0]+dt)):m._run(m.current)
            moves=[c for c in commands if c.get('manual_action')=='move']
            self.assertEqual((moves[0]['right'],moves[0]['left']),(0,0))
            self.assertTrue(any(c['right']==10 and c['left']==10 for c in moves))
            self.assertEqual(commands[-1],{'action':'stop'})
            self.assertEqual(m.current['reason'],'PID_DURATION_COMPLETE')

if __name__=='__main__':unittest.main()
