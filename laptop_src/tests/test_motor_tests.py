import importlib.util,json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'web'))
from motor_tests import MotorTestManager,test_config,statistics_for

class Clock:
    def __init__(self):self.now=10.
    def __call__(self):return self.now
    def sleep(self,seconds):self.now+=seconds

class Board:
    def __init__(self,clock):
        self.clock=clock;self.commands=[];self.active=False;self.started=False;self.neutral=True;self.start=0;self.duration=0;self.generation=3;self.reason='PHYSICAL_START';self.sample=10000;self.right=0;self.left=0;self.af=self.ar=self.bf=self.br=0;self.manager=None;self.cancel_after=None
    def send(self,p):
        self.commands.append(dict(p))
        if p.get('action')=='stop':self.active=False;self.right=self.left=0;self.generation+=1;self.reason='REMOTE_STOP';return {'ok':True}
        action=p['motor_action']
        if action=='arm':self.active=True;self.neutral=True;self.started=False;self.duration=p['duration_ms']/1000
        if action=='move':
            if self.neutral:self.neutral=False
            elif not self.started:self.started=True;self.start=self.clock()
            self.right=p['right'];self.left=p['left']
        if action=='end':self.active=False;self.right=self.left=0;self.generation+=1;self.reason='MOTOR_TEST_COMPLETE'
        return {'ok':True}
    def state(self):
        now=self.clock();sample=int(now*5+1e-6)*200
        if sample>self.sample and self.active and self.started:
            dt=max(0,min(sample/1000,self.start+self.duration)-max(self.sample/1000,self.start))
            r=round(abs(self.right)*100*dt);l=round(abs(self.left)*200*dt)
            if self.right>=0:self.ar+=r
            else:self.af+=r
            if self.left>=0:self.bf+=l
            else:self.br+=l
        self.sample=sample
        if self.active and self.started and now-self.start>=self.duration:
            self.active=False;self.right=self.left=0;self.reason='MOTOR_TEST_COMPLETE';self.generation+=1
        if self.cancel_after and self.started and now-self.start>=self.cancel_after:self.manager.request_stop()
        state='MOTOR_TEST' if self.active else 'LOCKED' if self.reason=='REMOTE_STOP' else 'READY'
        v={'fw':'orin-v1.3.0-bench','state':state,'reason':self.reason,'generation':self.generation,'pins':{'D11':1},'servo_returning':False,'enc_sample_ms':sample,
           'motor_test':{'start_ms':round(self.start*1000),'started':self.started},
           'motor_a':{'pwm_command':abs(self.right)*255//100,'target':abs(self.right)*255//100},'motor_b':{'pwm_command':abs(self.left)*255//100,'target':abs(self.left)*255//100},
           'enc_a':{'forward':self.af,'reverse':self.ar,'invalid':0,'delta':0},'enc_b':{'forward':self.bf,'reverse':self.br,'invalid':0,'delta':0}}
        return {'vehicle':v,'uart_error':None,'uart_age_s':.05,'servo_settings_synced':True}

class RacingBoard(Board):
    # Firmware ends on its own (earlier) clock; status telemetry lags behind the ack.
    def __init__(self,clock,lag=.3,early=.2,end_reason='MOTOR_TEST_COMPLETE'):
        super().__init__(clock);self.lag=lag;self.early=early;self.end_reason=end_reason;self.done_at=None;self.reported=None
    def _tick(self):
        if self.active and self.started and self.clock()-self.start>=self.duration-self.early:
            self.active=False;self.right=self.left=0;self.reason=self.end_reason;self.generation+=1;self.done_at=self.clock()
    def send(self,p):
        self._tick()
        if p.get('action')!='stop':
            if p['generation']!=self.generation:self.commands.append(dict(p));raise ValueError('STALE_GENERATION')
            if p['motor_action']!='arm' and not self.active:self.commands.append(dict(p));raise ValueError('TEST_NOT_ARMED')
        return super().send(p)
    def state(self):
        self._tick();d=super().state()
        if self.done_at is not None and self.clock()-self.done_at<self.lag and self.reported:d['vehicle'].update(self.reported)
        elif self.active:self.reported=dict(state='MOTOR_TEST',reason='MOTOR_TEST_ARMED',generation=self.generation)
        return d

class MotorEndRace(unittest.TestCase):
    def run_race(self,**kw):
        clock=Clock();b=RacingBoard(clock,**kw)
        tmp=tempfile.TemporaryDirectory(dir=ROOT/'tests');self.addCleanup(tmp.cleanup)
        m=MotorTestManager(tmp.name,b.state,b.send,clock,clock.sleep);b.manager=m
        orig=m.sleep
        def sleep(t):orig(t);m.last_seen=m.clock()
        m.sleep=sleep
        m.start(dict(owner='a'*32,wheels_raised=True,wheel='right',direction='forward',right_pct=15,left_pct=0,seconds=1,warmup_s=0));m.thread.join(5)
        return m,b
    def test_move_rejected_after_firmware_completion_is_completed(self):
        m,b=self.run_race(lag=.6)
        self.assertTrue(any(c.get('motor_action')=='move' and c['generation']!=b.generation for c in b.commands))
        self.assertEqual((m.current['status'],m.current['reason']),('completed','DURATION_COMPLETE'))
        self.assertFalse(any(c.get('action')=='stop' for c in b.commands))
    def test_telemetry_lag_longer_than_wait_still_fails_and_stops(self):
        m,b=self.run_race(lag=2.0)
        self.assertEqual((m.current['status'],m.current['reason']),('failed','STALE_GENERATION'))
        self.assertTrue(any(c.get('action')=='stop' for c in b.commands))
    def test_other_end_reason_is_not_treated_as_completion(self):
        m,b=self.run_race(lag=0,end_reason='PCA_IO_FAILED')
        self.assertEqual(m.current['status'],'failed')
        self.assertTrue(any(c.get('action')=='stop' for c in b.commands))

class MotorTests(unittest.TestCase):
    def test_ground_ten_second_limit_and_completion(self):
        p=dict(self.payload(),surface='ground',area_clear=True,seconds=10,warmup_s=0,right_pct=20,left_pct=20)
        self.assertEqual(test_config(p)['duration_ms'],10000)
        for k,v in [('seconds',10.01),('right_pct',21),('left_pct',19),('direction','reverse'),('area_clear',False)]:
            with self.assertRaises(ValueError):test_config(dict(p,**{k:v}))
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            m,b=self.manager(tmp);state=b.state
            def current():
                d=state();d['vehicle']['web_timed_test_start']=True;d['camera_age_s']=.1;return d
            m.state=current;original_sleep=m.sleep
            def sleep(t):
                original_sleep(t);m.last_seen=m.clock()
            m.sleep=sleep
            result=m.start(p);m.thread.join(2)
            self.assertEqual(m.current['status'],'completed')
            self.assertEqual(b.commands[0]['duration_ms'],10000)
            self.assertEqual(b.right,0);self.assertEqual(b.left,0)
    def payload(self,**kwargs):return dict(owner='a'*32,wheels_raised=True,wheel='both',direction='forward',right_pct=10,left_pct=10,seconds=.9,warmup_s=.1,**kwargs)
    def manager(self,path):
        clock=Clock();board=Board(clock);m=MotorTestManager(path,board.state,board.send,clock,clock.sleep);board.manager=m;return m,board
    def test_fifty_percent_boundary(self):
        for direction,sign in [('forward',1),('reverse',-1)]:
            c=test_config(dict(self.payload(),right_pct=50,left_pct=50,direction=direction))
            self.assertEqual((c['right_command'],c['left_command']),(50*sign,50*sign))
        for value in [51,-1,19.8,True]:
            with self.assertRaises(ValueError):test_config(dict(self.payload(),left_pct=value))

    def test_parameters_and_single_wheel(self):
        p=self.payload();self.assertEqual(test_config(p)['right_command'],10)
        for key,value in [('right_pct',51),('left_pct',True),('seconds',float('nan')),('seconds',31),('warmup_s',1),('wheels_raised',False)]:
            with self.assertRaises(ValueError):test_config(dict(p,**{key:value}))
        c=test_config(dict(p,wheel='right',direction='reverse'));self.assertEqual((c['right_command'],c['left_command']),(-10,0))
    def test_completed_logs_signs_and_comparison(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            m,b=self.manager(tmp);result=m.start(self.payload());m.thread.join(2);self.assertFalse(m.thread.is_alive())
            run=m.result(result['run_id']);self.assertEqual(run['status'],'completed');self.assertEqual(run['reason'],'DURATION_COMPLETE')
            self.assertAlmostEqual(run['summary']['right_left_ratio'],.5,places=2)
            self.assertTrue(all(x['right_cps']>=0 and x['left_cps']>=0 for x in run['rows'] if x['right_cps'] is not None))
            self.assertIn(b'right_cps',m.download(run['id'],'csv'));self.assertIn(b'"type": "sample"',m.download(run['id'],'jsonl'))
            self.assertEqual(b.commands[1]['right'],0);self.assertEqual(b.commands[1]['left'],0)
            self.assertNotIn('owner',Path(tmp,run['id']+'.json').read_text())
    def test_reverse_counts_are_negative_and_completed_can_repeat(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            m,b=self.manager(tmp);p=self.payload();p['direction']='reverse'
            first=m.start(p);m.thread.join(2);self.assertEqual(m.current['status'],'completed')
            self.assertTrue(all(x['right_cps']<=0 and x['left_cps']<=0 for x in m.current['rows'] if x['right_cps'] is not None))
            second=m.start(self.payload());m.thread.join(2);self.assertEqual(m.current['status'],'completed');self.assertNotEqual(first['run_id'],second['run_id']);self.assertEqual(len(m.snapshot()['history']),2)

    def test_browser_lost_stops_and_records_failure(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            m,b=self.manager(tmp);p=self.payload();p['seconds']=2;m.start(p);m.thread.join(2)
            self.assertEqual(m.current['reason'],'BROWSER_HEARTBEAT_LOST');self.assertEqual(m.current['status'],'failed');self.assertEqual(b.commands[-1],{'action':'stop'})
    def test_manual_stop_interrupts(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            m,b=self.manager(tmp);b.cancel_after=.3;m.start(self.payload());m.thread.join(2)
            self.assertEqual(m.current['status'],'stopped');self.assertEqual(b.commands[-1],{'action':'stop'})
    def test_stop_during_coast_still_locks(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            m,b=self.manager(tmp);b.cancel_after=1.0;m.start(self.payload());m.thread.join(2)
            self.assertEqual(m.current['status'],'stopped');self.assertEqual(b.commands[-1],{'action':'stop'})

    def test_one_owner_and_recovery_never_restarts(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            m,b=self.manager(tmp)
            with patch('motor_tests.threading.Thread.start'):
                result=m.start(self.payload())
                with self.assertRaises(ValueError):m.start(self.payload())
                with self.assertRaises(ValueError):m.heartbeat({'run_id':result['run_id'],'owner':'b'*32})
                self.assertTrue(m.heartbeat({'run_id':result['run_id'],'owner':'a'*32})['ok'])
            other,_=self.manager(tmp);other.recover();self.assertIsNone(other.current);self.assertEqual(other.result(result['run_id'])['status'],'interrupted');self.assertEqual(b.commands,[])
    def test_download_path_and_insufficient_data(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            m,_=self.manager(tmp)
            with self.assertRaises(ValueError):m.download('../secret','csv')
        summary=statistics_for([],test_config(self.payload()));self.assertIsNone(summary['mean_right_cps']);self.assertEqual(summary['steady_samples'],0)

    def test_heartbeat_after_completion_is_not_a_new_authorization(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            m,b=self.manager(tmp);result=m.start(self.payload());m.thread.join(2)
            before=list(b.commands);seen=m.last_seen
            reply=m.heartbeat({'run_id':result['run_id'],'owner':'a'*32})
            self.assertFalse(reply['active']);self.assertEqual(reply['status'],'completed')
            self.assertEqual(m.last_seen,seen);self.assertEqual(b.commands,before)
            with self.assertRaises(ValueError):m.heartbeat({'run_id':result['run_id'],'owner':'b'*32})

    def test_late_heartbeat_cannot_revive_expired_lease(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            m,b=self.manager(tmp)
            with patch('motor_tests.threading.Thread.start'):
                result=m.start(self.payload());seen=m.last_seen
                m.clock.sleep(1.01)
                with self.assertRaisesRegex(ValueError,'BROWSER_HEARTBEAT_LOST'):
                    m.heartbeat({'run_id':result['run_id'],'owner':'a'*32})
                self.assertEqual(m.last_seen,seen);self.assertEqual(b.commands,[])
