import sys,json,tempfile,unittest,time
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'web'))
from manual_drive import ManualDriveManager

class ManualTests(unittest.TestCase):
    def make(self,folder):
        profile={'revision':2,'min_us':1010,'center_us':1460,'max_us':1810}
        now=[10.];commands=[];hooks=[None]
        vehicle={'fw':'orin-v1.4.1-manual','state':'READY','reason':'PHYSICAL_START','generation':3,'pins':{'D11':1},'servo_returning':False,'enc_sample_ms':10000,
                 'motor_a':{'pwm_command':0},'motor_b':{'pwm_command':0},'enc_a':{'delta':0},'enc_b':{'delta':0}}
        def state():
            if hooks[0]:hooks[0]()
            vehicle['enc_sample_ms']=int(now[0]*1000)
            return {'vehicle':vehicle,'uart_age_s':.01,'uart_error':None,'camera_age_s':.01,'camera_error':None,'servo_settings_synced':True}
        def send(p):
            commands.append(dict(p))
            if p.get('action')=='stop':vehicle['state']='LOCKED';vehicle['generation']+=1
            elif p.get('manual_action')=='arm':vehicle['state']='MANUAL'
            return {'ok':True}
        def center(c):profile.update(center_us=c,revision=profile['revision']+1)
        manager=ManualDriveManager(Path(folder)/'runs',state,send,lambda:dict(profile),center,Path(folder)/'defaults.json',clock=lambda:now[0])
        return manager,now,commands,hooks,vehicle,profile
    def test_fifty_percent_boundary(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,_,commands,_,_,profile=self.make(f)
            self.assertEqual(m._validate(dict(self.payload(),right_pct=50,left_pct=50),profile)['right_pct'],50)
            for value in [51,-1,19.8,True]:
                with self.assertRaises(ValueError):m._validate(dict(self.payload(),right_pct=value),profile)
            self.assertEqual(commands,[])
    def payload(self):return {'control_version':2,'owner':'a'*32,'area_clear':True,'record':False,'right_pct':10,'left_pct':15,'center_us':1460}
    def prepare(self,m):
        with patch('manual_drive.threading.Thread.start'):return m.start(self.payload())['run_id']
    def test_compact_snapshot_does_not_read_history_or_send_commands(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,now,commands,*_=self.make(f)
            with patch.object(Path,'glob',side_effect=AssertionError('history read')):
                result=m.snapshot(include_history=False)
            self.assertNotIn('history',result)
            self.assertEqual(result['input_lease_ms'],1000)
            self.assertEqual(commands,[])
    def test_missing_camera_and_wrong_firmware_block_start(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,*_=self.make(f);real=m.state
            m.state=lambda:dict(real(),camera_error='missing')
            with self.assertRaisesRegex(ValueError,'camera'):m.start(self.payload())
            m.state=real;real()['vehicle']['fw']='orin-v1.3.0-bench'
            with self.assertRaisesRegex(ValueError,'Flash'):m.start(self.payload())
    def test_owner_sequence_and_expiry(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,now,commands,*_=self.make(f);run=self.prepare(m)
            p={'owner':'a'*32,'run_id':run,'seq':1,'direction':1,'steer':0}
            with self.assertRaises(ValueError):m.input(dict(p,owner='b'*32))
            self.assertTrue(m.input(p)['ok'])
            with self.assertRaises(ValueError):m.input(p)
            seen=m.last_input;now[0]+=1.01
            with self.assertRaisesRegex(ValueError,'BROWSER_INPUT_LOST'):m.input(dict(p,seq=2))
            self.assertEqual(m.last_input,seen);self.assertEqual(commands,[])
    def test_independent_pwm_smooth_release_and_steering_hold_log(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,now,commands,hooks,*_=self.make(f);run=self.prepare(m);seq=[0]
            def refresh():
                t=now[0]-10
                if t>3.5:m.request_stop();return
                seq[0]+=1;m.input({'owner':'a'*32,'run_id':run,'seq':seq[0],'direction':1 if t<2 else 0,'steer':1 if t<.3 else 0})
            hooks[0]=refresh
            with patch('manual_drive.time.sleep',lambda dt:now.__setitem__(0,now[0]+dt)):m._run(m.current)
            moves=[p for p in commands if p.get('manual_action')=='move']
            self.assertEqual((moves[0]['right'],moves[0]['left']),(0,0))
            self.assertTrue(any(p['right']==10 and p['left']==15 for p in moves));self.assertTrue(any(p['us']>1460 for p in moves));self.assertGreater(moves[-1]['us'],1460,'Released steering must hold its command')
            self.assertTrue(all(p['right']==p['left']==0 for p in moves[-3:]));self.assertEqual(commands[-1],{'action':'stop'})
            self.assertTrue((m.root/run/'controls.jsonl').exists());self.assertEqual(m.current['status'],'stopped')
    def test_loss_stops_without_auto_resume(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,now,commands,*_=self.make(f);self.prepare(m)
            with patch('manual_drive.time.sleep',lambda dt:now.__setitem__(0,now[0]+dt)):m._run(m.current)
            self.assertEqual(m.current['reason'],'BROWSER_INPUT_LOST');self.assertEqual(commands[-1],{'action':'stop'})
            self.assertFalse(m.busy())
    def test_trim_save_and_center_only_persistence(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,now,commands,hooks,v,profile=self.make(f);run=self.prepare(m)
            p=dict(self.payload(),run_id=run,save=True,center_us=1470,right_pct=9)
            m.settings(p)
            self.assertEqual(profile['center_us'],1470);self.assertEqual(profile['min_us'],1010)
            saved=m.snapshot()['settings_history']
            self.assertEqual(len(saved),1)
            self.assertEqual(saved[0]['before']['center_us'],1460)
            self.assertEqual(saved[0]['config']['center_us'],1470)
            self.assertEqual(saved[0]['config']['right_pct'],9)
            self.assertEqual(saved[0]['profile']['center_us'],1470)
            self.assertEqual(json.loads(m.defaults_path.read_text()),{'right_pct':9,'left_pct':15,'steer_rate_us_s':300,'accel_seconds':2.0,'decel_seconds':1.0})
            m.intent=(1,0)
            with self.assertRaises(ValueError):m.settings(dict(p,center_us=1600))
            with self.assertRaises(ValueError):m.settings(dict(p,owner='b'*32))
            with self.assertRaises(ValueError):m.settings(dict(p,left_pct=51))
            other,*_=self.make(f);self.assertIsNone(other.current)
            self.assertEqual(other.snapshot()['history'][0]['status'],'interrupted')
            self.assertEqual(other.snapshot()['settings_history'][0]['id'],saved[0]['id'])

    def test_camera_loss_stops_motion(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,now,commands,*_=self.make(f);run=self.prepare(m);original=m.state;seq=[0]
            def state():
                seq[0]+=1;m.input({'owner':'a'*32,'run_id':run,'seq':seq[0],'direction':1,'steer':0})
                return dict(original(),camera_error='disconnected' if now[0]>10.4 else None)
            m.state=state
            with patch('manual_drive.time.sleep',lambda dt:now.__setitem__(0,now[0]+dt)):m._run(m.current)
            self.assertEqual(m.current['reason'],'CAMERA_LOST');self.assertEqual(commands[-1],{'action':'stop'})

    def test_direction_change_waits_before_reverse(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,now,commands,hooks,*_=self.make(f);run=self.prepare(m);seq=[0];stamped=[];original=m.send
            def send(p):stamped.append((now[0],dict(p)));return original(p)
            m.send=send
            def refresh():
                if now[0]>12:m.request_stop();return
                seq[0]+=1;m.input({'owner':'a'*32,'run_id':run,'seq':seq[0],'direction':1 if now[0]<10.5 else -1,'steer':0})
            hooks[0]=refresh
            with patch('manual_drive.time.sleep',lambda dt:now.__setitem__(0,now[0]+dt)):m._run(m.current)
            reverse=[t for t,p in stamped if p.get('right',0)<0]
            self.assertTrue(reverse);self.assertGreaterEqual(reverse[0]-10.5,.7)

    def test_passive_page_cannot_cancel_current_owner(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,*_=self.make(f);run=self.prepare(m)
            self.assertFalse(m.stop_for_page({'owner':'b'*32,'run_id':run},'WINDOW_BLUR')['active'])
            self.assertFalse(m.cancel.is_set())
            self.assertFalse(m.stop_for_page({'owner':'a'*32,'run_id':'0'*32},'PAGE_HIDDEN')['active'])
            self.assertFalse(m.cancel.is_set())
            self.assertTrue(m.stop_for_page({'owner':'a'*32,'run_id':run},'WINDOW_BLUR')['active'])
            self.assertTrue(m.cancel.is_set());self.assertEqual(m.reason,'WINDOW_BLUR')

    def test_control_area_exit_neutralizes_without_ending_session(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,*_=self.make(f);run=self.prepare(m)
            p={'owner':'a'*32,'run_id':run,'seq':1,'direction':1,'steer':1,'event':'KEY_DOWN'}
            m.input(p);m.input(dict(p,seq=2,direction=0,steer=0,event='CONTROL_AREA_EXIT'))
            self.assertEqual(m.intent,(0,0));self.assertTrue(m.busy());self.assertFalse(m.cancel.is_set())
            self.assertEqual(m.current['last_input_event']['event'],'CONTROL_AREA_EXIT')

    def test_steering_speed_persistence_validation_and_legacy_defaults(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,now,commands,hooks,v,profile=self.make(f)
            m.defaults_path.write_text(json.dumps({'right_pct':8,'left_pct':12}))
            legacy,*_=self.make(f)
            self.assertEqual(legacy.defaults['steer_rate_us_s'],300)
            self.assertIsNone(legacy.defaults_error)
            for bad in (0,49,801,True,300.5):
                with self.assertRaises(ValueError):m.settings(dict(self.payload(),steer_rate_us_s=bad))
            m.settings(dict(self.payload(),steer_rate_us_s=600,save=True))
            restored,*_=self.make(f)
            self.assertEqual(restored.defaults['steer_rate_us_s'],600)

    def test_steering_rate_and_return_respect_elapsed_time_and_limits(self):
        def run(rate):
            with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
                m,now,commands,hooks,v,profile=self.make(f);run_id=self.prepare(m);seq=[0]
                m.current['config']['steer_rate_us_s']=rate
                def refresh():
                    elapsed=now[0]-10
                    if elapsed>2.8:m.request_stop();return
                    seq[0]+=1;m.input({'owner':'a'*32,'run_id':run_id,'seq':seq[0],'direction':0,'steer':1 if elapsed<1 else 0,'center':elapsed>=1.3})
                hooks[0]=refresh
                with patch('manual_drive.time.sleep',lambda dt:now.__setitem__(0,now[0]+dt)):m._run(m.current)
                pulses=[p['us'] for p in commands if p.get('manual_action')=='move']
                self.assertTrue(all(profile['min_us']<=p<=profile['max_us'] for p in pulses))
                self.assertEqual(pulses[-1],profile['center_us'])
                self.assertTrue(all(p.get('right',0)==p.get('left',0)==0 for p in commands))
                return max(pulses)
        self.assertGreater(run(600),run(100))

    def test_smooth_release_space_and_explicit_center(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,now,commands,hooks,*_=self.make(f);run=self.prepare(m);seq=[0];samples=[];original=m.send
            def send(p):samples.append((round(now[0]-10,2),dict(p)));return original(p)
            m.send=send
            def refresh():
                t=now[0]-10
                if t>5:m.request_stop();return
                seq[0]+=1
                m.input({'owner':'a'*32,'run_id':run,'seq':seq[0],'direction':1 if t<2.1 or 3.3<t<4.2 else 0,'steer':1 if t<.4 else 0,'center':t>4.2,'hard_zero':4.2<=t<4.5})
            hooks[0]=refresh
            with patch('manual_drive.time.sleep',lambda dt:now.__setitem__(0,now[0]+dt)):m._run(m.current)
            moves=[(t,p) for t,p in samples if p.get('manual_action')=='move']
            self.assertTrue(any(2.2<t<2.9 and 0<p['right']<10 for t,p in moves),'Release must ramp down instead of cutting power')
            self.assertTrue(all(p['right']==p['left']==0 for t,p in moves if 4.3<t<4.5),'Space must bypass smooth deceleration')
            held=[p['us'] for t,p in moves if .6<t<3]
            self.assertEqual(len(set(held)),1,'No automatic recenter after releasing steering')
            self.assertEqual(moves[-1][1]['us'],1460,'Explicit C request recenters')
            self.assertTrue(all(type(p['right']) is int and type(p['left']) is int for t,p in moves))

    def test_time_settings_validate_and_persist(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,*_=self.make(f)
            for bad in (True,-1,5.1,float('nan'),float('inf'),'1'):
                for key in ('accel_seconds','decel_seconds'):
                    with self.assertRaises(ValueError):m.settings(dict(self.payload(),**{key:bad}))
            m.settings(dict(self.payload(),accel_seconds=3.2,decel_seconds=.8,save=True))
            restored,*_=self.make(f)
            self.assertEqual(restored.defaults['accel_seconds'],3.2);self.assertEqual(restored.defaults['decel_seconds'],.8)

    def test_zero_time_direct_pwm_and_release(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,now,commands,hooks,*_=self.make(f);run=self.prepare(m);seq=[0]
            m.settings(dict(self.payload(),run_id=run,accel_seconds=0,decel_seconds=0))
            def refresh():
                if now[0]>10.8:m.request_stop();return
                seq[0]+=1;m.input({'owner':'a'*32,'run_id':run,'seq':seq[0],'direction':1 if now[0]<10.4 else 0,'steer':0})
            hooks[0]=refresh
            with patch('manual_drive.time.sleep',lambda dt:now.__setitem__(0,now[0]+dt)):m._run(m.current)
            moves=[p for p in commands if p.get('manual_action')=='move']
            self.assertTrue(any(p['right']==10 and p['left']==15 for p in moves))
            self.assertTrue(all(p['right'] in (0,10) and p['left'] in (0,15) for p in moves))
            self.assertEqual(moves[-1]['right'],0)

    def test_invalid_flags_cannot_renew_input(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,*_=self.make(f);run=self.prepare(m)
            for flag in ('center','hard_zero'):
                with self.assertRaises(ValueError):m.input({'owner':'a'*32,'run_id':run,'seq':1,'direction':0,'steer':0,flag:1})
            self.assertEqual(m.input_seq,-1)

    def test_old_page_cannot_start_new_control_semantics(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
            m,*_=self.make(f);p=self.payload();p.pop('control_version')
            with self.assertRaisesRegex(ValueError,'refresh'):m.start(p)
            self.assertIsNone(m.current)
