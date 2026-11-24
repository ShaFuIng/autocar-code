import ast
import copy
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import patch
import ops07_runtime as rt
from ops07_route import entry_plan, seed_curve_map, seed_dash, seed_track, LapClock
from build_ops07 import ROOT, OUT, PACKAGES, build
from replay_ops07 import method
from ops07_client_candidate import Client


def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


PKG=ROOT/'data/ops07-packages/latest-2lap-97ed/web'
CurveMap=module(PKG/'curve_map.py','test_curve_map').CurveMap
# Execute the original route-state methods, without importing OpenCV/NumPy or
# any detector/device dependency. Image-path methods are outside this test.
source=ast.parse((PKG/'dash_route.py').read_text(encoding='utf8'))
nodes=[]
for node in source.body:
    if isinstance(node,ast.Assign):
        try:ast.literal_eval(node.value);nodes.append(node)
        except (ValueError,TypeError):pass
    elif isinstance(node,ast.ClassDef) and node.name=='DashRoute':
        node.body=[n for n in node.body if isinstance(n,ast.FunctionDef) and n.name in ('__init__','update_route')]
        nodes.append(node)
scope={};exec(compile(ast.Module(body=nodes,type_ignores=[]),'original_dash_route_state','exec'),scope)
DashRoute=scope['DashRoute']
TP=module(ROOT/'data/experiments/track-pos-20261108/candidate/track_pos.py','test_track').TrackPos
sys.modules['track_pos']=types.SimpleNamespace(TrackPos=TP)


class Boost:
    c={'flat_pitch_deg':0.,'max_age_s':1.}
    def _read(self):return 0.,0.


class Controller:
    def __init__(self,p):self.p=p;self.integral=0


class Manual:
    def __init__(self):
        self.lock=threading.RLock();self.kick=threading.Event();self.current=None;self.stops=[];self.on=False
        self.v=dict(generation=1,ms=1000,state='MANUAL',pins={'D11':1},pca_ok=True,steer_output_enabled=True,
                    steer_us_command=1390,enc_a={'delta':0},enc_b={'delta':0},
                    motor_a={'pwm_command':0,'target':0},motor_b={'pwm_command':0,'target':0})
    def state(self):return dict(vehicle=self.v,uart_age_s=0)
    def busy(self):return self.on
    def request_stop(self,r):self.stops.append(r);self.on=False


class Live:
    def __init__(self):
        self.lock=threading.RLock();self.operation_lock=threading.RLock();self.manual=Manual();self.run=None
        self.latest={'capture_at':0,'valid':True};self.consecutive=3;self.log=io.StringIO()
    def start(self,p):
        self.owner=p['owner']
        self.controller=Controller(p['parameters']);self.cmap=CurveMap();self.dash=DashRoute(Boost())
        self.run=dict(id='run1',state='running',execution_mode=p['execution_mode'],deadline=None,parameters=p['parameters'])
        self.manual.current=dict(id='run1',generation=1,execution_mode='orin_continuous',limits={'center_us':1390},
                                 command={'steer_us':1390,'left':0,'right':0})
        self.manual.on=True;return {'run_id':'run1'}
    def end(self,r):
        if self.run:self.run['state']='stopped'
        self.manual.request_stop(r);self.ops07.ended(r)
    def _pid_send(self,p):self.sent=p;return True


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(dir=OUT);self.addCleanup(self.temp.cleanup)
        self.now=0.;self.live=Live()
        self.package=dict(id='latest-2lap-97ed',identity='hash',parameters={'center_us':1390,'max_gap':.4},
                          parameter_revision=136,path_algorithm='test')
        self.r=rt.Runtime(self.live,self.package,self.temp.name,clock=lambda:self.now,flag_reader=lambda u:self.flag)
        self.live.ops07=self.r;self.r.flag_url='http://offline-fixture';self.flag=None
    def arm(self,traffic=False,location='flag'):
        return self.r.command('arm-hold',dict(location=location,traffic_light=traffic,owner='a'*32,area_clear=True))
    def request(self):return dict(token=self.r.token,session_id=self.r.session['id'])
    def fresh(self):self.live.latest['capture_at']=self.now
    def test_green_edge_token_and_no_browser(self):
        self.arm(True);self.r.tick();self.flag=1;self.r.poll_flag();self.assertTrue(self.r.holding)
        self.flag=0;self.r.poll_flag();self.flag=None;self.r.poll_flag();self.flag=1;self.r.poll_flag();self.assertTrue(self.r.holding)
        self.flag=0;self.r.poll_flag();self.flag=1;self.r.poll_flag();self.assertFalse(self.r.holding)
        self.now=36000;self.fresh();self.r.tick();self.assertEqual(self.r.session['state'],'running')
        self.assertEqual(self.r.clockwork.active_s,36000)
        self.r.release(self.request());self.assertEqual(len([e for e in self.r.session['events'] if e['kind']=='release']),1)
    def test_slow_flag_reply_after_pause_does_not_release(self):
        self.arm(True);self.r.tick();self.flag=0;self.r.poll_flag()
        def response(url):
            self.r.command('pause',self.request())
            return 1
        self.r.flag_reader=response;self.r.poll_flag()
        self.assertEqual(self.r.session['state'],'paused');self.assertTrue(self.r.holding)
    def test_pause_centers_preserves_origin_and_curve(self):
        self.arm(location='crest');self.r.tick();self.r.command('release',self.request())
        self.live.dash.opened=5;self.live.cmap.cur={'t0':3.,'yaw':70.}
        self.now=10;self.fresh();self.r.command('pause',self.request());self.assertTrue(self.live.manual.current['ops07_hold'])
        self.assertEqual(self.live.manual.current['pid_pwm'],0)
        self.now=110;self.fresh();self.r.command('resume',dict(session_id=self.r.session['id'],moved=False))
        self.r.tick();self.assertEqual(self.r.session['state'],'running')
        self.assertEqual(self.live.dash.opened,105)
        self.assertEqual(self.live.cmap.cur,{'t0':103.,'yaw':70.})
        self.assertEqual(self.r.session['origin'],'crest');self.assertEqual(self.r.clockwork.active_s,10)
    def test_no_release_before_mcu_center_ack(self):
        self.arm();self.live.manual.v['steer_us_command']=1800
        self.r.tick();self.assertEqual(self.r.session['state'],'centering')
        with self.assertRaises(ValueError):self.r.command('release',self.request())
        self.assertTrue(self.r.holding)
    def test_fault_does_not_center_or_release_and_old_token_rejected(self):
        self.arm(True);old=self.request();self.live.end('I2C_FAILED')
        self.assertEqual(self.r.session['center_status'],'unconfirmed')
        with self.assertRaises(ValueError):self.r.release(old)
        self.flag=1;self.r.poll_flag();self.assertTrue(self.r.holding)
    def test_mcu_clock_regression_stops_even_same_generation(self):
        self.arm();self.r.tick();self.live.manual.v['ms']=10;self.r.tick()
        self.assertIn('OPS07_MCU_CLOCK_REGRESSED',self.live.manual.stops)
    def test_restart_persists_position_no_output(self):
        self.arm(location='ramp');self.r.tick();self.r.command('release',self.request())
        self.now=10;self.r.tick()
        fresh=Live();recovered=rt.Runtime(fresh,self.package,self.temp.name,clock=lambda:0)
        self.assertEqual(recovered.session['origin'],'ramp');self.assertIsNone(recovered.token)
        self.assertTrue(recovered.session['needs_location']);self.assertEqual(recovered.clockwork.active_s,10)
        self.assertIsNone(fresh.run)
    def test_record_failure_stops(self):
        self.arm()
        with patch.object(rt,'atomic_json',side_effect=OSError('disk')):
            with self.assertRaises(OSError):self.r.persist()
        self.assertIn('OPS07_RECORDING_FAILED',self.live.manual.stops)
    def test_hold_freezes_route_but_updates_fresh_frames(self):
        self.arm(location='crest');self.live.dash.opened=3
        self.now=200
        self.assertTrue(rt.holding_frame(self.live,dict(valid=True,path=[[0,0],[1,1]]),200,5,5,b'jpeg'))
        self.assertEqual(self.live.dash.opened,3);self.assertEqual(self.live.sent['pwm'],0)
        self.assertEqual(self.live.preview,b'jpeg')

    def test_direct_start_runs_on_orin_after_center_without_browser(self):
        self.r.command('arm-hold',dict(location='flag',traffic_light=False,owner='a'*32,
                                     area_clear=True,start_when_ready=True))
        self.live.manual.v['steer_us_command']=1600
        self.r.tick();self.assertTrue(self.r.holding)
        self.live.manual.v['steer_us_command']=1390
        self.r.tick();self.assertEqual(self.r.session['state'],'running')

    def test_idle_restart_stays_idle_without_hardware(self):
        self.r.session['revision']='previous-release';self.r.persist()
        recovered=rt.Runtime(Live(),self.package,self.temp.name,clock=lambda:0)
        self.assertEqual(recovered.session['state'],'idle')
        self.assertFalse(recovered.session['needs_location'])
        self.assertEqual(recovered.session['revision'],rt.REVISION)

    def test_first_model_load_without_preview_fields(self):
        self.live.worker=None;self.live.motor_busy=lambda:False
        self.r.package.update(recognition={'model_id':'v4-default'})
        self.r.package['parameters']['row_ratio']=.6
        calls=[]
        self.live.prepare=lambda p:(calls.append(p) or {'ok':True,'hardware_output':False})
        result=self.r.command('load',{})
        self.assertTrue(result['ok']);self.assertEqual(len(calls),1)
        self.assertIsNone(self.live.run);self.assertFalse(self.live.manual.on)

    def test_matching_preview_reused_without_reload(self):
        self.live.worker=types.SimpleNamespace(is_alive=lambda:True)
        self.live.motor_busy=lambda:False;self.live.model_id='v4-default'
        self.live.path_alias='test';self.live.row_ratio=.6
        self.r.package.update(recognition={'model_id':'v4-default'})
        self.r.package['parameters']['row_ratio']=.6
        self.assertTrue(self.r.command('load',{})['reused'])


class RouteTests(unittest.TestCase):
    def test_client_no_retry_after_unknown_ack(self):
        calls=[]
        def request(path,p=None):
            calls.append((path,p))
            if p is not None:raise TimeoutError('lost ACK')
            return dict(revision=rt.REVISION,package_identity='hash',id='session',token='token')
        c=Client(request,dict(identity='hash'));c.refresh()
        with self.assertRaises(TimeoutError):c.command('pause')
        with self.assertRaises(ValueError):c.command('pause')
        self.assertEqual(len(calls),2)
        c.refresh();self.assertFalse(c.unknown)
    def test_client_rejects_wrong_package_and_mutable_parameters(self):
        c=Client(lambda p:dict(revision=rt.REVISION,package_identity='other'),dict(identity='hash'))
        with self.assertRaises(ValueError):c.refresh()
        c.state={}
        with self.assertRaises(ValueError):c.command('arm-hold',parameters={})
    def test_generated_output_guard_and_legacy_modes(self):
        for pkg in PACKAGES:
            path=OUT/'candidates'/pkg/'web/live_pid.py'
            send=method(path,'_pid_send',{'ops07_support':rt,'time':types.SimpleNamespace(monotonic=lambda:0)})
            for mode in ('browser_lease','orin_timed','orin_continuous'):
                calls=[];live=types.SimpleNamespace(run=dict(execution_mode=mode,parameters={'center_us':1390}),
                    ops07=types.SimpleNamespace(holding=True),manual=types.SimpleNamespace(pid_input=calls.append))
                self.assertTrue(send(live,dict(pwm=15,pwm_right=16,pwm_left=14,target_us=1800)))
                self.assertEqual(calls[0]['pwm'],0 if mode=='orin_continuous' else 15)
                self.assertEqual(calls[0]['target_us'],1390 if mode=='orin_continuous' else 1800)
                if mode=='orin_continuous':self.assertNotIn('pwm_right',calls[0])
    def test_generated_manual_deadline_keeps_uart_protection(self):
        check=method(OUT/'candidates'/PACKAGES[-1]/'web/manual_drive.py','_check',{'deadline_expired':rt.deadline_expired})
        obj=types.SimpleNamespace(cancel=threading.Event(),lock=threading.RLock(),clock=lambda:20,last_input=20,
            current=dict(controller='visual_pid',execution_mode='orin_continuous',deadline=None),
            state=lambda:dict(uart_age_s=0,camera_age_s=0))
        check(obj)
        obj.current.update(execution_mode='orin_timed',deadline=10)
        with self.assertRaisesRegex(ValueError,'PID_DURATION_COMPLETE'):check(obj)
        obj.current.update(execution_mode='orin_continuous',deadline=None);obj.state=lambda:dict(uart_age_s=10)
        with self.assertRaisesRegex(ValueError,'UART_LOST'):check(obj)
    def test_seed_additions_leave_original_methods_identical(self):
        for pkg in PACKAGES:
            for name in ('curve_map.py','dash_route.py'):
                original=ast.parse((ROOT/'data/ops07-packages'/pkg/'web'/name).read_text(encoding='utf8'))
                candidate=ast.parse((OUT/'candidates'/pkg/'web'/name).read_text(encoding='utf8'))
                for n in ast.walk(candidate):
                    if isinstance(n,ast.ClassDef):n.body=[m for m in n.body if not (isinstance(m,ast.FunctionDef) and m.name=='seed')]
                self.assertEqual(ast.dump(original),ast.dump(candidate))
    def test_all_package_entries_next_curve(self):
        for pkg in PACKAGES:
            cfg=json.loads((ROOT/'data/ops07-packages'/pkg/'web/curve_map.json').read_text())
            for phase in ('first','after_down'):
                for entry in ('flag','12_to_01','02_to_03_before','02_to_03_after','04_to_05_before','04_to_05_after','05_to_06','ramp','crest'):
                    plan=entry_plan(pkg,entry,phase,x_m=1.)
                    cm=CurveMap(cfg);seed_curve_map(cm,plan)
                    _,info=cm.step(0,-30,'WAIT_CLIMB',True,speed=0)
                    self.assertEqual(info['idx'],plan['idx'])
                    expected=(cfg['map_start'] if phase=='first' else cfg['map_after_down'])
                    self.assertEqual(info['expected_deg'],expected[plan['idx']] if plan['idx']<len(expected) else None)
                    if plan['landmark']:self.assertIn('lm',cm.odo)
                    d=DashRoute(Boost());seed_dash(d,plan);self.assertEqual(d.state,plan['dash_state'])
                    tp=TP();seed_track(tp,plan);self.assertEqual(tp.s_at(0),plan['s_m'])
    def test_unmeasured_entry_and_mid_curve_rejected(self):
        for entry in ('02_to_03_before','04_to_05_before','curve_middle'):
            with self.assertRaises(ValueError):entry_plan(PACKAGES[-1],entry)
    def test_flag_anchor_and_jump_filter(self):
        c=LapClock('ramp');c.tick(8,True)
        for s,n,t in [(0,'DOWNHILL_START',1),(4.45,'CURVE_START_curve4',2),(16.5,'CURVE_START_curve4',2),(17.7,'CURVE_START_curve4',2)]:
            result=c.observe(dict(s_m=s,anchor={'name':n,'t':t}))
        self.assertFalse(result['valid']);self.assertEqual(len(c.laps),1)
        c.observe(dict(s_m=17.8));self.assertEqual(len(c.laps),1)
    def test_legacy_deadlines_unchanged(self):
        for mode in ('browser_lease','orin_timed'):
            self.assertFalse(rt.deadline_expired(dict(execution_mode=mode,deadline=5),4))
            self.assertTrue(rt.deadline_expired(dict(execution_mode=mode,deadline=5),5))
        self.assertFalse(rt.deadline_expired(dict(execution_mode='orin_continuous',deadline=None),1e9))


if __name__=='__main__':unittest.main()
