"""Offline integration checks. All vehicle requests are replaced by a recorder."""
import ast
import base64
import json
import re
import shutil
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from ops07 import Operations, PACKAGE_IDS, ROOT, inspect_package, observation
from server import Conflict, Store, handler, WorkbenchHTTPServer


class Transport:
    def __init__(self):
        self.calls = []
        self.pid = dict(state='idle', run=None)
        self.manual = dict(vehicle=dict(uart_age_s=.1, vehicle=dict(state='LOCKED', pca_ok=True, i2c_errors=7, pins={'D11':1})))
        self.extra = dict(position=None)
        self.fail = False
        self.gate = None
        self.entered = threading.Event()

    def request(self, path, payload=None):
        self.calls.append((path, payload))
        self.entered.set()
        if self.gate:
            self.gate.wait(3)
        if self.fail:
            raise ValueError('ACK unknown')
        if path == '/api/pid/status':
            return self.pid
        if path.startswith('/api/manual/status'):
            return self.manual
        if path.startswith('/ops07/observation/'):
            return self.extra
        if path == '/api/pid/stop':
            return {'ok': True}
        raise AssertionError('Unexpected endpoint: '+path)

    def stop_all(self):
        self.calls.append(('stop_all', None))
        return {'confirmed': not self.fail}


class OpsTests(unittest.TestCase):
    def setUp(self):
        self.test_root = ROOT / 'data/experiments/ops07-ui-20261108/test-tmp'
        self.test_root.mkdir(parents=True,exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(dir=self.test_root)
        self.store = Store(self.tmp.name)
        self.transport = Transport()
        self.ops = Operations(SimpleNamespace(store=self.store, integration=self.transport))

    def tearDown(self):
        self.store.db.close()
        assert Path(self.tmp.name).resolve().is_relative_to(self.test_root.resolve())
        self.tmp.cleanup()

    def payload(self, **changes):
        return dict(dict(rev=0, package_id=PACKAGE_IDS[0], traffic_light=True, front_car=False,
                         location='02_to_03_after', moved=True), **changes)

    def test_three_real_packages_and_fixed_slow_adjustment(self):
        catalog = self.ops.catalog()
        self.assertEqual([x['file_count'] for x in catalog['packages']], [103,106,112])
        self.assertTrue(all(p['verified'] for p in catalog['packages']))
        self.assertFalse(any(catalog['capabilities'].values()))
        self.assertEqual(catalog['packages'][0]['parameters']['steer_rate'],1000)
        self.assertEqual(catalog['packages'][0]['parameters']['min_us'],780)
        self.assertEqual(self.transport.calls, [])

    def test_load_persist_refresh_and_reset_preserve_history_and_pid(self):
        self.store.save('visual-pid',0,{'user_value':123})
        saved = self.ops.post('load', self.payload())
        restored = Operations(self.ops.app).catalog()['selection']
        self.assertEqual(saved, restored)
        self.assertEqual(restored['data']['location'],'02_to_03_after')
        self.assertFalse(restored['data']['applied_to_vehicle'])
        self.ops.post('reset',{'rev':1})
        self.assertEqual(self.store.get('ops07-selection')['data'],{})
        self.assertEqual(self.store.get('visual-pid')['data'],{'user_value':123})
        self.assertEqual(self.store.db.execute("SELECT count(*) FROM history WHERE key='ops07-selection'").fetchone()[0],2)
        self.assertEqual(self.transport.calls,[])

    def test_conflicting_tabs_cannot_overwrite_loaded_selection(self):
        self.ops.post('load',self.payload())
        with self.assertRaises(Conflict):self.ops.post('load',self.payload())

    def test_no_unsupported_start_or_arbitrary_parameters(self):
        for action in ('start','resume'):
            with self.assertRaises(ValueError):self.ops.post(action,{'seconds':999999})
        for changes in ({'front_car':True},{'location':'curve_middle'},{'pwm':50},{'traffic_light':'true'},{'package_id':'../secret'}):
            with self.assertRaises(ValueError):self.ops.post('load',self.payload(**changes))
        self.assertEqual(self.transport.calls,[])

    def test_package_tampering_rejected_on_every_load(self):
        base=Path(self.tmp.name)/'packages';name=PACKAGE_IDS[0]
        shutil.copytree(ROOT/'data/ops07-packages'/name,base/name)
        with (base/name/'web/live_pid.py').open('a') as f:f.write('\n# changed\n')
        with self.assertRaisesRegex(ValueError,'已改變'):inspect_package(base,name)

    def test_i2c_counter_is_not_active_fault_and_stale_hardware_not_current(self):
        result=observation(self.transport.pid,self.transport.manual)
        self.assertNotIn('I2C',[i['code'] for i in result['issues']])
        self.transport.manual['vehicle']['vehicle'].update(pca_ok=False,state='FAULT',reason='I2C_FAILURE')
        self.assertIn('I2C',[i['code'] for i in observation(self.transport.pid,self.transport.manual)['issues']])
        self.transport.manual['vehicle']['uart_age_s']=20
        self.assertNotIn('I2C',[i['code'] for i in observation(self.transport.pid,self.transport.manual)['issues']])

    def test_physical_estop_is_clear_and_no_false_lap_timer(self):
        self.transport.manual['vehicle']['vehicle']['pins']['D11']=0
        result=observation({'latest':{'track_pos':{'lap':12}},'run':{'state':'stopped','reason':'PHYSICAL_ESTOP'}},self.transport.manual)
        self.assertIn('ESTOP',[i['code'] for i in result['issues']])
        self.assertIsNone(result['timing']);self.assertIsNone(result['position'])

    def test_no_reliable_path_has_explicit_operator_remedy(self):
        result=observation(dict(state='no_reliable_path',latest={'valid':False}),self.transport.manual)
        self.assertIn('VISION',[i['code'] for i in result['issues']])

    def test_status_poll_is_single_flight_read_only(self):
        self.transport.gate=threading.Event()
        self.ops.status();self.assertTrue(self.transport.entered.wait(2))
        self.ops.status();self.assertEqual(len(self.transport.calls),1)
        self.transport.gate.set()
        deadline=time.monotonic()+2
        while self.ops.polling and time.monotonic()<deadline:time.sleep(.005)
        self.assertFalse(self.ops.polling)
        self.assertTrue(all(payload is None for _,payload in self.transport.calls))
        self.assertTrue(self.ops.status()['connected'])

    def test_log_position_keeps_provenance_and_disconnect_last_observation(self):
        uid='a'*32
        self.transport.pid.update(run={'id':uid,'state':'running'})
        self.transport.extra=dict(run_id=uid,position={'s_m':31,'segment':'straight_to_curve2','lap':3},sample_age_s=.2)
        self.ops._poll()
        self.assertTrue(self.ops.live['position']['current'])
        self.assertIsNone(self.ops.live['timing'])
        self.assertEqual(self.store.get('ops07-last-observation')['data']['run_id'],uid)
        self.transport.fail=True;self.ops._poll()
        self.assertFalse(self.ops.status()['connected'])
        self.assertEqual(self.ops.live['position']['s_m'],31)

    def test_stop_ack_never_claims_physical_stop_or_retries(self):
        self.transport.fail=True
        result=self.ops.post('stop',{'intent':'stop'})
        self.assertFalse(result['acknowledged']);self.assertFalse(result['stopped_confirmed'])
        self.assertEqual(self.transport.calls.count(('/api/pid/stop',{})),1)
        self.assertEqual(self.transport.calls.count(('stop_all',None)),1)
        self.transport.fail=False
        result=self.ops.post('pause',{'intent':'pause'})
        self.assertTrue(result['acknowledged']);self.assertFalse(result['stopped_confirmed'])
        self.assertEqual(self.store.get('ops07-last-command')['rev'],2)

    def test_frame_offline_does_not_fetch_or_return_cached_picture(self):
        self.ops.frame_bytes=b'old';self.ops.frame_at=time.monotonic()
        self.assertEqual(self.ops.frame(),(b'',503))
        self.assertEqual(self.transport.calls,[])

    def test_real_start_is_explicit_fixed_and_one_request(self):
        from ops07_runtime import REVISION
        saved=self.ops.save_selection(self.payload(package_id='best-today-46db',traffic_light=False,moved=False,location='flag'))
        package=inspect_package(self.ops.base,'best-today-46db')
        state=dict(revision=REVISION,package_identity=package['identity'],state='idle',id=None,token=None)
        calls=[]
        def request(path,p=None):
            calls.append((path,p))
            return state
        self.transport.request=request
        self.ops.post('start',dict(intent='start',area_clear=True,rev=saved['rev']))
        self.assertEqual([x[0] for x in calls],['/api/ops07/status','/api/ops07/arm-hold'])
        self.assertNotIn('parameters',calls[-1][1]);self.assertNotIn('seconds',calls[-1][1])
        self.assertTrue(calls[-1][1]['start_when_ready'])
        calls.clear()
        with self.assertRaises(ValueError):self.ops.post('start',dict(intent='start',area_clear=False,rev=saved['rev']))
        self.assertFalse(calls)

    def test_wrong_package_load_leaves_original_selection(self):
        saved=self.ops.save_selection(self.payload(package_id='best-today-46db'))
        self.ops.session={'package_identity':saved['data']['identity']}
        with self.assertRaises(ValueError):self.ops.post('load',self.payload(rev=saved['rev']))
        self.assertEqual(self.store.get('ops07-selection'),saved)
        self.assertEqual(self.transport.calls,[])

    def test_assets_and_cross_origin_write_rejection(self):
        sent=[];app=SimpleNamespace(ops07=self.ops)
        for path,mime in [('/ops07.js','javascript'),('/ops07.css','css'),('/ops07-track.webp','webp')]:
            fake=SimpleNamespace(path=path,headers={},send=lambda body,status=200,mime=None:sent.append((status,mime,len(body))))
            handler(app).do_GET(fake)
            self.assertEqual(sent[-1][0],200);self.assertIn(mime,sent[-1][1]);self.assertGreater(sent[-1][2],100)
        fake=SimpleNamespace(path='/api/ops07/load',headers={'Origin':'http://evil.example','Host':'127.0.0.1:8093','X-Workbench':'1'},send=lambda body,status=200:sent.append((status,body)))
        handler(app).do_POST(fake)
        self.assertEqual(sent[-1][0],403)
        self.assertFalse(WorkbenchHTTPServer.allow_reuse_address)

    def test_bounded_remote_log_reader_and_path_guard(self):
        tree=ast.parse((ROOT/'integration_remote.py').read_text('utf-8'))
        function=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='rpc')
        namespace=dict(Path=Path,json=json,base64=base64,re=re)
        exec(compile(ast.Module(body=[function],type_ignores=[]),'remote_reader','exec'),namespace)
        uid='b'*32;folder=Path(self.tmp.name)/'orin_interface_v1/logs/manual_driving'/uid;folder.mkdir(parents=True)
        row={'kind':'sample','vision':{'capture_at':time.monotonic(),'frame_seq':77},'proposal':{'track_pos':{'s_m':22,'lap':9}}}
        (folder/'pid.jsonl').write_text('x'*300000+'\n'+json.dumps(row)+'\n'+json.dumps({'kind':'stop','reason':'PHYSICAL_ESTOP'})+'\n{partial',encoding='utf-8')
        with patch.object(Path,'home',return_value=Path(self.tmp.name)):
            result=namespace['rpc']({'path':'/ops07/observation/'+uid,'method':'GET'})
            data=json.loads(base64.b64decode(result['body']))
            self.assertEqual(data['position']['s_m'],22);self.assertEqual(data['stop_reason'],'PHYSICAL_ESTOP')
            for path,method in [('/ops07/observation/../secret','GET'),('/ops07/observation/'+uid,'POST')]:
                with self.assertRaises(ValueError):namespace['rpc']({'path':path,'method':method})


if __name__=='__main__':
    unittest.main(verbosity=2)
