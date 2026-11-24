import base64
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.request
import urllib.error
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('dashboard', ROOT/'web/server.py')
dashboard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dashboard)


class DashboardTests(unittest.TestCase):
    def test_web_calibration_enable_gates(self):
        import time
        v={'fw':'orin-v1.4.1-manual','servo_calibration':True,'state':'LOCKED','generation':3,'web_calibration_start':True,'pins':{'D11':1},'servo_profile':dict(dashboard.saved_profile,synced=True)}
        with patch.dict(dashboard.data,vehicle=v,uart_at=time.monotonic(),uart_error=None):
            self.assertEqual(dashboard.validate_control({'action':'enable','generation':3}),'CAL_ENABLE,3')
            for key,value in [('state','FAULT'),('web_calibration_start',False),('generation',4),('pins',{'D11':0})]:
                old=v[key];v[key]=value
                with self.assertRaises(ValueError):dashboard.validate_control({'action':'enable','generation':3})
                v[key]=old

    def setUp(self):
        for name,value in [('saved_profile',{'revision':1,'min_us':1010,'center_us':1460,'max_us':1810}),('profile_loaded',True)]:
            holder=patch.object(dashboard,name,value)
            holder.start();self.addCleanup(holder.stop)

    def test_authentication_and_read_only_routes(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            home = Path(tmp)
            secret = home/'.config/orin_lab'
            secret.mkdir(parents=True)
            (secret/'token').write_text('test-password')
            with patch.object(dashboard.Path, 'home', return_value=home):
                server = dashboard.ThreadingHTTPServer(('127.0.0.1',0),dashboard.Handler)
                thread = threading.Thread(target=server.serve_forever)
                thread.start()
                try:
                    base = 'http://127.0.0.1:' + str(server.server_port)
                    with self.assertRaises(urllib.error.HTTPError) as exc:
                        urllib.request.urlopen(base+'/api/status')
                    self.assertEqual(exc.exception.code,401)
                    header = {'Authorization':'Basic '+base64.b64encode(b'jetson:test-password').decode()}
                    with urllib.request.urlopen(urllib.request.Request(base+'/api/status',headers=header)) as r:
                        state = json.load(r)
                    self.assertIn('uart_age_s',state)
                    with self.assertRaises(urllib.error.HTTPError) as exc:
                        urllib.request.urlopen(urllib.request.Request(base+'/drive',data=b'{}',headers=header))
                    self.assertEqual(exc.exception.code,404)
                finally:
                    server.shutdown()
                    thread.join()
                    server.server_close()

    def test_calibration_validation(self):
        import time
        vehicle={'fw':'orin-v1.3.0-bench','servo_calibration':True,'state':'READY',
                 'generation':3,'pins':{'D11':1},'steer_output_enabled':False,'steer_us_command':0,
                 'servo_profile':dict(dashboard.saved_profile,synced=True)}
        with patch.dict(dashboard.data,vehicle=vehicle,uart_at=time.monotonic(),uart_error=None):
            self.assertEqual(dashboard.validate_control({'generation':3,'us':1460}),'SERVO,3,1460')
            self.assertEqual(dashboard.validate_control({'generation':3,'us':1340}),'SERVO,3,1340')
            with self.assertRaises(ValueError): dashboard.validate_control({'action':'step','generation':3,'delta':100})
            for payload in ({'generation':2,'us':1460},{'generation':3,'us':True},{'generation':3,'us':499}):
                with self.assertRaises(ValueError): dashboard.validate_control(payload)
            vehicle.update(steer_output_enabled=True,steer_us_command=1460)
            self.assertEqual(dashboard.validate_control({'action':'step','generation':3,'delta':10}),'SERVO,3,1470')
            vehicle['steer_us_command']=1470
            self.assertEqual(dashboard.validate_control({'action':'step','generation':3,'delta':10}),'SERVO,3,1480')
            self.assertEqual(dashboard.validate_control({'action':'center','generation':3}),'CENTER,3')
            vehicle['servo_returning']=True
            with self.assertRaises(ValueError): dashboard.validate_control({'action':'step','generation':3,'delta':10})
            vehicle['servo_returning']=False
            vehicle['steer_us_command']=1810
            with self.assertRaises(ValueError): dashboard.validate_control({'action':'step','generation':3,'delta':5})
            vehicle['servo_profile']['revision']=2
            with self.assertRaises(ValueError): dashboard.validate_control({'action':'center','generation':3})
            vehicle['state']='LOCKED'
            self.assertEqual(dashboard.validate_control({'action':'stop'}),'STOP')
        self.assertTrue(dashboard.frame(1,'SYNC').startswith(b'$1,SYNC*'))

    def test_save_persists_without_control_and_reloads(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            path=Path(tmp)/'servo.json'
            v={'steer_output_enabled':True,'steer_us_command':1460}
            with patch.object(dashboard,'PROFILE_PATH',path),patch.dict(dashboard.data,vehicle=v),patch.object(dashboard,'submit_control') as control:
                result=dashboard.save_profile({'expected_revision':1,'min_us':1010,'center_us':1470,'max_us':1810})
                control.assert_not_called()
                self.assertTrue(result['saved']);self.assertFalse(result['synced'])
                self.assertEqual(v['steer_us_command'],1460)
                self.assertEqual(json.loads(path.read_text())['center_us'],1470)
                dashboard.saved_profile={'revision':1,'min_us':1010,'center_us':1460,'max_us':1810}
                loaded=dashboard.load_profile(path)
                self.assertEqual(loaded['revision'],2);self.assertEqual(loaded['center_us'],1470)

    def test_invalid_profile_and_excluded_position_do_not_overwrite(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
            path=Path(tmp)/'servo.json'
            dashboard.write_profile(dashboard.saved_profile,path)
            original=path.read_bytes()
            with patch.object(dashboard,'PROFILE_PATH',path),patch.dict(dashboard.data,vehicle={'steer_output_enabled':True,'steer_us_command':1460}):
                for payload in ({'expected_revision':1,'min_us':1480,'center_us':1500,'max_us':1810},
                                {'expected_revision':0,'min_us':1010,'center_us':1460,'max_us':1810},
                                {'expected_revision':1,'min_us':1010,'center_us':1810,'max_us':1810}):
                    with self.assertRaises(ValueError):dashboard.save_profile(payload)
                    self.assertEqual(path.read_bytes(),original)
            path.write_text('{broken json')
            with self.assertRaises(ValueError):dashboard.load_profile(path)

    def test_motor_command_validation(self):
        import time
        v={'fw':'orin-v1.3.0-bench','state':'READY','generation':3,'pins':{'D11':1},
           'servo_profile':dict(dashboard.saved_profile,synced=True),'servo_returning':False}
        with patch.dict(dashboard.data,vehicle=v,uart_at=time.monotonic(),uart_error=None):
            self.assertEqual(dashboard.validate_motor_control({'motor_action':'arm','generation':3,'duration_ms':5000}),'TEST_ARM,3,5000')
            v['state']='MOTOR_TEST'
            for value in (-50,50):
                self.assertEqual(dashboard.validate_motor_control({'motor_action':'move','generation':3,'right':value,'left':value}),f'TEST,3,{value},{value}')
            for value in (-51,51,19.8,True):
                with self.assertRaises(ValueError):dashboard.validate_motor_control({'motor_action':'move','generation':3,'right':value,'left':0})
            self.assertEqual(dashboard.validate_motor_control({'motor_action':'move','generation':3,'right':-10,'left':15}),'TEST,3,-10,15')
            for payload in ({'motor_action':'move','generation':3,'right':51,'left':0},{'motor_action':'move','generation':2,'right':10,'left':0}):
                with self.assertRaises(ValueError):dashboard.validate_motor_control(payload)
            with self.assertRaises(ValueError):dashboard.validate_control({'action':'step','generation':3,'delta':10})
            self.assertEqual(dashboard.validate_motor_control({'action':'stop'}),'STOP')

    def test_notebook_cells_compile(self):
        notebook = json.loads((ROOT/'lab/notebooks/01_GPU與相機驗證.ipynb').read_text(encoding='utf-8'))
        for cell in notebook['cells']:
            if cell['cell_type']=='code':
                compile(''.join(cell['source']),'notebook','exec')
