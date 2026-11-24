import sys,json,io,base64,tempfile,threading,queue,unittest
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'laptop_workbench/data/experiments/k-live-control/payload/web'),str(ROOT/'web'),str(ROOT/'laptop_workbench')]
import live_pid
live_pid.ROOT=ROOT/'web'
import live_model_worker as worker
from live_pid import LivePID
from road_live import RoadController
from pid_paths import selected,ROAD_FILES
JOB=ROOT/'laptop_workbench/data/analysis/99e548cc753542e2b402518004abd54f'
PARAMS=json.loads((JOB/'job.json').read_text(encoding='utf-8'))['spec']['groups'][0]['parameters']

class KLiveTests(unittest.TestCase):
    def test_registry_and_dependencies(self):
        for a in ('road-image-v3','road-memory-v3'):self.assertTrue(selected(a,control=True)['control'])
        for n in ROAD_FILES:self.assertTrue((ROOT/'laptop_workbench'/n).is_file())
        with self.assertRaises(ValueError):selected('road-v7-v3',control=True)

    def test_frozen_v37_replay_equivalence(self):
        result=json.loads((JOB/'result.json').read_text(encoding='utf-8'))
        for name in ('A','B'):
            pid=RoadController(PARAMS)
            for row in result['models'][name]:
                r=row['A'];actual=pid.step_path(r['capture_s'],r['path'] if r['valid'] else [])
                for key in ('valid','reason','steer_us','pwm','sample_row_ratio','derivative_rebased'):
                    self.assertEqual(actual.get(key),row['pid'].get(key),(name,r['frame'],key))

    def test_worker_protocol_and_session_memory(self):
        image=np.zeros((360,640,3),np.uint8)
        cv2.line(image,(140,359),(270,110),(255,255,255),8)
        cv2.line(image,(500,359),(370,110),(255,255,255),8)
        single=image.copy();single[:,330:]=0
        class Model:
            def __init__(self,w):pass
            def predict(self,im):return im[:,:,0]>128
        for algorithm in ('road-image-v3','road-memory-v3'):
            frames=[image,image,single,single]
            requests=[dict(seq=i+1,captured=1+i*.15,row_ratio=.6,path_algorithm=algorithm,jpeg=base64.b64encode(cv2.imencode('.jpg',im)[1]).decode()) for i,im in enumerate(frames)]
            with tempfile.TemporaryDirectory() as d:
                root=Path(d);(root/'weights').write_bytes(b'fixture');(root/'config').write_text(json.dumps({'test':dict(adapter='fake',weights=str(root/'weights'))}))
                out=io.StringIO()
                with patch.object(sys,'argv',['worker',str(root/'config'),'test']),patch.object(sys,'stdin',io.StringIO('\n'.join(map(json.dumps,requests)))),patch.dict(worker.ADAPTERS,{'fake':Model}),patch('sys.stdout',out):worker.main()
                rows=[json.loads(x) for x in out.getvalue().splitlines()[1:]]
            self.assertEqual([r['seq'] for r in rows],[1,2,3,4])
            self.assertTrue(all(r['valid'] for r in rows))
            self.assertTrue(all(r['path_algorithm']==algorithm for r in rows))
            self.assertTrue(any('single' in s['source'] for s in rows[-1]['path_diagnostic']['road_segments']))
            self.assertIsNotNone(cv2.imdecode(np.frombuffer(base64.b64decode(rows[-1]['preview']),np.uint8),1))

    def stream(self,algorithm,fault=None):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);clock=[100.];commands=[];stops=[]
            class Sink:
                def profile(self):return dict(center_us=1360,min_us=940,max_us=1900)
                def busy(self):return False
                def start(self,p,_pid=False):
                    self.request=p;self.root=root;(root/'fake').mkdir();return dict(run_id='fake')
                def pid_input(self,p):commands.append(p)
                def request_stop(self,r):stops.append(r)
            sink=Sink()
            with patch('live_pid.threading.Thread.start'):live=LivePID(sink,None,threading.RLock(),lambda:False)
            live.path_algorithm=algorithm;live.row_ratio=.6;live.model_id='v4-default';live.state='ready';live.consecutive=3
            path=[[600,200],[640,430],[670,680]] # requested .6 has no coverage
            live.latest=dict(capture_at=100.,path=path,weights_sha256='fixture');live.last_preview_client=100.
            with patch('live_pid.time.monotonic',side_effect=lambda:clock[0]):
                live.start(dict(parameters=PARAMS,parameter_revision=37,owner='a'*32,seconds=3,area_clear=True,path_algorithm=algorithm,execution_mode='orin_timed'))
            self.assertIsInstance(live.controller,live_pid.PID)
            q=queue.Queue();q.put(json.dumps(dict(ready=True,protocol=1,model_id=live.model_id,weights_sha256='fixture')))
            jpg=cv2.imencode('.jpg',np.zeros((360,640,3),np.uint8))[1].tobytes();n=[0]
            def camera():
                n[0]+=1;clock[0]+=.15
                if n[0]>4:live.worker_stop.set()
                return jpg,clock[0],n[0]
            class Pipe:
                def write(self,line):
                    r=json.loads(line);valid=not(fault=='invalid' and r['seq']==4)
                    if fault=='stale' and r['seq']==4:clock[0]+=.45
                    q.put(json.dumps(dict(protocol=1,seq=r['seq'],model_id=live.model_id,path_algorithm=algorithm,weights_sha256='fixture',valid=valid,path=path,preview=base64.b64encode(jpg).decode())))
                def flush(self):pass
                def close(self):pass
            class Proc:
                stdin=Pipe()
                @property
                def stdout(self):
                    while True:
                        x=q.get(timeout=3)
                        if x is None:return
                        yield x
                def poll(self):return None
                def terminate(self):q.put(None)
            live.camera=camera
            with patch('live_pid.subprocess.Popen',return_value=Proc()) as spawn,patch('live_pid.subprocess.run'),patch('live_pid.os.getuid',return_value=0,create=True),patch('live_pid.os.getgid',return_value=0,create=True),patch('live_pid.time.monotonic',side_effect=lambda:clock[0]):live._loop()
            self.assertIsNone(live.error)
            self.assertEqual(commands[0]['pwm'],0)
            self.assertEqual(len(commands),3 if fault else 4)
            self.assertTrue(any(c['pwm']==20 for c in commands))
            self.assertEqual(live.path_algorithm,'junction-k-lane-v3')
            self.assertIn('control_age_s',live.latest)
            args=' '.join(spawn.call_args.args[0]);self.assertIn('--network none',args)
            self.assertIn('/web:',args.replace(chr(92),'/'))
            if fault:self.assertEqual(stops,['PID_INVALID_OR_STALE_PATH'])
            else:live.end('FIXTURE_END')

    def test_live_a_dynamic_start_to_fake_sink(self):self.stream('junction-k-lane-v3')
    def test_live_b_dynamic_start_to_fake_sink(self):self.stream('junction-k-lane-v3')
    def test_live_invalid_still_stops(self):self.stream('junction-k-lane-v3','invalid')
    def test_live_control_age_still_stops(self):self.stream('junction-k-lane-v3','stale')

if __name__=='__main__':
 suite=unittest.TestSuite(KLiveTests(n) for n in ['test_registry_and_dependencies','test_live_a_dynamic_start_to_fake_sink','test_live_invalid_still_stops','test_live_control_age_still_stops']);result=unittest.TextTestRunner(verbosity=2).run(suite);sys.exit(not result.wasSuccessful())

