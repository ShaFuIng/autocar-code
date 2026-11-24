"""New shadow workflow deltas only; isolated files, no GPU/network/actuators."""
import copy
import json
import sys
import tempfile
import shutil
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT),str(ROOT.parent/'web'),str(ROOT.parent/'tests')]
import shadow_worker as worker
from shadow_offline import specification
from analysis_jobs import Analysis,read,write
from visual_pid import DEFAULTS
from test_calibrated_pid import scene

PARAMS=dict(DEFAULTS,center_us=1360.,row_ratio=.6)

class Store:
    def get(self,key):return dict(rev=1,data=dict(parameters=copy.deepcopy(PARAMS)))

class ShadowTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='shadow-test-',dir=ROOT)
        self.root=Path(self.temp.name)
        assert self.root.resolve().is_relative_to(ROOT.resolve())
        self.weight=self.root/'fake.pt';self.weight.write_bytes(b'fixture')
        self.calls=0
        self.inputs=[]
        for i in range(6):
            im,_=scene(sides=('left','right') if i<4 else ('left',))
            im[:16,:16]=i*10  # Distinct JPEG identity, outside lane observation.
            file=self.root/f'{i}.jpg';cv2.imencode('.jpg',im)[1].tofile(str(file))
            self.inputs.append(dict(image=str(file),frame=i,capture_s=10+i*.15))
        self.payload=dict(start=0,end=5,stride=1,pose='unknown',
            groups=[dict(algorithm=a,model='v4-640') for a in ['legacy9','calibrated-v7','calibrated-v7']])
        self.app=SimpleNamespace(store=Store())
        self.spec=specification(self.app,self.payload,6)
        self.patch=patch.object(worker,'CONFIGS',[('v4-640',str(self.weight),640)])
        self.patch.start()
    def tearDown(self):
        self.patch.stop();self.temp.cleanup()
    def predictor(self,weight,size):
        def predict(im):
            self.calls+=1
            return im[:,:,0]>128
        return predict
    def run_job(self,name,spec=None):
        folder=self.root/name;folder.mkdir()
        write(folder/'job.json',dict(id=name,source='fixture',models=['v4-640'],spec=spec or self.spec,
                                    total=24,execution='isolated test',kind='shadow'))
        write(folder/'input.json',self.inputs)
        return worker.run(folder,self.predictor)
    def test_shared_v4_and_independent_algorithms(self):
        r=self.run_job('one')
        self.assertEqual(self.calls,6)
        self.assertEqual(r['cache']['inferred'],6)
        self.assertEqual(r['cache']['path_reused'],6) # identical B/C
        self.assertEqual(r['models']['B'][4]['A']['mode'],'left')
        self.assertTrue(r['models']['B'][4]['pid']['valid'])
        self.assertFalse(r['models']['A'][4]['A']['valid'])
        self.assertIsNotNone(r['summaries']['A']['first_stop'])
        self.assertIsNone(r['summaries']['B']['first_stop'])
        self.assertEqual([r['frames'][i]['capture_s'] for i in range(6)],[f['capture_s'] for f in self.inputs])
    def test_pid_only_reuses_masks_and_paths(self):
        self.run_job('one');s=copy.deepcopy(self.spec)
        s['groups'][1]['parameters']['kp']=90
        r=self.run_job('two',s)
        self.assertEqual(self.calls,6);self.assertEqual(r['cache']['mask_reused'],6)
        self.assertEqual(r['cache']['path_reused'],18)
        self.assertEqual(r['spec']['groups'][1]['parameters']['kp'],90)
    def test_calibration_change_recomputes_only_affected_paths(self):
        self.run_job('one');s=copy.deepcopy(self.spec)
        s['groups'][1]['calibration']['lane_width_cm']=48
        r=self.run_job('two',s)
        self.assertEqual(self.calls,6);self.assertEqual(r['cache']['path_reused'],12)
        self.assertNotEqual(r['models']['B'][4]['A']['path'],r['models']['C'][4]['A']['path'])
    def test_weight_identity_invalidates_inference(self):
        self.run_job('one');self.weight.write_bytes(b'new fixture')
        r=self.run_job('two')
        self.assertEqual(self.calls,12);self.assertEqual(r['cache']['path_reused'],6)
    def test_invalid_time_is_not_repaired(self):
        self.inputs[2]['capture_s']=self.inputs[1]['capture_s']
        with self.assertRaisesRegex(ValueError,'時間戳'):self.run_job('one')
        self.assertEqual(self.calls,0)
    def test_cancel_keeps_partial_and_completed_mask_cache(self):
        original=self.predictor
        def factory(w,s):
            predict=original(w,s)
            def wrapped(im):
                result=predict(im)
                if self.calls==2:write(self.root/'one/cancel.json',{})
                return result
            return wrapped
        self.predictor=factory
        r=self.run_job('one')
        self.assertTrue(r['partial'])
        self.assertEqual(read(self.root/'one/status.json')['state'],'cancelled')
        self.assertEqual(r['cache']['inferred'],2)
        self.predictor=original
        r=self.run_job('two')
        self.assertEqual(r['cache']['mask_reused'],2)
        self.assertEqual(self.calls,6)
    def test_spec_freezes_pid_and_does_not_change_source(self):
        before=copy.deepcopy(self.payload)
        spec=specification(self.app,self.payload,6)
        spec['groups'][0]['parameters']['center_us']=1370
        self.assertEqual(self.payload,before)
        self.assertEqual(self.app.store.get('visual-pid')['data']['parameters']['center_us'],1360)
        self.assertEqual(spec['pose'],'unknown')
    def test_reject_bad_interval_matrix_and_algorithm(self):
        for change in [dict(stride=0),dict(end=9),dict(groups=[dict(algorithm='made-up')]),
                       dict(groups=[dict(algorithm='calibrated-v7',matrix=[[0,0,0]]*3)])]:
            with self.assertRaises(ValueError):specification(self.app,{**self.payload,**change},6)
    def test_stride_gap_remains_pid_failure_and_never_restarts_single_run(self):
        rows=[dict(valid=True,path=[[630,350],[640,500]],capture_s=t,frame=i,mode='double') for i,t in enumerate([0,.15,.30,.45,1.,1.15,1.30])]
        out,summary=worker.pid_rows(rows,PARAMS)
        self.assertEqual(summary['first_stop']['index'],4)
        self.assertTrue(out[-1]['pid']['valid'])
        self.assertEqual(out[-1]['simulated_run'],'stopped')
        self.assertTrue(all(not r['pid']['hardware_output'] for r in out))
    def test_temporal_and_saved_ui_fixture(self):
        spec=copy.deepcopy(self.spec);spec['groups'][2]['algorithm']='temporal-v2'
        spec['groups'][2]['calibration']=None
        r=self.run_job('ui',spec)
        self.assertEqual(len(r['models']['C']),6)
        self.assertEqual(r['cache']['inferred'],6)
        self.assertTrue(all('image_path' in row['A'] for row in r['models']['C']))
        # Deliberately labelled fixture for the new result UI, never V4 evidence.
        uid='00000000000000000000000020261021'
        target=ROOT/'data/analysis'/uid
        target.mkdir(exist_ok=True)
        for p in (self.root/'ui').iterdir():
            if p.is_dir():shutil.copytree(p,target/p.name,dirs_exist_ok=True)
        r.update(id=uid,fixture=True)
        r['limitations'].insert(0,'流程驗收專用：合成影格＋假辨識器；V4名稱僅測介面契約，並非真正V4推論結果。')
        write(target/'result.json',r)
        write(target/'job.json',dict(id=uid,kind='shadow',source='synthetic-acceptance',title='離線影子流程驗收（合成影格／假辨識，非V4結果）',
            models=['假辨識器：測V4介面契約'],created=time.time(),state='completed',total=24,fixture=True))
        write(target/'status.json',dict(state='completed',completed=24,total=24))
    def test_registry_routes_shadow_and_rejects_overlap(self):
        from unittest.mock import Mock
        app=SimpleNamespace(store=SimpleNamespace(root=self.root,get=Store().get),index_synced=lambda:None,
            replays={'local':dict(title='local',has_images=True,kind='field',rows=self.inputs)},
            assets={'local/'+str(i).zfill(6)+'.jpg':Path(r['image']) for i,r in enumerate(self.inputs)})
        registry=Analysis(app)
        fake=Mock();fake.pid=12345;fake.poll.return_value=None
        payload={**self.payload,'kind':'shadow','models':['v4-640'],'source':'local','start':1,'end':5,'stride':2}
        with patch.object(registry,'models',return_value=[dict(id='v4-640',available=True)]), \
             patch('analysis_jobs.subprocess.Popen',return_value=fake) as launch, \
             patch('analysis_jobs.threading.Thread'):
            job=registry.start(payload)
            self.assertEqual([r['frame'] for r in read(registry.root/job['id']/'input.json')],[1,3,5])
            self.assertIn('shadow_worker.py',launch.call_args.args[0][1])
            with self.assertRaisesRegex(ValueError,'已有離線'):registry.start(payload)
            registry.cancel({'job':job['id']})
            self.assertTrue((registry.root/job['id']/'cancel.json').exists())

if __name__=='__main__':
    cv2.setNumThreads(2)
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ShadowTests))
    acceptance=dict(date='2026-10-21',passed=result.wasSuccessful(),tests=result.testsRun,
        scope='新增離線影子流程；合成影格＋假辨識器，實際路徑算法與PID。未重跑舊套件、未載入V4 GPU、未連Orin。',
        hardware_output=False)
    (ROOT/'OFFLINE_SHADOW_ACCEPTANCE.json').write_text(json.dumps(acceptance,ensure_ascii=False,indent=2),encoding='utf-8')
    sys.exit(not result.wasSuccessful())
