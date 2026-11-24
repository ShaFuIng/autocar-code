"""Only switch-v1 deltas. Synthetic masks/current-pose snapshot; no I/O to car."""
import base64
import contextlib
import hashlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'web'), str(ROOT/'tests')]
from test_calibrated_pid import scene
from calibrated_path import CalibratedPath, load_calibration
from visual_pid import PID, DEFAULTS

TRACES = {}


def run_sequence(sides):
    tracker = CalibratedPath()
    result = []
    for i, s in enumerate(sides):
        image, mask = scene(sides=s)
        result.append(tracker.step(image, mask, 1+i*.15))
    return result


class SwitchTests(unittest.TestCase):
    def test_background_candidates_diagnostics(self):
        r = run_sequence([('left', 'right')]*3)[-1]
        self.assertEqual(r['mode'], 'double')
        for name, d in r['candidates'].items():
            self.assertTrue(d['ready'], name)
            self.assertEqual(d['consecutive_valid'], 3)
            self.assertAlmostEqual(d['change_cm'], 0)
            self.assertGreater(d['support_cm'][1], d['support_cm'][0])
            self.assertTrue(all(f['residual_cm'] is not None for f in d['boundary_fits'].values()))
        json.dumps(r, allow_nan=False)

    def test_drop_each_side_has_no_pid_gap(self):
        for side in ('left', 'right'):
            rows = run_sequence([('left', 'right')]*4+[(side,)]*3)
            pid = PID(dict(DEFAULTS, center_us=1360., row_ratio=.6))
            outputs = [pid.step_path(1+i*.15, r['path']) for i, r in enumerate(rows)]
            self.assertTrue(all(r['valid'] for r in rows[2:]))
            self.assertTrue(all(o['valid'] for o in outputs[3:]))
            self.assertEqual(rows[4]['transition'], 'BACKGROUND_FALLBACK')
            self.assertEqual(rows[4]['mode'], side)
            self.assertTrue(all(not o['hardware_output'] for o in outputs))
            TRACES[side] = [{'seq':i+1, 'mode':r['mode'], 'valid':r['valid'],
                            'transition':r['transition'], 'pid_valid':o['valid']}
                           for i,(r,o) in enumerate(zip(rows,outputs))]

    def test_recovery_and_flicker_holds_single_until_three(self):
        rows = run_sequence([('left','right')]*3+[('left',)]+
                            [('left','right'),('left',)]+[('left','right')]*3)
        self.assertEqual([r['mode'] for r in rows[3:]], ['left']*5+['double'])
        self.assertTrue(all(r['valid'] for r in rows[2:]))

    def test_missing_current_evidence_never_uses_history(self):
        rows = run_sequence([('left','right')]*3+[()]+[('left',)]*3)
        self.assertEqual([r['valid'] for r in rows[3:]], [False,False,False,True])
        self.assertEqual(rows[3]['path'], [])
        self.assertTrue(all(d['consecutive_valid']==0 for d in rows[3]['candidates'].values()))

    def test_new_opposite_side_does_not_inherit_confirmation(self):
        rows = run_sequence([('left',)]*3+[('right',)]*3)
        self.assertEqual([r['valid'] for r in rows], [False,False,True,False,False,True])

    def test_capture_gap_resets_background_readiness(self):
        t = CalibratedPath()
        im, mask = scene()
        for i in range(3): t.step(im, mask, 1+i*.15)
        r = t.step(im, mask, 2.)
        self.assertFalse(r['valid'])
        self.assertTrue(all(d['consecutive_valid']==1 for d in r['candidates'].values()))

    def test_current_snapshot_dropout_and_pid(self):
        data,_,_ = load_calibration()
        folder=ROOT/'laptop_workbench/data/experiments/camera-measure'/data['source_snapshot']['id']
        im=cv2.imdecode(np.fromfile(str(folder/'raw.jpg'),np.uint8),1)
        mask=cv2.imdecode(np.fromfile(str(folder/'mask.png'),np.uint8),-1)[:,:,3]>0
        observations=[]
        for side in ('left','right'):
            tracker=CalibratedPath()
            pid=PID(dict(DEFAULTS,center_us=1360.,row_ratio=.6))
            for i in range(4):
                r=tracker.step(im,mask,1+i*.15)
                pid.step_path(1+i*.15,r['path'])
            only=mask.copy()
            only[:,320:]=False if side=='left' else only[:,320:]
            if side=='right':only[:,:320]=False
            r=tracker.step(im,only,1.60)
            p=pid.step_path(1.60,r['path'])
            self.assertEqual(r['mode'],side)
            self.assertTrue(p['valid'])
            observations.append(dict(side=side,diagnostic=r,proposal=p))
        TRACES['snapshot']=observations

    def test_worker_forwards_diagnostics_and_current_fallback(self):
        import live_model_worker as worker
        class Model:
            def __init__(self,weights):pass
            def predict(self,image):return image[:,:,0]>128
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            (root/'weights').write_bytes(b'fixture')
            (root/'config.json').write_text(json.dumps({'test':{'adapter':'fake','weights':str(root/'weights')}}))
            requests=[]
            for i,s in enumerate([('left','right')]*4+[('left',)]):
                im,_=scene(sides=s)
                jpeg=base64.b64encode(cv2.imencode('.jpg',im)[1]).decode()
                requests.append(dict(seq=i+1,captured=1+i*.15,jpeg=jpeg,row_ratio=.6,path_algorithm='calibrated-v7'))
            output=io.StringIO()
            with patch.object(sys,'argv',['worker',str(root/'config.json'),'test']), \
                 patch.object(sys,'stdin',io.StringIO(''.join(json.dumps(r)+'\n' for r in requests))), \
                 patch.dict(worker.ADAPTERS,{'fake':Model}),contextlib.redirect_stdout(output):
                worker.main()
            rows=[json.loads(line) for line in output.getvalue().splitlines()[1:]]
            self.assertTrue(rows[-1]['valid'])
            d=rows[-1]['path_diagnostic']
            self.assertEqual(d['transition'],'BACKGROUND_FALLBACK')
            self.assertEqual(d['switch_version'],1)
            self.assertTrue(d['candidates']['left']['ready'])


if __name__=='__main__':
    cv2.setNumThreads(2)
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(SwitchTests))
    sources=['web/calibrated_path.py','web/live_model_worker.py','tests/test_v7_switch.py']
    report=dict(passed=result.wasSuccessful(),tests=result.testsRun,
                source_sha256={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources},
                traces=TRACES,hardware_output=False,network_used=False,
                scope='僅切換v1差異；舊95項及前端11項未重跑。真實快照的遮罩丟線為人工模擬，非連續實車影片。')
    out=ROOT/'laptop_workbench/data/experiments/v7-pid-integration/switch-tests.json'
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    sys.exit(not result.wasSuccessful())
