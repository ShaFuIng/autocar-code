"""V7 current-pose fixtures, real geometry/PID, no hardware/network."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'web'),str(ROOT/'laptop_workbench'),str(ROOT/'tests')]
from calibrated_path import CalibratedPath,load_calibration,validate_row
from ground_calibration import project
from visual_pid import PID,DEFAULTS
import test_live_pid


def scene(shift=0, sides=('left','right'), bend=0):
    _,H,_=load_calibration()
    inverse=np.linalg.inv(H)
    mask=np.zeros((360,640),np.uint8)
    z=np.linspace(35,150,180)
    center=shift+bend*((z-75)/70)**2
    for side in sides:
        inner=-23 if side=='left' else 23
        outer=inner+(-3 if side=='left' else 3)
        pts=np.vstack([np.column_stack([center+inner,z]),
                       np.column_stack([center+outer,z])[::-1]])
        pixels=project(pts,inverse).round().astype(np.int32)
        cv2.fillPoly(mask,[pixels],1)
    return np.repeat((mask*255)[:,:,None],3,axis=2),mask>0


class V7Tests(unittest.TestCase):
    def test_worker_protocol_carries_v7_geometry_and_identity(self):
        import base64,contextlib,io,json,tempfile
        import live_model_worker as worker
        im,_=scene()
        class Model:
            def __init__(self,weights):pass
            def predict(self,image):return image[:,:,0]>128
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'weights').write_bytes(b'fixture')
            (root/'config.json').write_text(json.dumps({'test':{'adapter':'fake','weights':str(root/'weights')}}))
            jpeg=base64.b64encode(cv2.imencode('.jpg',im)[1]).decode()
            requests=[dict(seq=i,captured=i*.15,jpeg=jpeg,row_ratio=.6,path_algorithm='calibrated-v7') for i in range(1,6)]
            out=io.StringIO()
            with patch.object(sys,'argv',['worker',str(root/'config.json'),'test']), \
                 patch.object(sys,'stdin',io.StringIO(''.join(json.dumps(r)+'\n' for r in requests))), \
                 patch.dict(worker.ADAPTERS,{'fake':Model}),contextlib.redirect_stdout(out):
                worker.main()
            rows=[json.loads(line) for line in out.getvalue().splitlines()[1:]]
            self.assertEqual([r['valid'] for r in rows],[False,False,True,True,True])
            self.assertEqual(rows[-1]['path_diagnostic']['calibration_sha256'],load_calibration()[2])
            p=PID(dict(DEFAULTS,center_us=1360.,row_ratio=.6))
            p.step_path(.6,rows[-2]['path'])
            self.assertTrue(p.step_path(.75,rows[-1]['path'])['valid'])

    def test_geometry_to_pid_left_center_right_and_single(self):
        for shift in (-5,0,5):
            for sides in (('left','right'),('left',),('right',)):
                im,mask=scene(shift,sides)
                tracker=CalibratedPath()
                for i in range(3):r=tracker.step(im,mask,1+i*.15)
                self.assertTrue(r['valid'],(shift,sides,r))
                metric=np.asarray(r['metric_path_cm'])
                self.assertLess(abs(np.median(metric[:,0])-shift),1.)
                self.assertTrue(np.all((metric[:,1]>=50)&(metric[:,1]<=120)))
                pid=PID(dict(DEFAULTS,center_us=1360.,row_ratio=.6))
                self.assertEqual(pid.step_path(1,r['path'])['pwm'],0)
                p=pid.step_path(1.15,r['path'])
                self.assertTrue(p['valid'])
                self.assertFalse(p['hardware_output'])

    def test_current_snapshot_and_confirmation_switch(self):
        d,_,_=load_calibration()
        folder=ROOT/'laptop_workbench/data/experiments/camera-measure'/d['source_snapshot']['id']
        im=cv2.imdecode(np.fromfile(str(folder/'raw.jpg'),np.uint8),1)
        mask=cv2.imdecode(np.fromfile(str(folder/'mask.png'),np.uint8),-1)[:,:,3]>0
        tracker=CalibratedPath()
        for i in range(3):r=tracker.step(im,mask,1+i*.15)
        self.assertTrue(r['valid'])
        self.assertEqual(r['mode'],'double')
        only=mask.copy();only[:,320:]=False
        self.assertEqual(tracker.step(im,only,1.45)['mode'],'left')
        self.assertTrue(tracker.step(im,only,1.60)['valid'])
        self.assertEqual(tracker.step(im,only,1.75)['mode'],'left')
        self.assertFalse(tracker.step(im,mask&False,1.9)['valid'])

    def test_rows_pose_size_and_time_gap(self):
        for row in (.5,.6):self.assertTrue(50<=validate_row(row)<=120)
        with self.assertRaises(ValueError):validate_row(.75)
        im,mask=scene();tracker=CalibratedPath()
        with self.assertRaises(ValueError):tracker.step(im[:300],mask[:300],1)
        for i in range(3):tracker.step(im,mask,1+i*.15)
        self.assertFalse(tracker.step(im,mask,3)['valid'])

    def test_bend_supported_without_metric_extrapolation(self):
        im,mask=scene(bend=8)
        tracker=CalibratedPath()
        for i in range(3):r=tracker.step(im,mask,1+i*.15)
        self.assertTrue(r['valid'])
        p=np.asarray(r['metric_path_cm'])
        expected=8*((p[:,1]-75)/70)**2
        self.assertLess(np.max(abs(p[:,0]-expected)),1.)

    def test_real_start_uses_fake_manual_and_requires_pose_and_hash(self):
        import tempfile
        from pid_paths import PATHS
        self.assertTrue(PATHS['calibrated-v7']['control'])
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as folder:
            live,m,now,commands,request=test_live_pid.LiveTests().make(folder)
            live.path_algorithm='calibrated-v7';live.row_ratio=.6
            request['path_algorithm']='calibrated-v7'
            request['parameters']['row_ratio']=.6
            sha=load_calibration()[2]
            live.latest['path_diagnostic']={'calibration_sha256':sha}
            with self.assertRaisesRegex(ValueError,'相機姿態'):live.start(request)
            request['camera_pose_confirmed']=True
            with self.assertRaisesRegex(ValueError,'identity'):live.start(request)
            request['calibration_sha256']=sha
            with patch('manual_drive.threading.Thread.start'):
                live.start(request)
            self.assertEqual(live.run['calibration_sha256'],sha)
            self.assertEqual(commands,[])
            live.end()


if __name__=='__main__':unittest.main()
