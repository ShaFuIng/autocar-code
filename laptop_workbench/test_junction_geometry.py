import copy, hashlib, json, unittest
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
from junction_geometry import JunctionGeometry, geometry

class GeometryTest(unittest.TestCase):
    def test_pair_and_near_branch(self):
        mask=np.zeros((360,640),np.uint8)
        cv2.line(mask,(280,180),(200,260),1,5)
        cv2.line(mask,(360,180),(440,260),1,5)
        cv2.line(mask,(200,260),(30,300),1,5)
        fit,g=geometry(mask,np.array([2.,-280.]))
        self.assertEqual(g['source'],'PAIRED_BOUNDARIES')
        self.assertLess(max(abs(np.polyval(fit,[180,334])-320)),8)
        near=np.zeros_like(mask);cv2.line(near,(200,240),(30,300),1,5)
        self.assertIsNone(geometry(near,np.array([2.,-280.]))[0])

    def test_failed_motion_keeps_actual_anchor_frame(self):
        tracker=JunctionGeometry();tracker.main=np.array([0.,320.]);tracker.anchor_gray=np.zeros((360,640),np.uint8);tracker.anchor_t=0.
        tracker.last_t=0.
        def base(obj,image,mask,t,baseline):
            obj.gray=np.full((360,640),int(t*100),np.uint8);obj.last_t=t
            return dict(junction=dict(active=True,entry_direction=[],state='CROSS'),valid=False)
        calls=[]
        def motion(old,new):
            calls.append((int(old[0,0]),int(new[0,0])))
            if len(calls)==1:return None,dict(valid=False,reason='TEST_REJECT')
            return np.array([[1,0,4],[0,1,0],[0,0,1]],float),dict(valid=True,reason='TEST_VALID')
        with patch('junction_geometry.JunctionLeft.step',base),patch('junction_geometry.estimate_motion',motion),patch('junction_geometry.geometry',return_value=(None,{})):
            tracker.step(None,None,.1,{})
            self.assertEqual(tracker.anchor_t,0)
            tracker.step(None,None,.2,{})
        self.assertEqual(calls,[(0,10),(0,20)])
        self.assertAlmostEqual(tracker.main[1],324)
        self.assertEqual(tracker.anchor_t,.2)

    def test_frozen_results_and_focus(self):
        p=Path(__file__).parent/'data';e=p/'experiments/junction-cases'
        m=json.loads((e/'cases.json').read_text(encoding='utf-8'));c=m['cases'][0]
        report=json.loads((e/'v9-report.json').read_text(encoding='utf-8'))
        self.assertNotIn('geometry_job',m['cases'][1])
        for key,digest in report['cases'][0]['frozen_sha256'].items():
            self.assertEqual(hashlib.sha256((p/'analysis'/c[key]/'result.json').read_bytes()).hexdigest(),digest)
        rows=json.loads((p/'analysis'/c['geometry_job']/'result.json').read_text(encoding='utf-8'))['models']['J']
        for n in [96,99,105]:
            r=rows[n];self.assertIsNone(r['pid']);j=r['A']['junction']
            self.assertTrue(j['geometry']['selected']);self.assertTrue(j['geometry']['center'])
            self.assertEqual([j['entry_direction'][0][1],j['entry_direction'][-1][1]],[180,334])
        self.assertEqual(rows[94]['A']['junction']['geometry']['motion']['reason'],'REFERENCE_DISPLACEMENT_REJECTED')

if __name__=='__main__':unittest.main()
