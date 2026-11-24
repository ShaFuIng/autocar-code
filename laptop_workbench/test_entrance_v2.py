import unittest
from unittest.mock import patch
import numpy as np
import cv2
from pathlib import Path
from analysis_jobs import read,ROOT
from entrance_paths_v2 import EntrancePath,local_offset,clean_path

class V2Tests(unittest.TestCase):
    def test_subpixel_edge_respects_pid_pixel_domain(self):
        a=clean_path([[1279.8,500],[1279,510],[1000,520]])
        self.assertEqual(len(a),2)
        self.assertLessEqual(a[:,0].max(),1279)
    def scene(self,double=True):
        mask=np.zeros((360,640),np.uint8)
        cv2.line(mask,(180,354),(250,200),1,10)
        if double:cv2.line(mask,(580,354),(450,200),1,10)
        return np.zeros((360,640,3),np.uint8),mask
    def test_visible_single_survives_many_seconds(self):
        a=EntrancePath('entrance-image-v2');im,m=self.scene();a.step(im,m,0.)
        im,m=self.scene(False)
        for i in range(1,101):
            r=a.step(im,m,i*.1)
            self.assertTrue(r['valid']);self.assertEqual(r['mode'],'left')
        self.assertGreater(r['candidates'][0]['width_age_s'],.65)
    def test_changed_boundary_is_not_unconditionally_trusted(self):
        a=EntrancePath('entrance-image-v2');im,m=self.scene();a.step(im,m,0.)
        m=np.zeros_like(m);cv2.line(m,(20,354),(40,200),1,10)
        r=a.step(im,m,.1);self.assertFalse(r['valid'])
        self.assertEqual(r['rejections'][0]['reason'],'BOUNDARY_IDENTITY_CHANGED')
    def test_history_takes_over_without_width_candidate(self):
        a=EntrancePath('entrance-temporal-v2');im,m=self.scene();a.step(im,m,0.);a.width=None
        im,m=self.scene(False)
        with patch('entrance_paths_v2.estimate_motion',return_value=(np.eye(3),{'valid':True})):
            r=a.step(im,m,.1)
        self.assertTrue(r['valid']);self.assertEqual(r['mode'],'history_left')
        self.assertGreater(len(r['history_path']),0)
    def test_blank_has_no_invented_history_candidate(self):
        a=EntrancePath('entrance-temporal-v2');im,m=self.scene();a.step(im,m,0.)
        with patch('entrance_paths_v2.estimate_motion',return_value=(np.eye(3),{'valid':True})):
            r=a.step(im,np.zeros_like(m),.1)
        self.assertFalse(r['valid'])
    def test_local_fold_keeps_other_sections(self):
        z=np.linspace(25,100,80);x=15*np.sin(z/8)
        sections,cuts=local_offset(np.column_stack([x,z]),23,'left')
        self.assertGreater(cuts,0);self.assertGreater(len(sections),0)
        self.assertTrue(all(np.all(np.diff(s[:,1])>0) for s in sections))
    def test_actual_120_calibrated_single_not_all_discarded(self):
        base=ROOT/'data/analysis/31a8163b457d4ecc8069ceaa68f1fbc2'
        cal=read(base/'job.json')['spec']['groups'][2]['calibration']
        im=cv2.imdecode(np.fromfile(str(base/'raw/000120.jpg'),np.uint8),1)
        mask=cv2.imdecode(np.fromfile(str(base/'v4-640/000120.png'),np.uint8),0)>0
        r=EntrancePath('entrance-v7-v2',cal).step(im,mask,14.366580963134766)
        self.assertTrue(r['valid']);self.assertGreater(len(r['path']),5)
        self.assertTrue(any('fold_splits' in c for c in r['candidates']))

if __name__=='__main__':unittest.main()
