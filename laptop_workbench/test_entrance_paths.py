import unittest
import numpy as np
import cv2
from entrance_paths import EntrancePath, AdaptivePID, sample_range, clean_path
from analysis_jobs import ROOT, read

class EntranceTests(unittest.TestCase):
    def setUp(self):
        self.params=read(ROOT/'data/analysis/31a8163b457d4ecc8069ceaa68f1fbc2/job.json')['spec']['groups'][0]['parameters']
    def mask(self):
        m=np.zeros((360,640),np.uint8)
        cv2.polylines(m,[np.array([[140,354],[200,290],[350,220],[480,180]])],False,1,10)
        cv2.polylines(m,[np.array([[620,354],[600,290],[575,220],[560,180]])],False,1,10)
        cv2.polylines(m,[np.array([[0,300],[70,270],[0,230]])],False,1,10)
        return m
    def test_bend_keeps_boundary_identity(self):
        m=self.mask();r=EntrancePath('entrance-image-v1').step(np.zeros((360,640,3),np.uint8),m,0.)
        self.assertEqual(r['mode'],'double');self.assertTrue(r['valid'])
        self.assertGreater(r['path'][0][0],640) # far bend may be entirely right of camera axis
    def test_single_ready_without_three_frame_wait(self):
        m=self.mask();tracker=EntrancePath('entrance-image-v1');im=np.zeros((360,640,3),np.uint8)
        tracker.step(im,m,0.)
        m[:,540:]=0
        r=tracker.step(im,m,.1)
        self.assertIn(r['mode'],('left','right'));self.assertTrue(r['valid'])
    def test_dynamic_row_and_frozen_parameters(self):
        p=dict(self.params);a=AdaptivePID(p)
        line=[[640.,480.],[660.,600.]]
        a.step(0.,line);r=a.step(.1,line)
        self.assertTrue(r['valid']);self.assertGreater(r['sample_row_ratio'],.6)
        self.assertEqual(p,self.params);self.assertEqual(a.pid.p,self.params)
        self.assertFalse(r['hardware_output'])
    def test_no_extrapolation_or_empty_history(self):
        a=AdaptivePID(self.params)
        r=a.step(0.,[[640,100],[650,200]])
        self.assertFalse(r['valid']);self.assertEqual(r['sample_reason'],'NO_SUPPORTED_ROW')
        self.assertFalse(a.step(.1,[])['valid'])
    def test_derivative_rebased_on_row_change(self):
        a=AdaptivePID(self.params)
        a.step(0.,[[300,400],[900,650]])
        a.step(.1,[[300,400],[900,650]])
        r=a.step(.2,[[492,480],[900,650]])
        self.assertTrue(r['derivative_rebased']);self.assertAlmostEqual(r['d'],0.,places=6)
    def test_blank_does_not_invent_path(self):
        r=EntrancePath('entrance-image-v1').step(np.zeros((360,640,3),np.uint8),np.zeros((360,640),np.uint8),0.)
        self.assertFalse(r['valid']);self.assertEqual(r['path'],[])
    def test_stale_width_not_reused(self):
        m=self.mask();a=EntrancePath('entrance-image-v1');im=np.zeros((360,640,3),np.uint8)
        a.step(im,m,0.);m[:,540:]=0
        self.assertFalse(a.step(im,m,1.)['valid'])

if __name__=='__main__':unittest.main()
