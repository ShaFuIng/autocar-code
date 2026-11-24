import unittest
from unittest.mock import patch
import cv2
import numpy as np
from road_segments_v31 import RoadPath

class PatchTests(unittest.TestCase):
    def scene(self):
        im=np.zeros((360,640,3),np.uint8);m=np.zeros((360,640),np.uint8)
        cv2.line(m,(100,354),(260,170),1,10);cv2.line(m,(580,354),(440,170),1,10)
        return im,m
    def test_reacquire_visible_single_after_large_shift(self):
        im,m=self.scene();a=RoadPath('road-memory-v3.1');a.step(im,m,0)
        one=np.zeros_like(m);cv2.line(one,(230,354),(390,170),1,10)
        with patch('road_segments_v31.estimate_motion',return_value=(None,{'valid':False})):
            r=a.step(im,one,.1)
        self.assertTrue(r['valid']);self.assertTrue(r['candidates']);self.assertEqual(r['history_points'],0)
    def test_failed_alignment_stored_but_not_drawn_then_realign(self):
        im,m=self.scene();a=RoadPath('road-memory-v3.1');a.step(im,m,0);origin=a.memory_gray.copy()
        with patch('road_segments_v31.estimate_motion',return_value=(None,{'valid':False})):
            r=a.step(im+10,np.zeros_like(m),.1)
        self.assertFalse(r['valid']);self.assertTrue(a.memory);self.assertFalse(r['history_segments'])
        self.assertTrue(np.array_equal(origin,a.memory_gray))
        with patch('road_segments_v31.estimate_motion',return_value=(np.eye(3),{'valid':True,'median_error_px':0})):
            r=a.step(im,np.zeros_like(m),.2)
        self.assertTrue(r['valid']);self.assertGreater(r['history_points'],0)
        self.assertTrue(all(n['observed_at']==0 for n in a.memory))
    def test_old_memory_loses_confidence(self):
        im,m=self.scene();a=RoadPath('road-memory-v3.1');a.step(im,m,0)
        with patch('road_segments_v31.estimate_motion',return_value=(None,{'valid':False})):
            r=a.step(im,np.zeros_like(m),100)
        self.assertFalse(r['valid']);self.assertFalse(a.memory)

if __name__=='__main__':unittest.main()
