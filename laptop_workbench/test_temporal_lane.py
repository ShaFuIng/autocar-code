import unittest
import cv2
import numpy as np
from temporal_lane import TemporalLane,estimate_motion,transform,thin

def scene():
    rng=np.random.default_rng(8)
    image=rng.integers(0,255,(360,640,3),dtype=np.uint8)
    mask=np.zeros((360,640),np.uint8)
    cv2.line(mask,(90,350),(280,145),1,11)
    cv2.line(mask,(550,350),(360,145),1,11)
    return image,mask

class TemporalTests(unittest.TestCase):
    def test_translation(self):
        image,_=scene();g=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        moved=cv2.warpAffine(g,np.float32([[1,0,5],[0,1,3]]),(640,360))
        h,d=estimate_motion(g,moved)
        self.assertTrue(d['valid'])
        self.assertTrue(np.allclose(transform([[300,250]],h),[[305,253]],atol=.5))
    def test_memory_expires_without_self_renewal(self):
        im,mask=scene();b=TemporalLane();self.assertTrue(b.step(im,mask,0)['valid'])
        single=mask.copy();single[:,320:]=0
        r=b.step(im,single,.1);self.assertTrue(r['valid']);self.assertGreater(r['single_rows'],0)
        for t in [.2,.4,.6]:b.step(im,single,t)
        self.assertFalse(b.step(im,single,.8)['valid'])
    def test_single_cold_and_blank(self):
        im,mask=scene();mask[:,320:]=0
        self.assertFalse(TemporalLane().step(im,mask,0)['valid'])
        self.assertFalse(TemporalLane().step(im,mask*0,0)['valid'])
    def test_failed_alignment_discards_history(self):
        im,mask=scene();b=TemporalLane();b.step(im,mask,0)
        r=b.step(im*0,mask*0,.1)
        self.assertFalse(r['valid']);self.assertEqual(r['memory'],{'left':[],'right':[]})
    def test_no_input_mutation(self):
        im,mask=scene();copy=mask.copy();s=thin(mask)
        self.assertTrue(np.array_equal(mask,copy));self.assertLess(s.sum(),mask.sum())
    def test_time_reset(self):
        im,mask=scene();b=TemporalLane();b.step(im,mask,1)
        mask[:,320:]=0
        self.assertFalse(b.step(im,mask,0)['valid'])
if __name__=='__main__':unittest.main()
