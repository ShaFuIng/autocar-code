import unittest
from unittest.mock import patch
import cv2
import numpy as np
from road_segments_v3 import RoadPath,confidence,split_nodes,resample_memory

class RoadTests(unittest.TestCase):
    def test_fractional_warp_resampling_preserves_straight_road(self):
        memory=[dict(x=2*y+10,y=y,base=1.,observed_at=0.,alignment_error=.2) for y in np.arange(110.7,160.,4)]
        rows=resample_memory(memory,1.)
        self.assertGreater(len(rows),5)
        for n in rows.values():
            self.assertAlmostEqual(n['x'],2*n['y']+10)
            self.assertEqual(n['observed_at'],0.)
    def scene(self,short=False):
        m=np.zeros((360,640),np.uint8)
        cv2.line(m,(160,354),(280,170),1,10)
        cv2.line(m,(590,260 if short else 354),(480,210 if short else 170),1,10)
        return np.zeros((360,640,3),np.uint8),m
    def test_single_extends_beyond_double_overlap_same_frame(self):
        a=RoadPath('road-image-v3');im,m=self.scene();a.step(im,m,0.)
        im,m=self.scene(True);r=a.step(im,m,.1)
        self.assertGreater(r['coverage']['visible_rows'],r['coverage']['double_rows'])
        self.assertGreater(max(y for x,y in r['path']),650)
        self.assertIn('double',r['mode']);self.assertIn('single',r['mode'])
    def test_narrow_overlap_does_not_replace_broad_width(self):
        a=RoadPath('road-image-v3');im,m=self.scene();a.step(im,m,0.);span=np.ptp(a.width[:,0])
        im,m=self.scene(True);a.step(im,m,.1)
        self.assertEqual(np.ptp(a.width[:,0]),span)
    def test_age_and_alignment_reduce_confidence(self):
        n=dict(base=1.,observed_at=0.,alignment_error=0.)
        self.assertGreater(confidence(n,.1),confidence(n,1.))
        self.assertGreater(confidence(n,1.),confidence(dict(n,alignment_error=5),1.))
    def test_memory_propagation_does_not_refresh_age(self):
        a=RoadPath('road-memory-v3');im,m=self.scene();a.step(im,m,0.)
        with patch('road_segments_v3.estimate_motion',return_value=(np.eye(3),{'valid':True,'median_error_px':.1})):
            r=a.step(im,np.zeros_like(m),.1);r=a.step(im,np.zeros_like(m),.2)
        self.assertTrue(r['valid']);self.assertTrue(all(n['observed_at']==0 for n in a.memory))
        self.assertTrue(all(n['age_s']>=.2 for n in r['path_samples']))
    def test_failed_alignment_cannot_reuse_old_coordinates(self):
        a=RoadPath('road-memory-v3');im,m=self.scene();a.step(im,m,0.)
        with patch('road_segments_v3.estimate_motion',return_value=(None,{'valid':False})):
            r=a.step(im,np.zeros_like(m),.1)
        self.assertFalse(r['valid']);self.assertEqual(a.memory,[])
    def test_unrelated_sections_not_bridged(self):
        nodes=[dict(x=100,y=100),dict(x=101,y=104),dict(x=500,y=108),dict(x=501,y=140)]
        self.assertEqual(len(split_nodes(nodes)),3)
    def test_visible_conflict_rejects_history(self):
        a=RoadPath('road-memory-v3');im,m=self.scene();a.step(im,m,0.)
        for n in a.memory:n['x']+=100
        with patch('road_segments_v3.estimate_motion',return_value=(np.eye(3),{'valid':True})):
            r=a.step(im,m,.1)
        self.assertTrue(any(n['reason']=='HISTORY_CURRENT_CONFLICT' for n in r['rejections']))

if __name__=='__main__':unittest.main()
