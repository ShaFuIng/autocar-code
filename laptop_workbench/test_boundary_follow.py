import unittest
import numpy as np
from curve_replay import GEOMETRY,pixel
from boundary_follow import offset_boundary,PrioritySelector,anchored_identity,BoundaryIdentity
class BoundaryFollowTests(unittest.TestCase):
    def test_distant_crossing_is_not_right_boundary(self):
        self.assertFalse(anchored_identity([[310,240],[380,200]],'right'))
        self.assertTrue(anchored_identity([[633,312],[600,240]],'right'))
    def test_straight_offsets_to_center(self):
        for side,x in [('left',-.2525),('right',.2525)]:
            boundary=[pixel(x,z,640,360) for z in np.linspace(.8,2,50)]
            path=offset_boundary(boundary,side,GEOMETRY)
            self.assertGreater(len(path),10)
            self.assertLess(max(abs(p[0]) for p in path),.002)
    def test_blank_and_tiny(self):
        self.assertEqual(offset_boundary([],'left',GEOMETRY),[])
        self.assertEqual(offset_boundary([[200,300]],'left',GEOMETRY),[])
    def test_confirmation_and_no_stale_hold(self):
        s=PrioritySelector();p=[[0,1]]*10
        for _ in range(2):self.assertFalse(s.update({'double':p})['path'])
        self.assertEqual(s.update({'double':p})['mode'],'double')
        self.assertFalse(s.update({'left':p})['path'])
        s.update({'left':p});self.assertEqual(s.update({'left':p})['mode'],'left')
        self.assertFalse(s.update({})['path'])
    def test_keeps_identity_when_both_single_candidates_exist(self):
        s=PrioritySelector();p=[[0,1]]*10
        for _ in range(3):s.update({'left':p})
        self.assertEqual(s.update({'left':p,'right':p*2})['mode'],'left')
    def test_identity_survives_loss_of_near_anchor(self):
        import cv2
        tracker=BoundaryIdentity()
        mask=np.zeros((360,640),np.uint8)
        cv2.line(mask,(10,330),(450,160),1,9)
        a,d=tracker.step(mask,0)
        self.assertTrue(a['left'])
        mask[:]=0
        cv2.line(mask,(40,315),(455,160),1,9)
        a,d=tracker.step(mask,.1)
        self.assertTrue(a['left'])
        self.assertEqual(d['left']['reason'],'TRACK_ASSOCIATED')
        self.assertFalse(a['right'])
        a,d=tracker.step(np.zeros_like(mask),.2)
        self.assertFalse(a['left'])  # Never substitute an old path.
    def test_time_gap_cannot_retain_unanchored_identity(self):
        import cv2
        tracker=BoundaryIdentity();mask=np.zeros((360,640),np.uint8)
        cv2.line(mask,(10,330),(450,160),1,9);tracker.step(mask,0)
        mask[:]=0;cv2.line(mask,(40,315),(455,160),1,9)
        a,d=tracker.step(mask,2)
        self.assertFalse(a['left'])
    def test_projection_failure_is_distinct(self):
        diag={}
        self.assertFalse(offset_boundary([[x,20] for x in range(30)],'left',GEOMETRY,diagnostic=diag))
        self.assertEqual(diag['reason'],'PROJECTION_OUT_OF_RANGE')
if __name__=='__main__':unittest.main()
