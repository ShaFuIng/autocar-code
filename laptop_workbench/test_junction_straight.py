import unittest
import cv2
import numpy as np
from junction_straight import JunctionStraight,exit_candidates

class JunctionTests(unittest.TestCase):
    def mask(self):
        m=np.zeros((360,640),np.uint8)
        for x in [200,300,450,550]:cv2.line(m,(x,150),(x,238),1,5)
        return m
    def test_straight_exit_preferred_over_side_exit(self):
        c=exit_candidates(self.mask(),[0,250],[0,94])
        self.assertGreaterEqual(len(c),2)
        self.assertLess(c[0]['deviation_px'],1)
    def test_no_entry_does_not_invent_junction_path(self):
        a=JunctionStraight('junction-straight-v1');m=np.zeros((360,640),np.uint8)
        r=a.step(np.zeros((360,640,3),np.uint8),m,0)
        self.assertFalse(r['valid']);self.assertFalse(r['junction']['active'])
    def test_gap_bridged_to_visible_straight_exit(self):
        a=JunctionStraight('junction-straight-v1');a.entry=np.array([0.,250]);a.width=np.array([0.,94])
        r=a.step(np.zeros((360,640,3),np.uint8),self.mask(),1)
        self.assertEqual(r['reason'],'STRAIGHT_EXIT_BRIDGE');self.assertTrue(r['valid'])
        self.assertEqual(r['junction']['selected'],0)
        self.assertTrue(all(abs(x-500)<1 for x,y in r['path']))
    def test_missing_exit_is_explicit_short_estimate(self):
        a=JunctionStraight('junction-straight-v1');a.entry=np.array([0.,250]);a.width=np.array([0.,94])
        r=a.step(np.zeros((360,640,3),np.uint8),np.zeros((360,640),np.uint8),1)
        self.assertEqual(r['reason'],'ENTRY_DIRECTION_ONLY_NO_EXIT')
        self.assertIsNone(r['junction']['selected']);self.assertLessEqual(r['coverage']['visible_span_px'],170)

    def test_return_to_baseline_after_stable_exit(self):
        from types import SimpleNamespace
        a=JunctionStraight('junction-straight-v1');a.active=True;a.entry=np.array([0.,250]);a.width=np.array([0.,94])
        sample=[dict(x=250,y=y,source='double') for y in range(250,335,4)]
        a.base=SimpleNamespace(width=np.array([[y,94] for y in range(170,335,4)]),step=lambda *args:dict(path_samples=sample,valid=True,path=[],mode='double'))
        m=np.zeros((360,640),np.uint8)
        for x in [200,300]:cv2.line(m,(x,150),(x,354),1,5)
        for i in range(10):r=a.step(np.zeros((360,640,3),np.uint8),m,i*.1)
        self.assertFalse(r['junction']['active']);self.assertEqual(r['junction']['reason'],'FOLLOW_BASELINE')

if __name__=='__main__':unittest.main()
