import unittest
import numpy as np
from curve_replay import BoundaryTracker,ground,pixel,pursuit

def road(center=0,side='both'):
    mask=np.zeros((360,640),dtype=bool)
    for y in range(140,350):
        # Render constant-width ground boundaries.
        mid=ground(320,y,640,360)
        if mid is None:continue
        for name,x in [('left',center-.2525),('right',center+.2525)]:
            if side!='both' and name!=side:continue
            u=pixel(x,mid[1],640,360)[0]
            lo=max(0,int(u)-3);hi=min(640,int(u)+4)
            if hi>lo:mask[y,lo:hi]=True
    return mask

class CurveTests(unittest.TestCase):
    def test_projection_roundtrip(self):
        for u,v in [(180,230),(500,300),(320,200)]:
            self.assertTrue(np.allclose(pixel(*ground(u,v,640,360),640,360),[u,v]))
    def test_blank_never_holds_path(self):
        b=BoundaryTracker();self.assertTrue(b.step(road(),0)['valid'])
        self.assertFalse(b.step(np.zeros((360,640),bool),.1)['valid'])
    def test_single_without_history_rejected(self):
        self.assertFalse(BoundaryTracker().step(road(side='left'),0)['valid'])
    def test_single_expires(self):
        b=BoundaryTracker();b.step(road(),0)
        self.assertGreater(b.step(road(side='left'),.1)['single_rows'],0)
        self.assertFalse(b.step(road(side='left'),.7)['valid'])
    def test_straight_and_no_extrapolation(self):
        r=BoundaryTracker().step(road(),0)
        self.assertTrue(r['valid'])
        self.assertLess(abs(pursuit(r['path'],1)['curvature']),.03)
        self.assertIsNone(pursuit([[0,.6],[.1,.8]],2))
    def test_sign(self):
        self.assertGreater(pursuit([[.2,1]],1)['curvature'],0)
        self.assertLess(pursuit([[-.2,1]],1)['curvature'],0)
    def test_time_reversal_discards_identity(self):
        b=BoundaryTracker();b.step(road(),1)
        self.assertFalse(b.step(road(side='left'),.9)['valid'])
    def test_source_untouched(self):
        m=road();before=m.copy();BoundaryTracker().step(m,0);self.assertTrue(np.array_equal(m,before))
if __name__=='__main__':unittest.main()
