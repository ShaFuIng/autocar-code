import unittest
import numpy as np
from evaluate_ground_paths import normal_offset
class OffsetTests(unittest.TestCase):
    def test_straight_inner_edges(self):
        z=np.linspace(50,120,100)
        for side,x in [('left',-23),('right',23)]:
            path=normal_offset(np.column_stack([z*0+x,z]),23,side)
            np.testing.assert_allclose(path[:,0],0,atol=1e-9)
    def test_offset_is_perpendicular_and_constant(self):
        z=np.linspace(50,120,100);curve=np.column_stack([.1*z,z])
        offset=normal_offset(curve,23,'left')-curve
        np.testing.assert_allclose(np.linalg.norm(offset,axis=1),23)
        np.testing.assert_allclose(np.sum(offset*np.gradient(curve,axis=0),axis=1),0,atol=1e-9)
    def test_degenerate_rejected(self):
        with self.assertRaises(ValueError):normal_offset([[0,0]]*8,23,'left')
if __name__=='__main__':unittest.main()
