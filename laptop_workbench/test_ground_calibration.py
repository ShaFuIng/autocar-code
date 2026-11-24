import unittest
import numpy as np
from ground_calibration import calculate,fit,project
class ProjectionTests(unittest.TestCase):
    def test_known_projection_and_independent_point(self):
        H=np.array([[3,.1,320],[0,.2,180],[0,.01,1.]])
        lines=[]
        for z in (50,80,120):
            points=project([[-23,z],[23,z]],H)
            lines.append(dict(cm=z,left=points[0].tolist(),right=points[1].tolist(),confirmed=True))
        r=calculate(lines)
        self.assertLess(r['fit']['max_cm'],.001)
        independent=project([[5,100]],H)
        np.testing.assert_allclose(project(independent,r['matrix']),[[5,100]],atol=.001)
    def test_invalid_inputs(self):
        with self.assertRaises(ValueError):fit([[0,0],[1,1],[2,2],[3,3]],[[0,0],[1,1],[2,2],[3,3]])
        with self.assertRaises(ValueError):calculate([])
    def test_validation_not_used(self):
        lines=[dict(cm=z,left=[320-10000/(z+20),100+10000/(z+20)],
             right=[320+10000/(z+20),100+10000/(z+20)],confirmed=True) for z in (50,80,120)]
        first=calculate(lines)
        project([[333,173]],first['matrix'])
        self.assertEqual(first,calculate(lines))
if __name__=='__main__':unittest.main()
