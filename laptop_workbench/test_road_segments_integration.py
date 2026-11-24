import copy
import unittest
import test_shadow_offline as fixtures
from shadow_offline import specification
from road_segments_v3 import ALGORITHMS

class RoadIntegration(unittest.TestCase):
    setUp=fixtures.ShadowTests.setUp
    tearDown=fixtures.ShadowTests.tearDown
    predictor=fixtures.ShadowTests.predictor
    run_job=fixtures.ShadowTests.run_job
    def test_segment_registry_and_result_contract(self):
        p=copy.deepcopy(self.payload)
        for g,a in zip(p['groups'],ALGORITHMS):g['algorithm']=a
        spec=specification(self.app,p,6)
        r=self.run_job('v3',spec)
        self.assertEqual(self.calls,6)
        self.assertIsNotNone(spec['groups'][2]['calibration'])
        for rows in r['models'].values():
            for row in rows:
                self.assertIn('observed_edges',row['A'])
                self.assertIn('road_segments',row['A'])
                self.assertIn('path_samples',row['A'])
                self.assertIn('coverage',row['A'])
                self.assertIn('rejections',row['A'])
                self.assertIn('sample_row_ratio',row['pid'])
                self.assertFalse(row['pid']['hardware_output'])

if __name__=='__main__':unittest.main()
