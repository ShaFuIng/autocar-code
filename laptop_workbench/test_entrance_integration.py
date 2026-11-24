"""Only new entry algorithms and path-edit mask migration; no old tests run."""
import copy
import unittest
from unittest.mock import patch
import test_shadow_offline as fixtures
from shadow_offline import specification
import shadow_worker as worker
from entrance_paths import ALGORITHMS

class EntryIntegration(unittest.TestCase):
    setUp=fixtures.ShadowTests.setUp
    tearDown=fixtures.ShadowTests.tearDown
    predictor=fixtures.ShadowTests.predictor
    run_job=fixtures.ShadowTests.run_job
    def test_three_new_algorithms_via_registry_spec(self):
        p=copy.deepcopy(self.payload)
        for g,a in zip(p['groups'],ALGORITHMS):g['algorithm']=a
        spec=specification(self.app,p,6)
        self.assertIsNotNone(spec['groups'][2]['calibration'])
        r=self.run_job('new',spec)
        self.assertEqual(self.calls,6)
        for rows in r['models'].values():
            self.assertEqual(len(rows),6)
            self.assertTrue(all('sample_row_ratio' in x['pid'] for x in rows))
            self.assertTrue(all(not x['pid']['hardware_output'] for x in rows))
    def test_path_code_edit_reuses_verified_inference(self):
        self.run_job('before')
        real_sha=worker.sha
        def changed(p):return 'new-path-only-version' if str(p)==worker.__file__ else real_sha(p)
        with patch.object(worker,'sha',changed):r=self.run_job('after')
        self.assertEqual(r['cache']['inferred'],0)
        self.assertEqual(r['cache']['mask_reused'],6)
        self.assertEqual(self.calls,6)
    def test_requested_row_invalidates_candidate_scoring_only(self):
        p=copy.deepcopy(self.payload);p['groups']=p['groups'][:1]
        p['groups'][0]['algorithm']='entrance-image-v1'
        spec=specification(self.app,p,6);self.run_job('row1',spec)
        spec['groups'][0]['parameters']['row_ratio']=.7
        r=self.run_job('row2',spec)
        self.assertEqual(r['cache']['inferred'],0)
        self.assertEqual(r['cache']['path_reused'],0)

if __name__=='__main__':unittest.main()
