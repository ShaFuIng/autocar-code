import copy,json,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from junction_tracked import JunctionTracked,candidates
import hashlib
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
from analysis_jobs import read
import junction_cases as jc
from test_junction_cases import JunctionReviewsTest

class TrackedTest(unittest.TestCase):
    def test_missing_lines_do_not_assert_junction(self):
        tr=JunctionTracked();tr.entry=np.array([0.,320.]);tr.width=np.array([1.,-100.])
        baseline=dict(path=[],path_samples=[],valid=False,road_segments=[])
        with patch('junction_tracked.estimate_motion',return_value=(None,{'valid':False})):
            for i in range(15):r=tr.step(np.zeros((360,640,3),np.uint8),np.zeros((360,640),np.uint8),i*.15,baseline)
        self.assertEqual(r['junction']['state'],'FOLLOW');self.assertFalse(r['valid']);self.assertNotIn('junction',baseline)

    def test_near_only_is_not_an_exit(self):
        mask=np.zeros((360,640),np.uint8);mask[250:335,200:205]=1;mask[250:335,435:440]=1
        good,rejected=candidates(mask,np.array([0.,320.]),np.array([0.,230.]))
        self.assertEqual(good,[]);self.assertTrue(any(c['reason']=='NEAR_ONLY_NOT_EXIT' for c in rejected))

    def test_frozen_baselines_and_candidate_contract(self):
        root=Path(__file__).parent/'data';m=read(root/'experiments/junction-cases/cases.json');rep=read(root/'experiments/junction-cases/v2-report.json')
        for case,summary in zip(m['cases'],rep['cases']):
            old=root/'analysis'/case['job'];new=root/'analysis'/case['candidate_job'];a=read(old/'result.json');c=read(new/'result.json')
            self.assertEqual(sha(old/'result.json'),summary['baseline_sha256']);self.assertEqual(a['frames'],c['frames']);self.assertEqual(c['cache']['inferred'],0)
            for i,row in enumerate(c['models']['C']):
                self.assertIsNone(row['pid']);r=row['A'];self.assertEqual(r['valid'],bool(r['path']))
                self.assertEqual(sha(new/'C'/f'{i:06d}.png'),sha(old/'A'/f'{i:06d}.png'))
                if r['junction']['reason']=='EXIT_UNCONFIRMED':self.assertEqual(r['path'],[])
                if r['junction']['selected'] is not None:self.assertLessEqual(r['junction']['candidates'][r['junction']['selected']]['points'][0][1],230)
            self.assertEqual(c['models']['C'][-1]['A']['junction']['state'],'FOLLOW')

class CandidateAnnotationTest(JunctionReviewsTest):
    def test_candidate_identity(self):
        case=jc.catalog(self.app)['cases'][0];p=dict(self.p,group='C',candidate_job=case['candidate_job'])
        saved=jc.save(self.app,p);self.assertEqual(saved['candidate_job'],case['candidate_job'])
        with self.assertRaises(ValueError):jc.save(self.app,dict(p,id='c'*32,candidate_job='old'))

if __name__=='__main__':unittest.main()
