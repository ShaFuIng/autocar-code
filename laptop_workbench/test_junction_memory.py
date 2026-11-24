import copy,json,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from junction_memory import JunctionMemory,candidates
import hashlib
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
from analysis_jobs import read
import junction_cases as jc
from test_junction_cases import JunctionReviewsTest

class MemoryTest(unittest.TestCase):
    def test_missing_lines_do_not_assert_junction(self):
        tr=JunctionMemory();tr.entry=np.array([0.,320.]);tr.width=np.array([1.,-100.])
        baseline=dict(path=[],path_samples=[],valid=False,road_segments=[])
        with patch('junction_memory.estimate_motion',return_value=(None,{'valid':False})):
            for i in range(15):r=tr.step(np.zeros((360,640,3),np.uint8),np.zeros((360,640),np.uint8),i*.15,baseline)
        self.assertEqual(r['junction']['state'],'FOLLOW');self.assertFalse(r['valid']);self.assertNotIn('junction',baseline)

    def test_near_only_is_not_an_exit(self):
        mask=np.zeros((360,640),np.uint8);mask[250:335,200:205]=1;mask[250:335,435:440]=1
        good,rejected=candidates(mask,np.array([0.,320.]),np.array([0.,230.]))
        self.assertEqual(good,[]);self.assertTrue(any(c['reason']=='NEAR_ONLY_NOT_EXIT' for c in rejected))

    def test_memory_is_bounded_and_aligned(self):
        root=Path(__file__).parent/'data';m=read(root/'experiments/junction-cases/cases.json');report=read(root/'experiments/junction-cases/v4-report.json')
        for case,summary in zip(m['cases'],report['cases']):
            for key,digest in summary['frozen_sha256'].items():self.assertEqual(sha(root/'analysis'/case[key]/'result.json'),digest)
            data=read(root/'analysis'/case['memory_job']/'result.json')
            for row in data['models']['E']:
                j=row['A']['junction']
                if j['reason']=='STRAIGHT_MEMORY_PREDICTION':
                    self.assertTrue(j['memory_aligned']);self.assertLessEqual(j['memory_age_s'],j['parameters']['memory_hold_s'])
                self.assertTrue(np.isfinite(np.array(row['A']['path'])).all())

    def test_frozen_baselines_and_candidate_contract(self):
        root=Path(__file__).parent/'data';m=read(root/'experiments/junction-cases/cases.json');rep=read(root/'experiments/junction-cases/v4-report.json')
        for case,summary in zip(m['cases'],rep['cases']):
            old=root/'analysis'/case['job'];new=root/'analysis'/case['memory_job'];a=read(old/'result.json');c=read(new/'result.json')
            self.assertEqual(sha(old/'result.json'),summary['baseline_sha256']);self.assertEqual(a['frames'],c['frames']);self.assertEqual(c['cache']['inferred'],0)
            for i,row in enumerate(c['models']['E']):
                self.assertIsNone(row['pid']);r=row['A'];self.assertEqual(r['valid'],bool(r['path']))
                self.assertEqual(sha(new/'E'/f'{i:06d}.png'),sha(old/'A'/f'{i:06d}.png'))
                if r['junction']['reason']=='EXIT_UNCONFIRMED':self.assertEqual(r['path'],[])
                if r['junction']['selected'] is not None:self.assertLessEqual(r['junction']['candidates'][r['junction']['selected']]['points'][0][1],230)
            self.assertEqual(c['models']['E'][-1]['A']['junction']['state'],'FOLLOW')

class CandidateAnnotationTest(JunctionReviewsTest):
    def test_candidate_identity(self):
        case=jc.catalog(self.app)['cases'][0];p=dict(self.p,group='E',memory_job=case['memory_job'])
        saved=jc.save(self.app,p);self.assertEqual(saved['memory_job'],case['memory_job'])
        with self.assertRaises(ValueError):jc.save(self.app,dict(p,id='c'*32,memory_job='old'))

if __name__=='__main__':unittest.main()
