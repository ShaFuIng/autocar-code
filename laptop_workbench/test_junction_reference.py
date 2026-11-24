import unittest,json,hashlib
from pathlib import Path
import numpy as np
from junction_reference import observation_fit
class ReferenceTest(unittest.TestCase):
 def test_branch_downweighted(self):
  points=[[320,y] for y in range(180,251,4)]+[[320-(y-250)*2,y] for y in range(254,315,4)]
  fit,good,bad=observation_fit(points);self.assertTrue(bad);raw=np.polyfit(np.array(points)[:,1],np.array(points)[:,0],1);self.assertLess(abs(np.polyval(fit,334)-320),abs(np.polyval(raw,334)-320)*.1)
 def test_short_fragment_not_anchor(self):self.assertIsNone(observation_fit([[320,y] for y in range(190,210,4)])[0])
 def test_baseline_and_focus(self):
  root=Path(__file__).parent/'data';exp=root/'experiments/junction-cases';m=json.loads((exp/'cases.json').read_text(encoding='utf8'));c=m['cases'][0];rep=json.loads((exp/'v8-report.json').read_text(encoding='utf8'));self.assertNotIn('reference_job',m['cases'][1])
  for key,digest in rep['cases'][0]['frozen_sha256'].items():self.assertEqual(hashlib.sha256((root/'analysis'/c[key]/'result.json').read_bytes()).hexdigest(),digest)
  d=json.loads((root/'analysis'/c['reference_job']/'result.json').read_text(encoding='utf8'))
  for i in [96,99,105]:
   row=d['models']['I'][i];self.assertIsNone(row['pid']);p=np.array(row['A']['junction']['entry_direction']);self.assertEqual(p[:,1].min(),180);self.assertEqual(p[:,1].max(),334);self.assertLess(max(abs(p[:,0]-np.polyval(np.polyfit(p[:,1],p[:,0],1),p[:,1]))),1e-6)
if __name__=='__main__':unittest.main()
