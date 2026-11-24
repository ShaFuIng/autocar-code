import unittest,json,hashlib
from pathlib import Path
import numpy as np
from junction_fused import retain_segment
class FusedTest(unittest.TestCase):
 def test_partial_support(self):
  m=np.zeros((360,640),np.uint8);m[:,200:205]=1;m[:,435:440]=1;m[230:240,:]=1;p=[[320,y] for y in range(190,335,4)];out=retain_segment(m,p)
  self.assertTrue(out);self.assertGreater(out[0][1],239);self.assertTrue(all(not 230<=y<240 for x,y in out))
 def test_no_support(self):self.assertEqual(retain_segment(np.zeros((360,640),np.uint8),[[320,y] for y in range(190,335,4)]),[])
 def test_baselines_and_bounds(self):
  root=Path(__file__).parent/'data';m=json.loads((root/'experiments/junction-cases/cases.json').read_text(encoding='utf8'));c=m['cases'][0];rep=json.loads((root/'experiments/junction-cases/v6-report.json').read_text(encoding='utf8'))
  self.assertNotIn('fused_job',m['cases'][1])
  for k,v in rep['cases'][0]['frozen_sha256'].items():self.assertEqual(hashlib.sha256((root/'analysis'/c[k]/'result.json').read_bytes()).hexdigest(),v)
  f=json.loads((root/'analysis'/c['corridor_job']/'result.json').read_text(encoding='utf8'));g=json.loads((root/'analysis'/c['fused_job']/'result.json').read_text(encoding='utf8'));self.assertEqual(f['frames'],g['frames'])
  for a,b in zip(f['models']['F'],g['models']['G']):
   self.assertIsNone(b['pid']);self.assertTrue(np.isfinite(np.array(b['A']['path'])).all());self.assertEqual(a['A']['junction']['entry_direction'],b['A']['junction']['entry_direction']);self.assertEqual(a['A']['junction']['state'],b['A']['junction']['state'])
   if a['A']['valid']:self.assertTrue(b['A']['valid'])
if __name__=='__main__':unittest.main()
