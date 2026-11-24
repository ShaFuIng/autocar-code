import unittest,json,hashlib
from pathlib import Path
import numpy as np
from junction_corridor import corridor_support
class CorridorTest(unittest.TestCase):
 def test_no_lines_rejected(self):
  s,p=corridor_support(np.zeros((360,640),np.uint8),[0,320]);self.assertFalse(s['supported'])
 def test_crossing_line_rejected(self):
  m=np.zeros((360,640),np.uint8);m[:,200:205]=1;m[:,435:440]=1
  self.assertTrue(corridor_support(m,[0,320])[0]['supported']);m[250:255,:]=1
  self.assertFalse(corridor_support(m,[0,320])[0]['supported'])
 def test_frozen_and_output(self):
  root=Path(__file__).parent/'data';exp=root/'experiments/junction-cases';m=json.loads((exp/'cases.json').read_text(encoding='utf8'));c=m['cases'][0];report=json.loads((exp/'v5-report.json').read_text(encoding='utf8'))
  self.assertNotIn('corridor_job',m['cases'][1])
  for key,digest in report['cases'][0]['frozen_sha256'].items():self.assertEqual(hashlib.sha256((root/'analysis'/c[key]/'result.json').read_bytes()).hexdigest(),digest)
  e=json.loads((root/'analysis'/c['memory_job']/'result.json').read_text(encoding='utf8'));f=json.loads((root/'analysis'/c['corridor_job']/'result.json').read_text(encoding='utf8'))
  self.assertEqual(e['frames'],f['frames']);self.assertTrue(f['models']['F'][100]['A']['valid'])
  for a,b in zip(e['models']['E'],f['models']['F']):
   self.assertEqual(a['A']['junction']['entry_direction'],b['A']['junction']['entry_direction']);self.assertEqual(a['A']['junction']['state'],b['A']['junction']['state']);self.assertIsNone(b['pid'])
   if a['A']['valid']:self.assertEqual(a['A']['path'],b['A']['path'])
if __name__=='__main__':unittest.main()
