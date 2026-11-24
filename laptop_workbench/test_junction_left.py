import unittest,json,hashlib
from pathlib import Path
import numpy as np
from junction_left import JunctionLeft
from unittest.mock import patch
class LeftTest(unittest.TestCase):
 def test_blank_does_not_seed(self):
  tr=JunctionLeft()
  with patch('junction_left.estimate_motion',return_value=(None,{'valid':False})):
   out=tr.step(np.zeros((360,640,3),np.uint8),np.zeros((360,640),np.uint8),0,dict(path=[],path_samples=[],valid=False,road_segments=[]))
  self.assertEqual(out['junction']['left_boundary'],[]);self.assertIsNone(tr.lane_width)
 def test_frozen_results_and_width(self):
  root=Path(__file__).parent/'data';exp=root/'experiments/junction-cases';m=json.loads((exp/'cases.json').read_text(encoding='utf8'));c=m['cases'][0];report=json.loads((exp/'v7-report.json').read_text(encoding='utf8'))
  self.assertNotIn('left_job',m['cases'][1])
  for key,digest in report['cases'][0]['frozen_sha256'].items():self.assertEqual(hashlib.sha256((root/'analysis'/c[key]/'result.json').read_bytes()).hexdigest(),digest)
  d=json.loads((root/'analysis'/c['left_job']/'result.json').read_text(encoding='utf8'));rows=d['models']['H'];width=None
  self.assertEqual(rows[103]['A']['junction']['reason'],'LEFT_BOUNDARY_WIDTH_CENTER')
  for row in rows:
   self.assertIsNone(row['pid']);j=row['A']['junction'];diag=j['left_diagnostics'];self.assertTrue(np.isfinite(np.array(row['A']['path'])).all())
   if diag['width_frozen'] and width is not None:self.assertEqual(diag['historical_width'],width)
   width=diag['historical_width']
   if j['left_center']:
    for (lx,y),(cx,cy) in zip(j['left_boundary'],j['left_center']):self.assertAlmostEqual(cx-lx,np.polyval(width,y)/2)
if __name__=='__main__':unittest.main()
