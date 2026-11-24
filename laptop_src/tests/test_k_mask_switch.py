import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'laptop_workbench'))
from junction_k_mask import MaskGate

def d(far=False,opening=False,support=True):return dict(far_branch=far,opening=opening,near_opening=opening,a_supported=support)
class GateTests(unittest.TestCase):
 def test_curve_or_near_branch_alone_does_not_take_over(self):
  g=MaskGate()
  for _ in range(30):self.assertNotEqual(g.step(d(False,True),True,False,True)['state'],'CROSS')
 def test_no_reference_does_not_take_over(self):
  g=MaskGate()
  for _ in range(30):self.assertNotEqual(g.step(d(True,True),False,False,True)['state'],'CROSS')
 def test_persistent_structure_and_reference_take_over(self):
  g=MaskGate()
  for _ in range(5):r=g.step(d(True,True),True,False,True)
  self.assertEqual(r['state'],'CROSS')
 def test_missing_reference_aborts(self):
  g=MaskGate()
  for _ in range(5):g.step(d(True,True),True,False,True)
  self.assertEqual(g.step(d(),False,False,True)['state'],'SUSPECT')
 def test_false_positive_returns_without_straight_reference_agreement(self):
  g=MaskGate()
  for _ in range(5):g.step(d(True,True),True,False,True)
  for _ in range(10):r=g.step(d(),True,False,True)
  self.assertEqual(r['state'],'A')
 def test_distant_opening_with_intact_a_stays_on_a(self):
  g=MaskGate()
  for _ in range(30):
   x=d(True,True);x['near_opening']=False
   self.assertNotEqual(g.step(x,True,True,True)['state'],'CROSS')
 def test_suspect_clears(self):
  g=MaskGate()
  for _ in range(2):g.step(d(True),True,False,True)
  for _ in range(8):r=g.step(d(),True,False,True)
  self.assertEqual(r['state'],'A')
if __name__=='__main__':unittest.main()
