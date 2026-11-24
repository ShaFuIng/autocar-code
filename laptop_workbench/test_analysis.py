import unittest,tempfile,json
from pathlib import Path
from types import SimpleNamespace
from analysis_jobs import Analysis,write
from server import Store
class AnalysisTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent/"data");self.store=Store(Path(self.tmp.name));self.a=Analysis(SimpleNamespace(store=self.store));self.uid='a'*32;f=self.a.root/self.uid;f.mkdir();write(f/'result.json',dict(source='fixture',frames=[{},{}],models={'v4-640':[]}))
 def tearDown(self):self.store.db.close();self.tmp.cleanup()
 def review(self,**kw):return dict(job=self.uid,start=0,end=1,model='v4-640',target='C',verdict='未確認',notes='test',**kw)
 def test_persistence_and_append(self):
  self.a.review(self.review());self.a.review(self.review());self.assertEqual(len(self.a.reviews(self.uid)),2)
  self.assertEqual(len(Analysis(SimpleNamespace(store=self.store)).reviews(self.uid)),2)
 def test_reject_bad_interval(self):
  p=self.review();p['end']=2
  with self.assertRaises(ValueError):self.a.review(p)
 def test_reject_model_and_target(self):
  for key,value in [('model','unknown'),('target','motor'),('notes','')]:
   p=self.review();p[key]=value
   with self.assertRaises(ValueError):self.a.review(p)
 def test_traversal_and_unknown_job(self):
  with self.assertRaises(ValueError):self.a.result('../outside')
 def test_reject_empty_or_unknown_models(self):
  for models in [[],['notinstalled'],['v4-640','v4-640']]:
   with self.assertRaises(ValueError):self.a.start(dict(source='unknown',models=models))
if __name__=='__main__':unittest.main()

