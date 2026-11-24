import copy, json, tempfile, unittest
from pathlib import Path
from types import SimpleNamespace
from server import Store, ROOT, App, handler, ThreadingHTTPServer
import junction_cases as jc

class JunctionReviewsTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.root=Path(self.tmp.name)
        out=self.root/'experiments/junction-cases';out.mkdir(parents=True)
        (out/'cases.json').write_bytes((ROOT/'data/experiments/junction-cases/cases.json').read_bytes())
        self.app=SimpleNamespace(store=Store(self.root),analysis=SimpleNamespace(result=lambda job:json.loads((ROOT/'data/analysis'/job/'result.json').read_text(encoding='utf8'))))
        case=jc.catalog(self.app)['cases'][0]
        self.p=dict(id='a'*32,case='first',job=case['job'],start=0,end=170,anchor=60,stage='穿越',issue='選錯出口',group='B',intent='straight',visibility='visible',polygon=[[.4,.4],[.6,.4],[.6,.6]],notes='僅測試資料')
    def tearDown(self):self.app.store.db.close();self.tmp.cleanup()
    def test_append_reload_and_idempotence(self):
        first=jc.save(self.app,self.p);self.assertFalse(first['training_truth']);self.assertEqual(first['source_video_frame'],710)
        self.assertEqual(jc.save(self.app,self.p),first)
        p=copy.deepcopy(self.p);p.update(id='b'*32,issue='未見問題');jc.save(self.app,p)
        self.app.store.db.close();self.app.store=Store(self.root)
        self.assertEqual(len(jc.reviews(self.app,'first')),2)
        self.assertEqual(jc.reviews(self.app,'first')[0]['issue'],'選錯出口')
        self.assertEqual(jc.reviews(self.app,'second'),[])
    def test_reject_invalid_annotations(self):
        cases=[dict(anchor=171),dict(start=61),dict(end=-1),dict(job='bad'),dict(polygon=[[0,0],[1,1]]),dict(polygon=[[0,0],[.5,.5],[1,1]]),dict(polygon=[[0,0],[float('nan'),.4],[1,1]]),dict(polygon=[[0,0],[2,.4],[1,1]]),dict(visibility='unclear'),dict(stage='fake'),dict(group='C')]
        for changes in cases:
            with self.subTest(changes=changes),self.assertRaises(ValueError):jc.save(self.app,dict(self.p,**changes))
        self.assertEqual(jc.reviews(self.app,'first'),[])
    def test_unclear_exit_and_duplicate_conflict(self):
        p=dict(self.p,visibility='unclear',polygon=[]);jc.save(self.app,p)
        with self.assertRaises(ValueError):jc.save(self.app,dict(p,notes='different'))
        self.assertEqual(len(jc.reviews(self.app,'first')),1)
    def test_both_results_and_assets(self):
        for case in jc.catalog(self.app)['cases']:
            r=self.app.analysis.result(case['job']);self.assertEqual(len(r['frames']),case['frames'])
            self.assertEqual(r['frames'][0]['player_s'],case['player_range_s'][0]);self.assertEqual(r['frames'][-1]['player_s'],case['player_range_s'][1])
            for g in r['spec']['groups']:self.assertFalse(g['pid_enabled'])
            for i,f in enumerate(r['frames']):
                self.assertEqual(f['source_video_frame'],case['source_frame_range'][0]+i)
                for name,ext in [('raw','jpg'),('A','png'),('B','png')]:self.assertTrue((ROOT/'data/analysis'/case['job']/name/f'{i:06d}.{ext}').is_file())
                for k in ['A','B']:self.assertIsNone(r['models'][k][i]['pid'])

    def test_http_save_read_and_origin_guard(self):
        import threading, urllib.request, urllib.error
        self.app.get=lambda path:App.get(self.app,path)
        self.app.post=lambda path,p:App.post(self.app,path,p)
        http=ThreadingHTTPServer(('127.0.0.1',0),handler(self.app))
        thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
        base='http://127.0.0.1:'+str(http.server_address[1])
        try:
            payload=json.dumps(self.p).encode()
            req=urllib.request.Request(base+'/api/junction/review',data=payload,headers={'X-Workbench':'1','Content-Type':'application/json','Origin':base})
            with urllib.request.urlopen(req) as r:self.assertEqual(json.load(r)['source_video_frame'],710)
            with urllib.request.urlopen(base+'/api/junction/reviews/first') as r:self.assertEqual(len(json.load(r)),1)
            bad=urllib.request.Request(base+'/api/junction/review',data=payload,headers={'X-Workbench':'1','Origin':'http://other.invalid'})
            with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(bad)
            self.assertEqual(caught.exception.code,403)
        finally:http.shutdown();http.server_close();thread.join()

if __name__=='__main__':unittest.main()
