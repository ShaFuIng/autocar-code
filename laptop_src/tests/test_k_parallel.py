import sys,unittest,threading,importlib.util,time,json,io,tempfile,base64,types
from pathlib import Path
import numpy as np,cv2
ROOT=Path(__file__).resolve().parents[1]
PAYLOAD=ROOT/'laptop_workbench/data/experiments/k-parallel/payload/web'
sys.path.insert(0,str(ROOT/'web'))
class Tests(unittest.TestCase):
 def test_control_continues_while_preview_blocked(self):
  spec=importlib.util.spec_from_file_location('latency_worker',PAYLOAD/'live_model_worker.py');w=importlib.util.module_from_spec(spec);spec.loader.exec_module(w)
  entered=threading.Event();release=threading.Event();output=[]
  def render(*args):entered.set();release.wait(3);return 'jpeg'
  w.render_preview=render;w.emit=output.append
  class Model:
   def __init__(self,*a):pass
   def predict(self,im):return np.zeros(im.shape[:2],bool)
  self_outer=self
  class K:
   def __init__(self,row):self.row=row
   def step(self,*a,on_a=None):
    r=dict(path=[[640,400]],quality=1,valid=True,path_diagnostic={})
    on_a(r)
    self_outer.assertTrue(output and output[-1]['kind']=='control')
    return r
   def close(self):pass
  w.ADAPTERS={'fake':Model};oldmodule=sys.modules.get('k_lane_live_path');sys.modules['k_lane_live_path']=types.SimpleNamespace(KLaneLivePath=K)
  oldargv,oldin=sys.argv,sys.stdin
  with tempfile.TemporaryDirectory() as temp:
   p=Path(temp);(p/'weights').write_bytes(b'test');(p/'config').write_text(json.dumps({'model':dict(adapter='fake',weights=str(p/'weights'))}))
   encoded=base64.b64encode(cv2.imencode('.jpg',np.zeros((360,640,3),np.uint8))[1]).decode()
   sys.argv=['worker',str(p/'config'),'model'];sys.stdin=io.StringIO(''.join(json.dumps(dict(seq=i,jpeg=encoded,row_ratio=.6,path_algorithm='junction-k-lane-v3',captured=i))+'\n' for i in range(3)))
   try:
    w.main();self.assertTrue(entered.wait(1));self.assertEqual([x['seq'] for x in output if x['kind']=='control'],[0,1,2]);self.assertFalse(any(x['kind']=='preview' for x in output))
   finally:
    release.set();sys.argv,sys.stdin=oldargv,oldin
    if oldmodule is None:sys.modules.pop('k_lane_live_path',None)
    else:sys.modules['k_lane_live_path']=oldmodule
if __name__=='__main__':unittest.main()
