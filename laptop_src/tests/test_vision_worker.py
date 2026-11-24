import unittest,sys,io,json,tempfile,base64
from pathlib import Path
from unittest.mock import patch
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'web'),str(ROOT/'laptop_workbench')]
import live_model_worker as worker
class WorkerLayerTest(unittest.TestCase):
 def test_worker_emits_reusable_mask_and_boundary_provenance(self):
  im=np.zeros((360,640,3),np.uint8);cv2.line(im,(140,359),(270,110),(255,255,255),8);cv2.line(im,(500,359),(370,110),(255,255,255),8)
  class Model:
   def __init__(self,w):pass
   def predict(self,image):return image[:,:,0]>128
  with tempfile.TemporaryDirectory() as t:
   root=Path(t);(root/'weights').write_bytes(b'fixture');(root/'config').write_text(json.dumps({'test':dict(adapter='fake',weights=str(root/'weights'))}))
   request=dict(seq=1,captured=1.,row_ratio=.6,path_algorithm='road-image-v3',jpeg=base64.b64encode(cv2.imencode('.jpg',im)[1]).decode());out=io.StringIO()
   with patch.object(sys,'argv',['worker',str(root/'config'),'test']),patch.object(sys,'stdin',io.StringIO(json.dumps(request)+'\n')),patch.dict(worker.ADAPTERS,{'fake':Model}),patch('sys.stdout',out):worker.main()
   r=json.loads(out.getvalue().splitlines()[-1]);mask=cv2.imdecode(np.frombuffer(base64.b64decode(r['mask_png']),np.uint8),0)
   self.assertEqual(mask.shape,(360,640));self.assertGreater(np.count_nonzero(mask),100);self.assertTrue(r['path_diagnostic']['observed_edges']);self.assertTrue(r['path_diagnostic']['boundaries']);self.assertEqual(r['recognition']['input_size'],640);self.assertEqual(r['seq'],1)
if __name__=='__main__':unittest.main()
