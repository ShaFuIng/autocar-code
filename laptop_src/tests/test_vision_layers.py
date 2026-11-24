import base64,json,sys,tempfile,threading,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'web'),str(ROOT/'laptop_workbench')]
from vision_archive import VisionArchive
from vision_evidence import evidence,index
import test_shadow_offline as fixtures
import shadow_worker as worker

class ArchiveTests(unittest.TestCase):
 def test_exact_raw_mask_metadata_and_index(self):
  with tempfile.TemporaryDirectory() as t:
   folder=Path(t);a=VisionArchive(folder);raw=cv2.imencode('.jpg',np.zeros((10,20,3),np.uint8))[1].tobytes();mask=cv2.imencode('.png',np.ones((10,20),np.uint8)*255)[1].tobytes()
   meta=dict(seq=7,capture_at=12.,captured_wall=1000.,path=[[640,400],[650,600]],path_diagnostic=dict(observed_edges=[[[10,20],[30,40]]]),weights_sha256='fixture',sample_row_ratio=.7)
   self.assertTrue(a.submit(raw,base64.b64encode(mask).decode(),meta));a.close();a.thread.join(3)
   self.assertEqual(a.saved,1);rows=evidence(folder);self.assertEqual(len(rows),1);self.assertEqual(rows[0]['seq'],7)
   app=SimpleNamespace(assets={},replays={'orin-manual-fixture':dict(pid_events=[],events=[])})
   key=index(app,folder,'orin-manual-fixture','v1');self.assertTrue(app.replays[key]['rows'][0]['exact_evidence']);self.assertEqual((folder/'vision-000007.jpg').read_bytes(),raw)
   (folder/'vision-000007.png').write_bytes(b'corrupt');self.assertEqual(evidence(folder),[])
 def test_full_queue_does_not_wait(self):
  with patch('vision_archive.threading.Thread.start'):
   a=VisionArchive(Path('.'),capacity=1)
  self.assertTrue(a.submit(b'a','AA==',{'seq':1}));self.assertFalse(a.submit(b'b','AA==',{'seq':2}));self.assertEqual(a.dropped,1)
 def test_disk_failure_is_diagnostic_not_control_exception(self):
  with tempfile.TemporaryDirectory() as t:
   a=VisionArchive(Path(t)/'missing');self.assertTrue(a.submit(b'a','AA==',{'seq':1}));a.close();a.thread.join(3);self.assertEqual(a.dropped,1);self.assertIsNotNone(a.error)
 def test_old_record_has_no_fabricated_mask(self):
  with tempfile.TemporaryDirectory() as t:self.assertEqual(evidence(Path(t)),[])

class LiveReuseTests(unittest.TestCase):
 setUp=fixtures.ShadowTests.setUp
 tearDown=fixtures.ShadowTests.tearDown
 predictor=fixtures.ShadowTests.predictor
 run_job=fixtures.ShadowTests.run_job
 def prepare(self,mismatch=None):
  for i,r in enumerate(self.inputs):
   im=worker.picture(r['image']);mask=(im[:,:,0]>128).astype(np.uint8)*255;file=self.root/f'mask-{i}.png';cv2.imencode('.png',mask)[1].tofile(str(file))
   r['live_evidence']=dict(mask=str(file),mask_sha256=worker.sha(file),raw_sha256=worker.sha(r['image']),weights_sha256=worker.sha(self.weight),recognition=dict(input_size=640,contract='v4-semantic-class1-nearest-v1'))
   if mismatch=='weights':r['live_evidence']['weights_sha256']='wrong'
   if mismatch=='image':r['live_evidence']['raw_sha256']='wrong'
   if mismatch=='size':r['live_evidence']['recognition']['input_size']=960
 def test_exact_live_masks_reused_no_model(self):
  self.prepare();r=self.run_job('exact');self.assertEqual(self.calls,0);self.assertEqual(r['cache']['live_mask_reused'],6)
 def test_different_weights_reinfer(self):
  self.prepare('weights');r=self.run_job('weights');self.assertEqual(self.calls,6);self.assertEqual(r['cache']['live_mask_reused'],0)
 def test_different_raw_does_not_borrow_mask(self):
  self.prepare('image');r=self.run_job('raw');self.assertEqual(self.calls,6)
 def test_different_input_size_reinfer(self):
  self.prepare('size');r=self.run_job('size');self.assertEqual(self.calls,6)

if __name__=='__main__':unittest.main()
