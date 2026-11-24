import sys,time,threading,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'web'))
from camera_pipeline import CameraPipeline

class PipelineTests(unittest.TestCase):
    def test_slow_recording_and_preview_do_not_block_control(self):
        stop=threading.Event();recording=threading.Event();release=threading.Event()
        class Recorder:
            run_id='test';error=None
            def write(self,frame,**kw):
                recording.set();release.wait(2);return True
        class Pure:current=None
        def preview(*args):release.wait(2)
        p=CameraPipeline(stop,preview,Recorder(),Pure(),encoder=lambda f:b'jpeg')
        try:
            first=time.monotonic();p.publish('frame',first)
            self.assertTrue(recording.wait(1))
            for i in range(12):
                p.publish('frame',time.monotonic());time.sleep(.01)
            self.assertGreater(p.control()[2],8)
            self.assertGreater(p.snapshot()['control_published'],8)
            self.assertLess(p.control()[1]-first,.5)
            self.assertEqual(p.snapshot()['recording_processed'],0)
        finally:stop.set();release.set()

    def test_queue_bounded_without_capture_wait(self):
        stop=threading.Event();release=threading.Event();entered=threading.Event()
        class Recorder:
            run_id='test';error=None
            def write(self,frame,**kw):entered.set();release.wait(2);return True
        class Pure:current=None
        p=CameraPipeline(stop,lambda *a:None,Recorder(),Pure(),encoder=lambda f:b'jpeg')
        try:
            p.publish(0,10);self.assertTrue(entered.wait(1))
            for i in range(30):p.publish(i,11+i)
            self.assertEqual(p.record_queue.qsize(),8)
            self.assertGreater(p.snapshot()['recording_dropped'],0)
        finally:stop.set();release.set()

    def test_recorder_rejects_queued_frame_from_previous_session(self):
        import tempfile
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'laptop_workbench/data/experiments/control-camera-split/payload/web'))
        from manual_drive import SessionRecorder
        with tempfile.TemporaryDirectory() as folder:
            r=SessionRecorder(folder);r.start('new')
            self.assertFalse(r.write(None,expected_id='old',capture_at=1))
            self.assertIsNone(r.writer);r.stop()

if __name__=='__main__':unittest.main()
