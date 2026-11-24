import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'web'))
from record_only import RecordOnly


class Writer:
    def __init__(self): self.started=[]; self.frames=0; self.stops=0
    def start(self, run): self.started.append(run)
    def write(self, frame): self.frames+=1
    def stop(self): self.stops+=1


class RecordingTests(unittest.TestCase):
    def test_no_heartbeat_idempotence_and_stop(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            now=[0];writer=Writer();r=RecordOnly(folder,writer,clock=lambda:now[0])
            first=r.start(True)['current']['id']
            self.assertEqual(r.start(True)['current']['id'],first)
            for _ in range(400):
                now[0]+=1;r.write(None);r.check()
            self.assertEqual(r.snapshot()['current']['status'],'recording')
            self.assertEqual(len(writer.started),2)  # five-minute segment rollover
            r.finish();r.finish();r.write(None)
            self.assertEqual(writer.frames,400)
            self.assertEqual(writer.stops,1)
            self.assertEqual(r.snapshot()['current']['status'],'stopped')

    def test_camera_loss_and_restart_do_not_resume(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            now=[0];r=RecordOnly(folder,Writer(),clock=lambda:now[0])
            with self.assertRaises(ValueError):r.start(False)
            r.start(True);now[0]=4;r.check()
            self.assertEqual(r.snapshot()['current']['reason'],'CAMERA_FRAMES_LOST')
            run=r.start(True)['current'];r2=RecordOnly(folder,Writer())
            self.assertIsNone(r2.snapshot()['current'])
            self.assertEqual(json.loads((Path(run['path'])/'summary.json').read_text(encoding='utf-8'))['status'],'interrupted')

    def test_disk_and_writer_failure(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            writer=Writer();r=RecordOnly(folder,writer)
            with patch('record_only.shutil.disk_usage') as disk:
                disk.return_value.free=0
                with self.assertRaises(ValueError):r.start(True)
            r.start(True)
            with patch.object(writer,'write',side_effect=RuntimeError('codec failed')):r.write(None)
            self.assertEqual(r.snapshot()['current']['status'],'failed')
            self.assertEqual(writer.stops,1)


if __name__=='__main__':unittest.main()
