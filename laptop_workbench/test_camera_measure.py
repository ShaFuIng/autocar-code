import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
from camera_measure import CameraMeasure

class CameraMeasureTests(unittest.TestCase):
    def test_capture_preserves_bytes_and_only_calls_camera(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as temp:
            orin=Mock();orin.call.return_value=b'original-camera-jpeg'
            app=SimpleNamespace(store=SimpleNamespace(root=Path(temp)),orin=orin)
            task=CameraMeasure(app)
            def worker(args,**kwargs):
                folder=Path(args[-1])
                metadata=json.loads((folder/'capture.json').read_text(encoding='utf8'))
                (folder/'result.json').write_text(json.dumps(metadata),encoding='utf8')
                return SimpleNamespace(returncode=0)
            with patch('camera_measure.subprocess.run',side_effect=worker):task.run('test')
            self.assertEqual(task.status()['state'],'completed')
            self.assertFalse(task.status()['hardware_output'])
            self.assertEqual((task.root/'test/raw.jpg').read_bytes(),b'original-camera-jpeg')
            orin.call.assert_called_once_with('camera')
    def test_failure_not_reported_as_complete(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as temp:
            orin=Mock();orin.call.side_effect=ValueError('offline')
            task=CameraMeasure(SimpleNamespace(store=SimpleNamespace(root=Path(temp)),orin=orin))
            task.run('failed')
            self.assertEqual(task.status()['state'],'failed')
    def test_running_request_does_not_duplicate(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as temp:
            task=CameraMeasure(SimpleNamespace(store=SimpleNamespace(root=Path(temp))))
            task.state={'state':'running','id':'existing'}
            self.assertEqual(task.start()['id'],'existing')
if __name__=='__main__':unittest.main()

