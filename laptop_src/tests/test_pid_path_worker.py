"""CPU-only worker protocol checks: no camera, container, network or actuator."""
import base64
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'web'))
import live_model_worker as worker


class Paths(unittest.TestCase):
    def test_temporal_worker_to_pid_proposals_without_actuators(self):
        """Exercise real worker geometry into PID with synthetic masks only."""
        from visual_pid import PID, DEFAULTS

        class Model:
            def __init__(self, weights): pass
            def predict(self, image): return image[:, :, 0] > 128

        for width, height in ((640, 360), (1280, 720)):
            for shift in (-35, 0, 35):
                image = np.zeros((360, 640, 3), np.uint8)
                cv2.line(image, (140+shift, 359), (270+shift, 110), (255, 255, 255), 8)
                cv2.line(image, (500+shift, 359), (370+shift, 110), (255, 255, 255), 8)
                image = cv2.resize(image, (width, height))
                blank = np.zeros_like(image)
                requests = []
                for seq, frame in enumerate((image, image, blank), 1):
                    requests.append(dict(seq=seq, captured=seq*.2, row_ratio=.75,
                        path_algorithm='temporal-v2',
                        jpeg=base64.b64encode(cv2.imencode('.jpg', frame)[1]).decode()))
                with tempfile.TemporaryDirectory() as folder:
                    root = Path(folder)
                    (root/'weights').write_bytes(b'test-only')
                    (root/'config.json').write_text(json.dumps({
                        'test': {'adapter': 'fake', 'weights': str(root/'weights')}}))
                    output = io.StringIO()
                    with patch.object(sys, 'argv', ['worker', str(root/'config.json'), 'test']), \
                         patch.object(sys, 'stdin', io.StringIO(
                             ''.join(json.dumps(r)+'\n' for r in requests))), \
                         patch.dict(worker.ADAPTERS, {'fake': Model}), \
                         contextlib.redirect_stdout(output):
                        worker.main()
                    results = [json.loads(line) for line in output.getvalue().splitlines()[1:]]
                pid = PID(dict(DEFAULTS, center_us=1360.))
                self.assertTrue(results[0]['valid'])
                self.assertTrue(results[1]['valid'])
                warmup = pid.step_path(.2, results[0]['path'])
                self.assertEqual(warmup['pwm'], 0)
                proposal = pid.step_path(.4, results[1]['path'])
                self.assertTrue(proposal['valid'])
                self.assertFalse(proposal['hardware_output'])
                self.assertLessEqual(abs(proposal['steer_us']-1360), DEFAULTS['steer_rate']*.2)
                if shift:
                    self.assertGreater((proposal['steer_us']-1360)*shift, 0)
                else:
                    self.assertLess(abs(proposal['steer_us']-1360), 2)
                # A stale capture cannot be made fresh by reusing its path.
                stale = pid.step_path(1.2, results[1]['path'])
                self.assertEqual(stale['pwm'], 0)
                self.assertEqual(stale['reason'], 'INVALID_OR_STALE_TIME')
                self.assertFalse(results[2]['valid'])
                self.assertEqual(pid.step_path(1.4, results[2]['path'])['pwm'], 0)

    def test_both_registered_workers_preserve_identity_and_pixels(self):
        image = np.zeros((360, 640, 3), np.uint8)
        cv2.line(image, (140, 359), (270, 110), (255, 255, 255), 8)
        cv2.line(image, (500, 359), (370, 110), (255, 255, 255), 8)
        jpeg = base64.b64encode(cv2.imencode('.jpg', image)[1]).decode()

        class Model:
            def __init__(self, weights): pass
            def predict(self, image): return image[:, :, 0] > 128

        for algorithm in ('legacy9', 'temporal-v2'):
            with tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                (root / 'weights').write_bytes(b'test-only')
                (root / 'config.json').write_text(json.dumps({
                    'test': {'adapter': 'fake', 'weights': str(root / 'weights')}}))
                request = dict(seq=1, jpeg=jpeg, row_ratio=.75,
                               path_algorithm=algorithm, captured=1.)
                output = io.StringIO()
                with patch.object(sys, 'argv', ['worker', str(root / 'config.json'), 'test']), \
                     patch.object(sys, 'stdin', io.StringIO(json.dumps(request)+'\n')), \
                     patch.dict(worker.ADAPTERS, {'fake': Model}), \
                     contextlib.redirect_stdout(output):
                    worker.main()
                result = json.loads(output.getvalue().splitlines()[-1])
                self.assertEqual(result['path_algorithm'], algorithm)
                self.assertEqual(result['seq'], 1)
                self.assertTrue(result['valid'])
                self.assertGreater(len(result['path']), 5)
                self.assertTrue(all(abs(x-640) < 8 for x, y in result['path']))


if __name__ == '__main__':
    unittest.main()
