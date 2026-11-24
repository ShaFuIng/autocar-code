"""Real LivePID loop + real temporal geometry/PID, exclusively in-memory I/O.

Running candidate fixtures are seeded explicitly to inspect the future control
branch. No registry flag is changed and no real start endpoint is called.
"""
import base64
import copy
import io
import json
import queue
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'web'), str(ROOT/'laptop_workbench')]
from live_pid import LivePID
from pid_paths import PATHS
from temporal_lane import TemporalLane
from visual_pid import DEFAULTS, PID

EVIDENCE = []

class Sink:
    def __init__(self, root):
        self.root = root
        self.commands = []
        self.stops = []
        self.active = True
        self.reject = False

    def profile(self):
        return dict(center_us=1360, min_us=940, max_us=1900)

    def busy(self):
        return self.active

    def pid_input(self, command):
        if self.reject:
            raise ValueError('FAKE_SINK_REJECTED')
        if not self.active:
            raise AssertionError('Command after stop')
        self.commands.append(copy.deepcopy(command))

    def request_stop(self, reason):
        self.active = False
        self.stops.append(reason)


class StreamTests(unittest.TestCase):
    def exercise(self, fault, algorithm='temporal-v2'):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            clock = [100.]
            with patch('live_pid.threading.Thread.start'):
                sink = Sink(root)
                live = LivePID(sink, None, threading.RLock(), lambda: False)
            live.model_id = 'v4-default'
            live.path_algorithm = algorithm
            live.row_ratio = .6 if algorithm=='calibrated-v7' else .75
            live.last_preview_client = clock[0]
            live.browser_at = clock[0]
            params = dict(DEFAULTS, center_us=1360.,row_ratio=live.row_ratio)
            live.controller = PID(params)
            live.owner = 'a'*32
            live.run = dict(id='fake', state='running', deadline=120.,
                            execution_mode='orin_timed', parameters=params,
                            weights_sha256='fixture')
            (root/'fake').mkdir()
            live.log = (root/'fake/pid.jsonl').open('w', encoding='utf-8')
            live.latest = dict(capture_at=100., weights_sha256='fixture')
            output = queue.Queue()
            output.put(json.dumps(dict(ready=True, protocol=1,
                                       model_id=live.model_id, weights_sha256='fixture')))
            tracker = TemporalLane()
            image = np.zeros((360, 640, 3), np.uint8)
            cv2.line(image, (170, 359), (300, 110), (255, 255, 255), 8)
            cv2.line(image, (530, 359), (400, 110), (255, 255, 255), 8)
            if algorithm=='calibrated-v7':
                from test_calibrated_pid import scene
                from calibrated_path import CalibratedPath
                image,_=scene(shift=5)
                tracker=CalibratedPath()
                for t in (99.5,99.65,99.8):tracker.step(image,image[:,:,0]>128,t)
                live.run['calibration_sha256']=tracker.sha
            jpeg = cv2.imencode('.jpg', image)[1].tobytes()
            sent = []
            calls = [0]
            crashed = [False]
            silent = [False]

            def camera():
                calls[0] += 1
                clock[0] += .15
                if calls[0] > 6:
                    live.worker_stop.set()
                    return jpeg, clock[0], calls[0]-1
                if calls[0] >= 4:
                    if fault == 'camera_missing':
                        return b'', clock[0], calls[0]
                    if fault == 'camera_stale':
                        return jpeg, clock[0]-1., calls[0]
                    if fault == 'future_capture':
                        return jpeg, clock[0]+1., calls[0]
                    if fault == 'nan_capture':
                        return jpeg, float('nan'), calls[0]
                    if fault == 'repeated_capture':
                        return jpeg, 100.45, calls[0]
                    if fault == 'backward_frame':
                        return jpeg, clock[0], 1
                    if fault == 'duplicate_frame':
                        live.check_safety()
                        return jpeg, clock[0], 3
                    if fault == 'browser_lost':
                        live.run['execution_mode'] = 'browser_lease'
                        live.browser_at = clock[0]-2.
                        live.check_safety()
                    if fault == 'deadline':
                        live.run['deadline'] = clock[0]
                        live.check_safety()
                    if fault == 'sink_stopped':
                        sink.active = False
                        live.check_safety()
                return jpeg, clock[0], calls[0]

            class Pipe:
                def write(self, line):
                    p = json.loads(line)
                    sent.append(p['seq'])
                    mask = image[:, :, 0] > 128
                    if len(sent) >= 4 and fault == 'blank':
                        mask = mask & False
                    r = tracker.step(image, mask, p['captured'])
                    result = dict(protocol=1, seq=p['seq'], model_id=live.model_id,
                                  weights_sha256='fixture', path_algorithm=algorithm,
                                  valid=r['valid'], quality=1. if r['valid'] else 0.,
                                  path=r['path'] if algorithm=='calibrated-v7' else [[x*2, y*2] for x, y in r['image_path']],
                                  preview=base64.b64encode(jpeg).decode())
                    if algorithm=='calibrated-v7':
                        result['path_diagnostic']={'calibration_sha256':tracker.sha}
                        if len(sent)>=4 and fault=='calibration':
                            result['path_diagnostic']['calibration_sha256']='changed'
                    if len(sent) >= 4:
                        if fault == 'seq': result['seq'] -= 1
                        if fault == 'protocol': result['protocol'] = 2
                        if fault == 'model': result['model_id'] = 'wrong'
                        if fault == 'algorithm': result['path_algorithm'] = 'legacy9'
                        if fault == 'weights': result['weights_sha256'] = 'changed'
                        if fault == 'invalid': result['valid'] = False
                        if fault == 'low_quality':
                            result.update(valid=False, quality=0.)
                        if fault == 'nan_path': result['path'][0][0] = float('nan')
                        if fault == 'missing_row': result['path'] = [[640, 100], [640, 200]]
                        if fault == 'inference_delay': clock[0] += .45
                        if fault == 'sink_reject': sink.reject = True
                        if fault == 'crash': crashed[0] = True; return
                        if fault == 'timeout': silent[0] = True; return
                    output.put(json.dumps(result))

                def flush(self): pass
                def close(self): pass

            class Process:
                stdin = Pipe()

                @property
                def stdout(self):
                    while True:
                        row = output.get(timeout=5)
                        if row is None: return
                        yield row

                def poll(self):
                    if silent[0]: clock[0] += .7
                    return 1 if crashed[0] else None

                def terminate(self):
                    output.put(None)

            live.camera = camera
            with patch('live_pid.subprocess.Popen', return_value=Process()), \
                 patch('live_pid.subprocess.run'), \
                 patch('live_pid.os.getuid', return_value=0, create=True), \
                 patch('live_pid.os.getgid', return_value=0, create=True), \
                 patch('live_pid.time.monotonic', side_effect=lambda: clock[0]), \
                 patch('live_pid.time.sleep', side_effect=lambda dt: clock.__setitem__(0, clock[0]+dt)):
                live._loop()
                if live.run['state'] == 'running':
                    live.end('FIXTURE_END')
            rows = [json.loads(x) for x in (root/'fake/pid.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertFalse(PATHS['temporal-v2']['control'])
            self.assertEqual(sink.commands[0]['pwm'], 0)
            self.assertTrue(any(x['pwm'] > 0 for x in sink.commands[:3]))
            self.assertTrue(all(940 <= x['target_us'] <= 1900 for x in sink.commands))
            self.assertEqual(len(sink.stops), 1)
            if fault != 'healthy':
                self.assertEqual(len(sink.commands), 3, fault)
                self.assertNotEqual(live.run['reason'], 'FIXTURE_END', fault)
                self.assertTrue(any(x['kind'] == 'stop_diagnostic' for x in rows))
            if fault == 'duplicate_frame':
                self.assertEqual(sent, [1, 2, 3])
                self.assertEqual(live.run['reason'], 'PID_VISION_STALE')
            evidence = dict(case=fault, algorithm=algorithm, fake_commands=len(sink.commands),
                            commands=sink.commands, stop_reason=live.run['reason'],
                            error=live.error, hardware_output=False)
            EVIDENCE.append(evidence)
            return evidence


CASES = ('healthy', 'seq', 'model', 'algorithm', 'weights', 'invalid', 'blank',
         'nan_path', 'missing_row', 'inference_delay', 'crash', 'timeout',
         'camera_missing', 'camera_stale', 'duplicate_frame', 'backward_frame',
         'future_capture', 'nan_capture', 'sink_reject', 'sink_stopped',
         'browser_lost', 'deadline', 'protocol', 'low_quality', 'repeated_capture')

for case in CASES:
    setattr(StreamTests, 'test_'+case, lambda self, case=case: self.exercise(case))

if __name__ == '__main__':
    unittest.main()
