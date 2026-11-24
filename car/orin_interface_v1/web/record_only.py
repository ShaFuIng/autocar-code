"""Camera-only recording. No UART, motion commands or browser lease."""
import json
import shutil
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from manual_drive import SessionRecorder, atomic_json


class RecordOnly:
    def __init__(self, root, recorder=None, clock=time.monotonic):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.recorder = recorder or SessionRecorder(self.root)
        self.clock = clock
        self.lock = threading.RLock()
        self.current = None
        self.last_frame = self.segment_at = 0
        for path in self.root.glob('*/summary.json'):
            try:
                run = json.loads(path.read_text(encoding='utf-8'))
                if run.get('status') == 'recording':
                    run.update(status='interrupted', reason='SERVER_RESTARTED_NO_RESUME')
                    atomic_json(path, run)
            except (OSError, ValueError):
                pass

    def snapshot(self):
        with self.lock:
            return {'ok': True, 'current': dict(self.current) if self.current else None}

    def _save(self):
        atomic_json(self.root / self.current['id'] / 'summary.json', self.current)

    def start(self, camera_ok):
        with self.lock:
            if self.current and self.current['status'] == 'recording':
                return self.snapshot()  # Duplicate start does not overwrite a take.
            if not camera_ok:
                raise ValueError('Camera unavailable; recording not started')
            if shutil.disk_usage(self.root).free < 256 * 1024 * 1024:
                raise ValueError('Less than 256 MiB free disk space')
            run_id = uuid.uuid4().hex
            self.current = {'id': run_id, 'mode': 'record_only', 'status': 'recording',
                            'created_at': datetime.now(timezone.utc).isoformat(),
                            'path': str(self.root / run_id), 'frames': 0,
                            'nominal_fps': 10, 'reason': '', 'motion_labels': False}
            self._save()
            self.recorder.start(run_id)
            self.last_frame = self.segment_at = self.clock()
            return self.snapshot()

    def finish(self, reason='USER_STOP', failed=False):
        with self.lock:
            if self.current and self.current['status'] == 'recording':
                try:
                    self.recorder.stop()
                finally:
                    self.current.update(status='failed' if failed else 'stopped', reason=reason,
                                        ended_at=datetime.now(timezone.utc).isoformat())
                    self._save()
            return self.snapshot()

    def write(self, frame, expected_id=None, capture_at=None):
        with self.lock:
            if not self.current or self.current['status'] != 'recording' or (expected_id is not None and self.current['id']!=expected_id):
                return
            try:
                if self.clock() - self.segment_at >= 300:
                    self.recorder.start(self.current['id'])
                    self.segment_at = self.clock()
                self.recorder.write(frame,expected_id=expected_id,capture_at=capture_at)
                self.last_frame = self.clock()
                self.current['frames'] += 1
                if self.current['frames'] % 10 == 0:
                    self._save()
            except Exception as exc:
                self.finish('RECORDING_ERROR: ' + str(exc), failed=True)

    def check(self):
        with self.lock:
            if not self.current or self.current['status'] != 'recording':
                return
            if self.clock() - self.last_frame > 3:
                self.finish('CAMERA_FRAMES_LOST', failed=True)
            elif shutil.disk_usage(self.root).free < 256 * 1024 * 1024:
                self.finish('DISK_SPACE_LOW', failed=True)
