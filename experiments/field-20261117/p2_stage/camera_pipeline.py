"""Latest-frame control/preview channels and bounded background recording.

No UART or control commands. Capture timestamps survive every stage unchanged.
"""
import queue
import threading
import time


class CameraPipeline:
    def __init__(self, stop, preview, recorder, record_only, encoder=None):
        self.stop = stop
        self.preview = preview
        self.recorder = recorder
        self.record_only = record_only
        self.encoder = encoder or self.encode
        self.condition = threading.Condition()
        self.latest = None
        self.control_latest = (None, 0., 0)
        self.sequence = 0
        self.record_queue = queue.Queue(maxsize=8)
        self.stats = dict(version=1, captured=0, control_published=0,
                          preview_published=0, recording_submitted=0,
                          recording_processed=0, recording_dropped=0,
                          recording_stale_session=0, recording_errors=0,
                          recording_write_max_ms=0., control_publish_max_gap_s=0.)
        self.last_record = self.last_control = 0.
        for target in (self.control_worker, self.preview_worker, self.record_worker):
            threading.Thread(target=target, daemon=True).start()

    @staticmethod
    def encode(frame):
        import cv2
        ok, encoded = cv2.imencode('.jpg', cv2.resize(frame, (640, 360)),
                                   [cv2.IMWRITE_JPEG_QUALITY, 60])
        if not ok:
            raise ValueError('Camera JPEG encode failed')
        return encoded.tobytes()

    def publish(self, frame, captured):
        # No encoding, filesystem calls, recorder locks, or queue waits here.
        with self.condition:
            self.sequence += 1
            self.latest = (frame, captured, self.sequence)
            self.stats['captured'] += 1
            self.condition.notify_all()
        if captured - self.last_record < .1:
            return
        self.last_record = captured
        manual_id = self.recorder.run_id
        run = self.record_only.current
        pure_id = run['id'] if run and run['status'] == 'recording' else None
        if manual_id is None and pure_id is None:
            return
        try:
            self.record_queue.put_nowait((frame, captured, manual_id, pure_id))
            self.stats['recording_submitted'] += 1
        except queue.Full:
            self.stats['recording_dropped'] += 1

    def next_frame(self, previous):
        with self.condition:
            self.condition.wait_for(lambda: self.stop.is_set() or
                                    (self.latest and self.latest[2] != previous), .1)
            return self.latest

    def wait_control(self, previous, timeout):   # pipeline-ahead-r1 (live_pid pipeline2.json event_wait): block until a control frame other than previous
        with self.condition:
            self.condition.wait_for(lambda: self.stop.is_set() or (self.control_latest[0] is not None and self.control_latest[2] != previous), timeout)
            return self.control_latest

    def control(self):
        with self.condition:
            return self.control_latest

    def snapshot(self):
        return dict(self.stats, recording_queue_depth=self.record_queue.qsize())

    def control_worker(self):
        previous = 0
        while not self.stop.is_set():
            item = self.next_frame(previous)
            if not item or item[2] == previous:
                continue
            frame, captured, previous = item
            try:
                begin = time.monotonic()
                jpeg = self.encoder(frame)
                now = time.monotonic()
                with self.condition:
                    self.control_latest = (jpeg, captured, previous)
                    self.condition.notify_all()   # pipeline-ahead-r1: wakes wait_control (next_frame waiters re-check their own predicate)
                if self.last_control:
                    self.stats['control_publish_max_gap_s'] = max(
                        self.stats['control_publish_max_gap_s'], now-self.last_control)
                self.last_control = now
                self.stats.update(control_published=self.stats['control_published']+1,
                                  control_encode_ms=(now-begin)*1000,
                                  control_publish_age_s=now-captured, control_seq=previous)
            except Exception as exc:
                self.stats['control_error'] = str(exc)

    def preview_worker(self):
        previous = 0
        while not self.stop.is_set():
            item = self.next_frame(previous)
            if not item or item[2] == previous:
                continue
            frame, captured, previous = item
            try:
                begin = time.monotonic()
                jpeg = self.encoder(frame)
                self.preview(jpeg, captured, previous, (time.monotonic()-begin)*1000)
                self.stats['preview_published'] += 1
            except Exception as exc:
                self.stats['preview_error'] = str(exc)
            self.stop.wait(.12)  # Browser-only limit; never delays control/capture.

    def record_worker(self):
        while not self.stop.is_set():
            try:
                frame, captured, manual_id, pure_id = self.record_queue.get(timeout=.1)
            except queue.Empty:
                continue
            begin = time.monotonic()
            try:
                if manual_id is not None:
                    accepted = self.recorder.write(frame, expected_id=manual_id,
                                                   capture_at=captured)
                    if accepted is False:
                        self.stats['recording_stale_session'] += 1
                if pure_id is not None:
                    self.record_only.write(frame, expected_id=pure_id, capture_at=captured)
                self.stats['recording_processed'] += 1
            except Exception as exc:
                self.stats['recording_errors'] += 1
                self.stats['recording_error'] = str(exc)
                if manual_id is not None and self.recorder.run_id==manual_id:
                    self.recorder.error = str(exc)
            finally:
                elapsed = (time.monotonic()-begin)*1000
                self.stats['recording_write_max_ms'] = max(
                    self.stats['recording_write_max_ms'], elapsed)
                self.record_queue.task_done()
