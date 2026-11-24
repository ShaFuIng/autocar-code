import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
import tempfile
import json
import subprocess

spec = importlib.util.spec_from_file_location('guard', Path(__file__).with_name('clock_guard.py'))
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)

class ClockTests(unittest.TestCase):
    def test_restore_late_rtc_reset(self):
        self.assertEqual(guard.decision(33, [1792300000, 1792300060]), (1792300060, True))
    def test_never_roll_back(self):
        self.assertEqual(guard.decision(1792400000, [1792300000]), (1792300000, False))
    def test_reject_uninitialized_and_invalid_floor(self):
        self.assertEqual(guard.decision(33, [0, None, 'bad', 9999999999]), (None, False))
    def test_connected_wifi_untouched(self):
        with tempfile.TemporaryDirectory() as d, patch.object(guard, 'RUNTIME', Path(d)), patch.object(guard, 'run', return_value=subprocess.CompletedProcess([], 0, 'wlP1p1s0:connected\n')) as run:
            guard.retry_wifi()
            self.assertEqual(run.call_count, 1)
    def test_retry_limit(self):
        with tempfile.TemporaryDirectory() as d, patch.object(guard, 'RUNTIME', Path(d)), patch.object(guard, 'run') as run:
            guard.atomic(Path(d)/'wifi-retry.json', {'attempts': 3, 'last_monotonic': -1000})
            guard.retry_wifi()
            run.assert_not_called()
    def test_check_is_read_only(self):
        with patch('sys.argv', ['guard', '--check']), patch.object(guard, 'rtc_ready', return_value=True), patch.object(guard, 'trusted_times', return_value=[1792300000]), patch.object(guard, 'atomic') as write, patch.object(guard, 'run') as run, patch.object(guard.time, 'clock_settime', create=True) as settime:
            guard.main()
            write.assert_not_called(); run.assert_not_called(); settime.assert_not_called()

if __name__ == '__main__':
    unittest.main()
