import tempfile, unittest
from unittest.mock import patch
from test_live_pid import LiveTests, ROOT

class DurationTests(unittest.TestCase):
    def test_custom_duration_and_deadline(self):
        for seconds in (21, 60, 120.5, 3600):
            with self.subTest(seconds=seconds), tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
                live,m,now,cmd,p=LiveTests().make(f)
                p.update(seconds=seconds,execution_mode='orin_timed')
                with patch('manual_drive.threading.Thread.start'):live.start(p)
                self.assertEqual(live.run['seconds'],seconds)
                self.assertIsNotNone(m.current['deadline'])
                live.run['deadline']=0;live.check_safety()
                self.assertEqual(live.run['reason'],'PID_DURATION_COMPLETE')
                self.assertTrue(m.cancel.is_set())
                self.assertEqual(cmd,[])

    def test_invalid_duration_has_no_output(self):
        for seconds in (0,-1,True,'60',float('inf'),float('nan')):
            with self.subTest(seconds=seconds), tempfile.TemporaryDirectory(dir=ROOT/'tests') as f:
                live,m,now,cmd,p=LiveTests().make(f);p['seconds']=seconds
                with self.assertRaises(ValueError):live.start(p)
                self.assertEqual(cmd,[])
                self.assertIsNone(live.run)
