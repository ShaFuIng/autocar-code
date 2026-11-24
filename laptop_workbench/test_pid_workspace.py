"""No network or hardware: verify field records and frozen run context."""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from server import App, Store
from pid_workspace import Workspace, summarize
from visual_pid import DEFAULTS

class FieldTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=Path(__file__).parent / 'data')
        self.store = Store(self.tmp.name)
        self.app = SimpleNamespace(store=self.store, integration=Mock())
        self.work = Workspace(self.app)
        self.work.records = lambda: [dict(id='a'*32, actual_seconds=3)]

    def tearDown(self):
        self.store.db.close()
        self.tmp.cleanup()

    def test_append_and_time_validation(self):
        payload = dict(run='a'*32, tracking='偏右', contact='未確認', at_s=None, notes='觀察')
        a, b = self.work.add_review(payload), self.work.add_review(payload)
        self.assertNotEqual(a['id'], b['id'])
        self.assertEqual(len(self.work.reviews()), 2)
        for invalid in (-1, 9, float('nan'), True):
            with self.assertRaises(ValueError):
                self.work.add_review(dict(payload, at_s=invalid))

    def test_unknown_geometry_and_completion(self):
        self.assertIsNone(self.work.geometry()['front_axle_blind_cm'])
        row = summarize([dict(kind='start', started=1), dict(kind='stop', at=3, reason='complete')], [])
        self.assertEqual(row['actual_seconds'], 2)
        self.assertEqual(row['line_contact'], 'unknown')

    def test_reserved_records_cannot_be_overwritten(self):
        for key in ('pid-field-review-a', 'pid-session-a'):
            with self.assertRaises(ValueError):
                App.validate_document(self.app, dict(key=key, data={}, rev=0))

    def test_old_remote_rejects_enabled_compensation(self):
        self.store.save('visual-pid',0,dict(parameters=dict(DEFAULTS,backlash_enabled=1)))
        self.app.integration.request.return_value={}
        with self.assertRaises(ValueError):
            App.post(self.app,'/api/pid/live/start',dict(parameter_revision=1))
        self.app.integration.request.assert_called_once_with('/api/pid/status')

    def test_timed_mode_rejects_old_remote_without_start(self):
        self.store.save('visual-pid',0,dict(parameters=DEFAULTS))
        self.app.integration.request.return_value={'backlash_version':2}
        with self.assertRaises(ValueError):
            App.post(self.app,'/api/pid/live/start',dict(parameter_revision=1,execution_mode='orin_timed'))
        self.app.integration.request.assert_called_once_with('/api/pid/status')

    def test_timed_mode_forwarded_and_frozen(self):
        self.store.save('visual-pid',0,dict(parameters=DEFAULTS))
        self.app.integration.request.return_value={'autonomous_pid_version':1,'backlash_version':2,'run_id':'c'*32}
        App.post(self.app,'/api/pid/live/start',dict(parameter_revision=1,execution_mode='orin_timed'))
        self.assertEqual(self.app.integration.request.call_args.args[1]['execution_mode'],'orin_timed')
        self.assertEqual(self.store.get('pid-session-'+'c'*32)['data']['execution_mode'],'orin_timed')

    def test_start_freezes_saved_context_using_mock_only(self):
        self.store.save('visual-pid', 0, dict(parameters=DEFAULTS))
        self.store.save('vehicle-measurements', 0, dict(items={
            'camera_forward': dict(value=10), 'camera_near_ground': dict(value=39)}))
        self.app.integration.request.return_value = dict(run_id='b'*32)
        App.post(self.app, '/api/pid/live/start', dict(parameter_revision=1))
        context = self.store.get('pid-session-'+'b'*32)['data']
        self.assertEqual(context['geometry']['front_axle_blind_cm'], 49)
        self.assertEqual(context['parameters'], DEFAULTS)
        self.assertEqual(self.app.integration.request.call_count, 2)
        with self.assertRaises(ValueError):
            App.post(self.app, '/api/pid/live/start', dict(parameter_revision=0))
        self.assertEqual(self.app.integration.request.call_count, 2)

if __name__ == '__main__':
    unittest.main()
