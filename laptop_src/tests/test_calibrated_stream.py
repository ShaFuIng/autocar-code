import test_pid_stream_safety as harness


class V7StreamTests(harness.StreamTests):
    def exercise(self, fault):
        return super().exercise(fault, algorithm='calibrated-v7')

    def test_calibration_identity_changes(self):
        self.exercise('calibration')
