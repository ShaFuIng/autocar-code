"""Passive UART evidence collector. No serial or control APIs."""
import copy

class FaultEvidence:
    def __init__(self):
        self.last_ms = None
        self.epoch = 0
        self.snapshot = None
        self.last_event = None
        self.last_i2c = None
        self.retry = None

    def consume(self, message, received_wall, received_monotonic):
        kind = message.get('type')
        if kind not in ('status', 'event', 'fault_diagnostics', 'i2c_failure', 'i2c_retry'):
            return None
        uptime = message.get('ms')
        reset = False
        if isinstance(uptime, int):
            if self.last_ms is not None and uptime < self.last_ms:
                self.epoch += 1
                self.snapshot = None
                self.last_event = None
                self.last_i2c = None
                self.retry = None
                reset = True
            self.last_ms = uptime
        if kind == 'event':
            self.last_event = copy.deepcopy(message)
        elif kind == 'fault_diagnostics':
            self.snapshot = copy.deepcopy(message)
        elif kind == 'i2c_retry':
            previous = self.retry
            self.retry = copy.deepcopy(message)
            if previous == self.retry:
                return None  # Persist transitions, not duplicate 4 Hz idle snapshots.
        elif kind == 'i2c_failure':
            self.last_i2c = copy.deepcopy(message)
        if kind == 'status' and not reset:
            return None
        return dict(time=received_wall, received_monotonic=received_monotonic,
                    firmware_epoch=self.epoch, uptime_reset=reset,
                    firmware_message=copy.deepcopy(message))

    def view(self):
        return dict(firmware_epoch=self.epoch,
                    snapshot=copy.deepcopy(self.snapshot),
                    retry=copy.deepcopy(self.retry),
                    last_i2c=copy.deepcopy(self.last_i2c),
                    last_event=copy.deepcopy(self.last_event))
