"""Selectable v3 preview policy. v2.1 and saved PID values remain unchanged."""
from preview_turn_pid import PreviewTurnPID


class StraightPreviewPID(PreviewTurnPID):
    def __init__(self, parameters):
        super().__init__(parameters)
        # Experimental earlier demand; existing travel/rate/freshness limits apply.
        self.policy.update(revision='preview-turn-r3', max_us=150.)
