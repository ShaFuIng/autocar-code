"""Flat bottom of the downhill right curve (downhill-bottom-r1).

User 2026-11-04 (raw-20261104h: 2f5e0412, 0bf4f209, f06132a7, 7cc8c7af): after the downhill settles the right turn is too
slow, the camera sees the neighbouring track and the car turns the wrong way (left).
 1. Speed scaling (downhill_speed.py) stayed on for DOWN_RELEASE_S (0.8 s) of level ground and cut a requested 1750 us to
    1534-1552 (0bf4 34.18-34.46 s, 2f5e 33.91-34.23 s, 7cc8 30.82-31.27 s).
    -> live_pid skips that scaling for commands right of centre once the pitch is back above LEVEL_PITCH after the steep
       part (the scale is still computed and logged so its rise limit keeps working if the pitch dips again). Left
       commands stay scaled: unscaled they would be stronger (a628f4f8 29.91 s 1227 -> 1103, 0fb475b7 30.3 s 1231 -> 1026).
 2. The path target jumped left in one frame onto the neighbouring track (2f5e 35.78 s e +0.13 -> -1.0, 7cc8 31.76 s
    +1.0 -> -0.59, 7cc8 126.37 s, f061 36.54 s, 0bf4 36.73 s) and the PID steered 900-1100 us. In the 11-04 records
    (downhill-bottom-20261104/survey.py) such jumps on level ground, 1.5 s or more after the slope start and before the car
    had turned 60 deg right, came almost only from passes that then failed, several staying 2.4-3 s on the wrong side;
    passes that turned right 80+ deg had none or short ones.
    -> step(): during the downhill tail, pitch >= PITCH_MAX, at least SINCE_MIN after the first pitch < SLOPE_PITCH and the
       yaw turned less than YAW_DONE (right negative) since then, an image error falling by more than JUMP_E in one
       frame (previous accepted error at most PE_MAX_S old) starts a HOLD: live_pid keeps the median of the commands sent
       in the last 0.5 s instead of the PID command. The hold ends when the error is back within JUMP_E of the error
       before the jump, after HOLD_MAX_S, once the yaw has turned YAW_DONE, or on a slope again.
Values are experiment values from these records; the closed-loop effect is not verified.

r2 (2026-11-05, cedc03ca 33.34 s): a jump from the right edge (previous error >= RIGHT_EDGE_E, 0.85) is the inner line leaving the
frame; the 0.5 s median there still held the left landing command (1305 us) and the car kept 1535-1592 us for 1.5 s. The hold
info now carries floor_us = the PID desired command of the frame before the jump (step(desired=...)); live_pid holds at least
that value. Other holds are unchanged (floor_us None).
"""
import math

REVISION = 'downhill-bottom-r2'
RIGHT_EDGE_E = .85   # r2: previous error at or beyond this = target at the right image edge (9ce62fb8 32.36 s: 0.89, same case)
LEVEL_PITCH = -2.
SLOPE_PITCH, PITCH_MAX, SINCE_MIN, JUMP_E, YAW_DONE, HOLD_MAX_S, PE_MAX_S = -5., -5., 1.5, .6, -60., 1.5, .5


class BottomJump:
    def __init__(self):
        self.t0 = None; self.turned = None; self.yaw_last = None
        self.pe = None; self.pe_t = None; self.hold_t0 = None; self.ref_e = None
        self.pd = None; self.ref_desired = None   # r2: PID desired command of the previous accepted frame / of the jump reference

    def _yaw(self, yaw):
        if yaw is None or not math.isfinite(yaw): return
        if self.yaw_last is not None and self.turned is not None: self.turned += (yaw - self.yaw_last + 180.) % 360. - 180.
        self.yaw_last = yaw

    def step(self, now, down_tail, pitch, e, yaw, desired=None):
        """Returns an info dict while the frame's command is to be held, else None."""
        if not down_tail: self.__init__(); return None
        if self.t0 is None and pitch is not None and pitch < SLOPE_PITCH: self.t0 = now; self.turned = 0.; self.yaw_last = None
        if self.t0 is None: return None
        self._yaw(yaw)
        if e is None: return None
        e = float(e); turned = self.turned if self.yaw_last is not None else None
        info = dict(revision=REVISION, e=round(e, 3), since_s=round(now - self.t0, 3), turned_deg=None if turned is None else round(turned, 1),
                    pitch=None if pitch is None else round(pitch, 1))
        if self.hold_t0 is not None:
            why = ('BACK' if e > self.ref_e - JUMP_E else 'TIMEOUT' if now - self.hold_t0 > HOLD_MAX_S
                   else 'TURNED' if turned is not None and turned <= YAW_DONE else 'SLOPE' if pitch is not None and pitch < PITCH_MAX else None)
            if why is None: return dict(info, reason='HOLD', ref_e=round(self.ref_e, 3), hold_s=round(now - self.hold_t0, 3), floor_us=self.ref_desired)
            self.hold_t0 = None; self.pe, self.pe_t, self.pd = e, now, desired; return None
        if (self.pe is not None and now - self.pe_t <= PE_MAX_S and self.pe - e > JUMP_E and pitch is not None and pitch >= PITCH_MAX
                and now - self.t0 >= SINCE_MIN and turned is not None and turned > YAW_DONE):
            self.hold_t0 = now; self.ref_e = self.pe
            self.ref_desired = round(float(self.pd), 1) if self.pe >= RIGHT_EDGE_E and self.pd is not None and math.isfinite(float(self.pd)) else None
            return dict(info, reason='JUMP', ref_e=round(self.ref_e, 3), hold_s=0., floor_us=self.ref_desired)
        self.pe, self.pe_t, self.pd = e, now, desired; return None
