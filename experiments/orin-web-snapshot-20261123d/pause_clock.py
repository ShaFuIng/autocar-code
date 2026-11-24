"""pause-clock-r1 (pause-resume-20261120): the control timeline = Orin monotonic clock minus the paused time removed so far.

Why: the control modules keep many time stamps (filters, windows, latches). Instead of shifting each stored field after a pause, every
clock reading and every sensor time stamp that enters the control is read on one timeline from which the paused stretch is cut out.
After a resume the modules see the frame after the pause as if it came one camera interval after the last controlled frame.

Sources mapped through this module (everything else in the control reads no clock):
  monotonic()   live_pid (module name `time` rebound to TIME), speed_policy, gyro_hold, output_stage, speed_adapt, imu_boost
  imu_t(t)      IMU latest.json 't' read by heading_hold._yaw/_gz, imu_boost._read, centre_estimator._imu, live_pid _L_arc_lat
  to_virtual()  camera capture time in live_pid._loop / _p2_peek (the worker still gets the real capture time: to_real())
Offset 0 (no pause yet, or idle after a run) -> every value is the unchanged float (x - 0.0 == x): no behaviour change.
Only LivePID._loop changes the offset (shift()), between frames, under LivePID.lock. Python 3 standard library."""
import time as _time

REVISION = 'pause-clock-r1'
_P = 0.0   # seconds removed from the control timeline (sum of applied pause compensations)


def offset():
    return _P


def monotonic():
    return _time.monotonic() - _P


def imu_t(t):
    return t - _P


def to_virtual(t_real):
    return t_real - _P


def to_real(t_virtual):
    return t_virtual + _P


def shift(d):
    """add d seconds to the removed time (d < 0 gives time back; used to return to 0 when no run is active)."""
    global _P
    _P = _P + float(d)
    return _P


class _TimeProxy(object):
    """stand-in for the `time` module inside live_pid: monotonic() on the control timeline, everything else (time, sleep, ...) real."""
    def monotonic(self):
        return _time.monotonic() - _P

    def __getattr__(self, name):
        return getattr(_time, name)


TIME = _TimeProxy()
