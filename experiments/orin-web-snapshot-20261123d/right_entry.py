"""Right-turn entry: fade out the high-speed steering cut (right-entry-r1).

User 2026-11-05: steer earlier into the right turns ("one frame earlier").
Records 2026-11-05 (31 runs, 144 right-lock entries): gain-schedule-r3 scales the steering offset by 15 / PWM above PWM 15
(PWM 20 -> 0.75). At a right-turn entry the speed is still 18-20 while the PID already asks for lock, so the sent command
stays 60-130 us short and reaches the 1750 lock up to 0.5 s (about 5 frames) after the PID: 9dbded48 73.63 s PID 1747 /
sent 1658, 78.24 s PID 1878 (capped 1750) / sent 1660, lock sent only at 78.74 s.
A one-frame extrapolation of the PID command was tried offline and moved only 4 of 144 entries (most entries are at PWM 15,
where the sent command already equals the PID).
Here, on the right side only, the scale k fades to 1 between START_US and FULL_US right of centre (the PID steering before
the cut): +240 us -> the normal k, +360 us (1750 lock at centre 1390) -> no cut. Left turns and offsets up to +240 us keep
gain-schedule-r3. +240 is where the right-lock curvature starts to grow (diff-assist records, curvature rises from +180 and
stops at +300). Experiment values, not safety limits.
"""
REVISION = 'right-entry-r1'
START_US = 240.
FULL_US = 360.


def k_for(k, pid_us, center_us):
    """Returns (k used, info or None). info only when the fade changes k."""
    dev = float(pid_us) - float(center_us)
    if dev <= START_US or k >= 1.: return k, None
    a = min(1., (dev - START_US) / (FULL_US - START_US))
    kk = k + (1. - k) * a
    return kk, dict(revision=REVISION, k_gs=round(k, 3), k=round(kk, 3), dev_us=round(dev, 1))
