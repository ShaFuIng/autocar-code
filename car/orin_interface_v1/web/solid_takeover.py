"""Solid-line takeover after the dashes on the upper section (solid-takeover-r1).

User 2026-11-06 (eb7d1033): after the dashed turn the car follows the dashes for 3-4 s, then the last 6.5-7.4 s before the slope
edge have no dash line on every lap. There the heading hold / lane segment keep the heading reference set by the turn; in eb7d
that reference did not match the straight, the camera said -0.6 .. -1.0 (lane to the left) for 6 s while the commands stayed
1160-1800, and the car reached the slope edge right of the lane.
Rule: on the upper section (route OPEN), heading hold active on HEADING_ONLY / HEADING_VISUAL (dash turn done, no dash line used),
heading error within ALIGN_DEG, a valid normal-path camera error, pitch not on the slope: when the camera error stays beyond
ON_ERR on the same side for ON_S, the PID command (the camera, the value before the heading hold) is sent instead of the heading /
lane-segment command, until the error is back within OFF_ERR. Experiment values; the closed-loop effect is not verified.
"""
REVISION = 'solid-takeover-r2'
ON_ERR, OFF_ERR, ON_S, ALIGN_DEG, MIN_PITCH = .4, .2, .5, 20., -5.


class SolidTakeover:
    def __init__(self):
        self.on = False; self.t0 = None; self.side = 0

    def reset(self):
        self.on = False; self.t0 = None; self.side = 0

    def step(self, now, route_open, hh, dash_used, image_error, normal, pitch):
        """Returns (pid_steer_us or None, info or None). info None when the rule does not apply to this frame."""
        hh = hh or {}
        ok = (route_open and hh.get('active') and hh.get('reason') in ('HEADING_ONLY', 'HEADING_VISUAL') and not dash_used and normal
              and image_error is not None and hh.get('err_deg') is not None and abs(float(hh['err_deg'])) <= ALIGN_DEG
              and hh.get('pid_steer_us') is not None and (pitch is None or float(pitch) > MIN_PITCH))
        if not ok:
            was = self.on; self.reset()   # r2: a takeover that ends because its conditions stop also reports a release
            return None, (dict(revision=REVISION, active=False, released=True, why='CONDITIONS') if was else None)
        e = float(image_error); s = 1 if e > 0 else -1; released = False
        if self.on:
            if abs(e) <= OFF_ERR: self.reset(); released = True   # r2: the camera is back on the lane
        else:
            if abs(e) >= ON_ERR:
                if self.t0 is None or s != self.side: self.t0 = now; self.side = s
                elif now - self.t0 >= ON_S: self.on = True
            else: self.t0 = None; self.side = 0
        info = dict(revision=REVISION, active=self.on, e=round(e, 3), pending_s=None if self.t0 is None or self.on else round(now - self.t0, 2), released=released)
        if self.on:
            info['pid_steer_us'] = round(float(hh['pid_steer_us']), 1)
            return float(hh['pid_steer_us']), info
        return None, info
