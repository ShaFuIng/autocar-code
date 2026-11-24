"""Gyro damping on straights (gyro-damp-r1).

User 2026-11-03: on straights the car comes back to the centre at an angle and only steers the other way when it is
near the other line. Six runs after the wheel change (analysis_wheels/weave.py): at every centre crossing the PID
command had already reversed, but the yaw rate reversed 0.38-0.95 s later; camera -> PID 0.13 s, PID -> sent 0.13 s,
sent -> MCU 0.26 s (800 us/s ramp), MCU -> yaw 0.38-0.51 s. With ~1 s from image to yaw response the image error
alone always overshoots. The gyro sees the car's rotation at once.

steer += K_G * (gz - gz_lp)   (gz deg/s, left positive -> a left rotation adds right steering)
gz_lp is a TAU_S low-pass, so a steady curve rate does not pull the steering straight. Only on straights: the
image error within E_MAX and |gz_lp| within GZ_LP_MAX; the term is limited to +-CAP_US. Values are experiment
values: at the centre crossings the yaw rate was 7-9.5 deg/s (-> 40-60 us), the D term 63-72 us.

r4 (user 2026-11-06, live f67f1b69 with the new linkage): on the fast straights 92 of 126 frames had |term| > 40 us and
39 hit the 90 us cap while the PID itself moved 33 us per frame; the car visibly twitched. The constants now come from
gyro_damp.json (key absent = the r3 value): K_G 18 -> 10, CAP_US 90 -> 50, and with scale_with_gain the term and the cap
are multiplied by the gain-schedule factor k (15 / PWM) that live_pid passes, so at PWM 30 the damp is halved like the
PID. Experiment values; closed loop not verified. (2026-11-06 evening: json back to 18 / 90 / no scaling after 4b534c7d.)

r5 (user 2026-11-06, live 941d0eba / 14e31a43 / 62df13f3: three crashes into the right pillar at the first tunnel): after the
right curve before the tunnel the car crossed the centre line still rotating right at 9-13 deg/s. The high-passed term
(gz - gz_lp) is ~0 for a sustained rotation and the |gz_lp| > GZ_LP_MAX guard was still tripped from the curve, so the damp
did nothing while the PID, at zero error, steered straight; the residual rotation carried the car into the pillar. With
yaw_mode on, a frame where the error is small AND the PID steering (after the gain schedule) is within yaw_off_us of the
centre AND |gz| > yaw_g0 uses the full yaw rate: term = K_G * gz, limited to +-yaw_cap_us (OK_YAW). A curve keeps the PID
far from the centre, so it is excluded by the steering offset, not by gz_lp; the twitch/sawtooth centre crossings get the
same opposing term. Replay 2026-11-06 (six runs): the term would have been -90 us for 1.6 s on the tunnel approach of
941d / 14e3; never active on a curve_ff frame. Key absent = r4 behaviour. Experiment values, closed loop not verified.
"""
import json, math
from pathlib import Path

REVISION = 'gyro-damp-r6'
K_G, TAU_S, E_MAX, GZ_LP_MAX, CAP_US = 18., 1.5, .35, 8., 90.   # r2 (live b9fa9cff): at 6 the median |term| was 11.7 us; r3 (user: straights still S-shaped): 12 -> 18
CONFIG_FILE = Path(__file__).with_name('gyro_damp.json')
DEFAULTS = dict(enabled=True, k_g=K_G, tau_s=TAU_S, e_max=E_MAX, gz_lp_max=GZ_LP_MAX, cap_us=CAP_US, scale_with_gain=False,
                yaw_mode=False, yaw_off_us=120., yaw_g0=3., yaw_cap_us=90., yaw_conv_g0=None)
# r6 (user 2026-11-06, live 6fc436d5): the r5 yaw mode also fought the car's own return to the centre (28.7-31.1 s: error -0.25,
# rotating left at 4-8 deg/s = coming back, +90 us right for 3 s; reversals of 180 us on the straights). A rotation is DIVERGING
# when the error and the yaw rate are on the same side (err x gz > 0: car left and still rotating left) and CONVERGING otherwise.
# Diverging: oppose above yaw_g0. Converging: oppose only above yaw_conv_g0 (the fast pre-overshoot rotation of the tunnel
# approach, 15-22 deg/s). yaw_conv_g0 absent/null = r5 (same threshold both ways). Replay 6fc436d5: yaw frames 272 -> 125.



def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


class GyroDamp:
    def __init__(self, config=None):
        self.lp = None; self.t = None; self.c = dict(config) if config is not None else load_config()

    def step(self, now, gz, err, steer, lo, hi, scale=None, center=None):
        c = self.c
        if gz is None or not math.isfinite(gz): return steer, dict(revision=REVISION, reason='NO_GYRO')
        dt = 0. if self.t is None else max(0., min(.5, now - self.t)); self.t = now
        self.lp = gz if self.lp is None else self.lp + (1. - math.exp(-dt / float(c['tau_s']))) * (gz - self.lp)
        info = dict(revision=REVISION, gz=round(gz, 2), gz_lp=round(self.lp, 2))
        if not c.get('enabled'): info.update(reason='DISABLED'); return steer, info
        if err is None or abs(err) > float(c['e_max']): info.update(reason='NOT_STRAIGHT_ERROR'); return steer, info
        s = 1.
        if c.get('scale_with_gain') and scale is not None and math.isfinite(float(scale)): s = max(0., min(1., float(scale))); info['scale'] = round(s, 3)
        cg = c.get('yaw_conv_g0'); conv = (float(err) * gz) < 0.   # r6: converging = rotating back toward the centre
        g_need = float(cg) if (conv and cg is not None) else float(c['yaw_g0'])
        if (c.get('yaw_mode') and center is not None and math.isfinite(float(center)) and abs(float(steer) - float(center)) <= float(c['yaw_off_us'])
                and abs(gz) > g_need):   # r5: residual rotation while the PID steers near the centre; r6: converging needs yaw_conv_g0
            cap = float(c['yaw_cap_us']) * s
            term = max(-cap, min(cap, float(c['k_g']) * s * gz))
            out = max(lo, min(hi, steer + term))
            info.update(reason='OK_YAW', term_us=round(term, 1), pid_steer_us=round(steer, 1), steer_us=round(out, 1), steer_off_us=round(float(steer) - float(center), 1), converging=conv)
            return out, info
        if abs(self.lp) > float(c['gz_lp_max']): info.update(reason='NOT_STRAIGHT_TURNING'); return steer, info
        cap = float(c['cap_us']) * s
        term = max(-cap, min(cap, float(c['k_g']) * s * (gz - self.lp)))
        out = max(lo, min(hi, steer + term))
        info.update(reason='OK', term_us=round(term, 1), pid_steer_us=round(steer, 1), steer_us=round(out, 1))
        return out, info
