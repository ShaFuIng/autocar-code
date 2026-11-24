"""gyro-hold-r1 (integration B3b, 2026-11-12): the one gyro heading-hold primitive of live_pid.py.

Law (crossing-pass-r9 / down-straight-r1 GYRO, unchanged): at the first lost frame the yaw is captured as the reference in
holder.<ref_attr>; then
    u   = k_gz * gz + k_yaw * (yaw - ref)        (left positive; gz None -> 0)
    u   = clamp(u, -lim_us, +lim_us)
    out = clamp(centre + u, min_us, max_us)
Reference capture / release, gains, limit and the timeout are the primitive's; the reference lives on the caller's object under the
caller's attribute name (as before), so its lifetime and every reset point are unchanged.

Parameters per call come from the caller's own json, exactly as before (crossing_pass.json "hold_gyro" for the crossing hold,
read once per process; down_straight.json for the downhill hold, re-read when the file changes). gyro_hold.json (read once at
import) holds the named profiles = the values used when a key is missing from that json (before: literals 4 / 10 / 250 / 1.5 in
live_pid). Both profiles use the same law; they are instances, not copies.

Sites with a different law are NOT routed here (see integration-20261112/b23/b3b/NOTES.md): heading_hold HEADING_ONLY / BLIND / LOST
(centre - clamp(K_HOLD * (ref - yaw) [+ I]), no gz term, own reference rules, lane_cap; NaN edge differs), upper_single ALIGN / HOLD
(own slew and limits), lane_segment gyro source, bottom_map MAP_TURN (yaw-rate target), upper_path (recorded profile).
xp-hold-r10 (integration B5, port of b4/c1): optional base. hold(..., base=None) = the law above (unchanged); with a number the hold
acts around base instead of the centre and is limited to c['rel_lim_us'] (default 100) instead of lim_us:
    out = clamp(base + clamp(u, -rel_lim_us, +rel_lim_us), min_us, max_us)
rel_base() computes that base for the crossing hold from the dispatched commands (PidHold history); it returns None when
c['rel_base'] is not 'last' / 'mean' (= off), and the caller then passes base=None. The down_straight hold never passes a base.
Python 3 standard library only."""
import json, time
import pause_clock   # pause-clock-r1 (pause-resume-20261120)
from pathlib import Path

REVISION = 'gyro-hold-r1'
CONFIG_FILE = Path(__file__).with_name('gyro_hold.json')
DEFAULTS = dict(profiles=dict(
    crossing=dict(k_gz=4., k_yaw=10., lim_us=250., max_s=1.5),   # crossing-pass-r9: crossing_pass.json hold_gyro
    downhill=dict(k_gz=4., k_yaw=10., lim_us=250.)))             # down-straight-r1 GYRO: down_straight.json


def load_config():
    """DEFAULTS updated by gyro_hold.json; a missing / unreadable / malformed file (or a profile without the default keys as numbers)
    -> DEFAULTS unchanged (never raises: live_pid imports this module)."""
    c = dict(profiles={k: dict(v) for k, v in DEFAULTS['profiles'].items()})
    try:
        f = json.loads(CONFIG_FILE.read_text(encoding='utf8')); n = {k: dict(v) for k, v in c['profiles'].items()}
        for k, v in (f.get('profiles') or {}).items():
            n.setdefault(k, {}).update({kk: float(vv) for kk, vv in dict(v).items() if kk in ('k_gz', 'k_yaw', 'lim_us', 'max_s')})
        for k, v in DEFAULTS['profiles'].items():
            if not all(isinstance(n[k].get(kk), float) for kk in v): raise ValueError(k)
        c['profiles'] = n
    except Exception: pass
    return c


CONFIG = load_config()


def profile(name):
    return CONFIG['profiles'][name]


def release(holder, ref_attr):
    """forget the captured reference (the next hold captures a new one)."""
    setattr(holder, ref_attr, None)


def hold(holder, ref_attr, heading, c, prof, c0, lo, hi, base=None):
    """one hold step: (steer_us, yaw - ref, gz) or None without a yaw. c = the caller's json dict, prof = profile(name) (defaults for
    missing keys). IMU read errors and bad config values propagate to the caller's try (as before). Same operation order as before.
    base (xp-hold-r10): None = hold around c0 with lim_us (unchanged); a number = hold around base with c['rel_lim_us'] (default 100)."""
    tn = pause_clock.monotonic(); yaw = heading._yaw(tn); gz = heading._gz(tn)
    if yaw is None: return None
    if getattr(holder, ref_attr, None) is None: setattr(holder, ref_attr, float(yaw))
    ref = getattr(holder, ref_attr); lim = float(c.get('lim_us', prof['lim_us'])) if base is None else float(c.get('rel_lim_us', 100.))
    u = float(c.get('k_gz', prof['k_gz'])) * float(gz or 0.) + float(c.get('k_yaw', prof['k_yaw'])) * (float(yaw) - ref)
    u = max(-lim, min(lim, u)); out = max(lo, min(hi, (c0 if base is None else base) + u))
    return out, float(yaw) - ref, gz


def rel_base(c, hold_state, lost_s, c0):
    """xp-hold-r10 (integration B4 C1, 2026-11-12: both r9 holds that fired on 11-12 replaced a right command of 1760 / 1720 us by
    1281 / 1320 (centre + 4*gz with the car turning right as asked), a 400-480 us reversal; the pre-r9 hold of the last command over-turned
    in 981f / dca0 / fa44 because the last frame before a loss was the peak of an error jump): the base of the crossing-window gyro hold,
    or None = off. c['rel_base'] 'last' = last valid command (hold_state.last, pre-compensation), 'mean' = mean of the commands sent
    (hold_state.hist) in rel_mean_s up to the last valid frame; the base decays linearly to c0 + rel_straight_off_us over rel_decay_s
    (null = no decay). No valid command yet -> None (the r9 centre hold). hold_state = live_pid's PidHold. Experiment values
    (integration-20261112/b4/c1). Pure arithmetic: no clock / IMU / file read."""
    rb = c.get('rel_base')
    if rb not in ('last', 'mean'): return None
    hl = getattr(hold_state, 'last', None)
    if hl is None or getattr(hold_state, 'last_t', None) is None: return None
    b = float(hl['steer_us'])
    if rb == 'mean':
        xs = [s for t, s in getattr(hold_state, 'hist', []) if 0. <= hold_state.last_t - t <= float(c.get('rel_mean_s', .3))]
        if xs: b = sum(xs) / len(xs)
    ds = c.get('rel_decay_s')
    if ds: st = c0 + float(c.get('rel_straight_off_us') or 0.); b = st + (b - st) * max(0., 1. - float(lost_s) / float(ds))
    return b


def expired(lost_s, c, prof):
    """timeout of a profile with max_s (crossing): lost longer than max_s since the last valid frame."""
    return lost_s > float(c.get('max_s', prof['max_s']))
