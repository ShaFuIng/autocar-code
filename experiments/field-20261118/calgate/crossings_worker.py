"""crossings-worker-r1 (crossings-20261114, offline candidate): switches for three worker-side rules of the
camera-pair-v2-junction path stage. Read by JunctionLine.set_row at each model load (not by live_pid).
  amb_one_side        (bool)  ground_identity: ambiguous current pairs + exactly one side with a continuous previous-frame
                              track -> that side alone (no path gap, no junction fallback corridor)
  corridor_cal_h_max  (px|null) junction_corridor: refuse a live entrance calibration whose horizon is below this row
  fallback_cont_px    (px|null) junction_switch: refuse a fallback corridor take more than this far from the previous
                              valid baseline at the control row
  outside_handback    (bool)  junction_switch (curve3-handback-r1, curve3-20261115): a corridor take that ends with
                              VEHICLE_OUTSIDE_CORRIDOR hands back to a continuous baseline for every take reason, and
                              retries on each frame of that NO_TARGET episode until STOP_S
  outside_handback_px (px|null) continuity limit of that hand-back vs the previous valid baseline (null = 80 = D)
  outside_handback_side (bool) only to a baseline on the same side of the visual centre as the last target, >= as far out
  outside_handback_hist_n / outside_handback_hist_px   the baseline covered the control row on the last n frames, each
                              step <= px (n 0 / absent = off; px null = 120)
  slope_width         ({pitch_deg, ratio}|null) ground_identity (slope-width-r1, slope-20261115): while the worker's IMU pitch
                              <= pitch_deg the single-line width ratio is the latest two-line ratio seen on this slope, else
                              'ratio'; live_pid then skips its downhill-width narrowing on those frames (line_estimate.slope_width)
  corridor_guard      (dict|null) junction_switch (corridor-guard-r1, corridor-20261118): guards on the fallback corridor take
                              (BASELINE_NO_PATH_CORRIDOR) and an early hand-back of such a take; keys (absent / null = off):
                              off (bool) no fallback take at all; gz_max (dps) no fallback take while |worker IMU gz| > gz_max;
                              cal_hmax (px) / cal_age_s (s) no fallback take while the corridor calibration horizon is below
                              cal_hmax / older than cal_age_s; dir_px (px, dir_dy rows, default 40) no fallback take whose
                              direction (x at the control row vs dir_dy rows above) differs from the previous valid baseline by
                              more than dir_px; back {px, cont_px, n}: a fallback take hands back to the baseline when the
                              baseline covers the control row, moved <= cont_px from the previous valid baseline, was continuous
                              (<= D) on the last n frames, and is more than px from the corridor at the control row;
                              optional gz_min (dps): only while |worker IMU gz| >= gz_min and the car yaws toward the
                              baseline's side of the corridor (right turn = gz < 0 = baseline right of the corridor);
                              optional reasons (list): take reasons handed back early (default ["BASELINE_NO_PATH_CORRIDOR"];
                              "BASELINE_JUMP_CORRIDOR" adds the baseline-jump takes)
  corridor_cal_gate   (dict|null) junction_corridor (corridor-cal-gate-r1, corridor-weak-20261118): a live corridor calibration
                              is refused when its horizon is below hmax px (y larger) or |worker IMU pitch| at that frame > pitch
                              deg; one with width slope > ws_max (upper deck, both outer edges) is refused, or with wide_drop_pitch
                              kept and dropped on the first frame with pitch <= wide_drop_pitch (downhill); keys absent / null /
                              {} = off; after 1 s without a calibration the saved default is used as before
  corridor_weak       (dict|null) junction_switch (corridor-weak-r1, corridor-weak-20261118): a corridor take whose support is
                              only 'weak' (one side / below N rows) while the baseline is back hands back to the baseline;
                              keys (absent / null / {} = off): px (default 60) baseline more than px from the corridor at the
                              control row; cont_px (default 40) baseline moved <= cont_px from the previous valid baseline;
                              hold_s (default 0.1) all of it true for hold_s (time, not frames); reasons (list, default
                              ["BASELINE_NO_PATH_CORRIDOR"]) take reasons concerned; two_min (fraction|null) baseline rows from
                              both lines ('double') >= two_min; skip_zones (list, default []) live_pid zone flags where the rule
                              is off (e.g. ["xp2"]); expire (bool) at NO_STRONG_SUPPORT hand back to a valid baseline continuous
                              with the previous one (<= D) instead of NO_TARGET, expire_px (px|null) only within expire_px;
                              ramp_s (s|null) after such a hand-back the target moves from the corridor to the baseline linearly
                              over ramp_s (the path is the baseline path shifted sideways); strong_take {two_min, lr_max}
                              (dict|null): no STRONG_CORRIDOR_DISAGREES take while >= two_min of the baseline rows come from
                              both lines and the corridor's weaker side has < lr_max rows (outside skip_zones); gz_max (dps|null)
                              the weak hand-back only while |worker IMU gz| <= gz_max (unknown gz -> no hand-back)
All off / file missing = the snapshot behaviour, byte-identical output. Experiment values, not validated limits."""
import json
from pathlib import Path

REVISION = 'crossings-worker-r1'
CURVE3_REVISION = 'curve3-handback-r1'
CONFIG_FILE = Path(__file__).with_name('crossings_worker.json')
DEFAULTS = dict(amb_one_side=False, corridor_cal_h_max=None, fallback_cont_px=None, outside_handback=False, outside_handback_px=None,
                outside_handback_side=False, outside_handback_hist_n=0, outside_handback_hist_px=None, slope_width=None, corridor_guard=None,
                corridor_weak=None, corridor_cal_gate=None)
GUARD_REVISION = 'corridor-guard-r1'
WEAK_REVISION = 'corridor-weak-r1'
CAL_GATE_REVISION = 'corridor-cal-gate-r1'
OVERRIDE = None   # offline replay only


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    if OVERRIDE is not None: c = dict(DEFAULTS, **OVERRIDE)
    if c.get('enabled') is False: return dict(DEFAULTS)
    return c


def any_on(c):
    return bool(c.get('amb_one_side')) or c.get('corridor_cal_h_max') is not None or c.get('fallback_cont_px') is not None or bool(c.get('outside_handback')) or c.get('slope_width') is not None or bool(c.get('corridor_guard')) or bool(c.get('corridor_weak')) or bool(c.get('corridor_cal_gate'))
