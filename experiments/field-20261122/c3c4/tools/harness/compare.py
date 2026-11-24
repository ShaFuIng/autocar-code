"""Comparison helpers: replay output vs the car log (fidelity) and replay vs replay (differential)."""
import json, math
from collections import Counter

# proposal sub-dicts / fields in fix33 pipeline order (used to name the first layer whose logged values differ)
ORDER = ['t', 'error', 'image_error', 'p', 'i', 'd', 'desired_us', 'raw_steer_us', 'limited', 'turn_preview', 'sample_row_ratio', 'imu_boost', 'warmup_held',
         'jump_filter', 'start_centre', 'dash_turn_end', 'dash_turn', 'heading_hold', 'upper_single', 'lane_segment', 'downhill_approach', 'crest_marks',
         'upper_lateral', 'solid_takeover', 'gain_schedule', 'heading_term', 'curve_map', 'cmd_comp', 'crossing_pass', 'curve_smooth', 'yaw_inner',
         'straight_slew', 'gyro_damp', 'preview_lock', 'curve_ff', 'flat_dip_guard', 'upper_path', 'speed_adapt', 'lane_lock_pwm', 'lock_slow',
         'brake_zones', 'turn_pwm', 'down_coast', 'downhill_dcap', 'downhill_speed', 'downhill_assist', 'downhill_dlimit', 'downhill_gz', 'down_straight',
         'crossing_hold', 'downhill_bottom', 'bottom_left_floor', 'bottom_commit', 'jump_guard', 'diff_assist', 'slope_pre_right', 'bottom_map',
         'bottom_curve', 'curve_hold', 'lost_s', 'last_steer_us', 'hold_frames', 'hold_s', 'play_comp', 'steer_us', 'pwm', 'reason', 'valid']
RANK = {k: i for i, k in enumerate(ORDER)}


def load(path):
    with open(path, encoding='utf8') as f:
        meta = json.loads(f.readline()); rows = [json.loads(l) for l in f]
    return meta, rows


def num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def diff_paths(a, b, tol=0., path=''):
    """paths where a and b differ (floats compared with absolute tolerance tol; tol=0 -> exact)."""
    out = []
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b), key=str):
            if k not in a: out.append(path + '/' + str(k) + ' (only new)')
            elif k not in b: out.append(path + '/' + str(k) + ' (only old)')
            else: out += diff_paths(a[k], b[k], tol, path + '/' + str(k))
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b): out.append(path + ' (len)')
        else:
            for i, (x, y) in enumerate(zip(a, b)): out += diff_paths(x, y, tol, path + '[%d]' % i)
    elif num(a) and num(b):
        fa, fb = float(a), float(b)
        if not (fa == fb or (math.isfinite(fa) and math.isfinite(fb) and abs(fa - fb) <= tol)): out.append(path)
    elif a != b: out.append(path)
    return out


def first_layer(paths):
    keys = [p.split('/')[1].split(' ')[0].split('[')[0] for p in paths]
    return min(keys, key=lambda k: RANK.get(k, 999)) if keys else None


def q(vals, ps=(.5, .9, .99, 1.)):
    if not vals: return {}
    s = sorted(vals); n = len(s)
    return {('p%g' % (p * 100) if p < 1 else 'max'): round(s[min(n - 1, int(p * (n - 1) + .5))], 4) for p in ps}
