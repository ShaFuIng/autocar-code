"""curve-1 / curve-2 passes as arrays (shared by the c1exit scripts; laptop only)."""
import numpy as np, lib
from lib import g
C = lib.CENTRE
F = ['t', 'gz', 'e', 'ectl', 'pre', 'sent', 'P', 'I', 'D', 'des', 'y', 'psi', 'cin', 'cout', 'pwm', 'x', 'turned_cm', 'valid', 'yin', 'cffd', 'lffd', 'ctr', 'gk', 'pcdir', 'ccomp', 'gs_in', 'cs_out', 'yin_in', 'cff_in', 'lff_in']


def run_arrays(run):
    Q = lib.load(run); n = len(Q)
    A = {k: np.full(n, np.nan) for k in F}
    for i, p in enumerate(Q):
        A['t'][i] = p['t']; A['gz'][i] = g(p, 'inputs', 'gz'); A['e'][i] = g(p, 'image_error'); A['ectl'][i] = g(p, 'error')
        A['pre'][i] = g(p, 'steer_us') - C; A['sent'][i] = g(p, 'play_comp', 'steer_us') - C
        A['P'][i] = g(p, 'p'); A['I'][i] = g(p, 'i'); A['D'][i] = g(p, 'd'); A['des'][i] = g(p, 'desired_us') - C
        A['y'][i] = g(p, 'arc_lat', 'y_cm'); A['psi'][i] = g(p, 'arc_lat', 'psi_deg'); A['cin'][i] = g(p, 'arc_lat', 'clr_in_cm'); A['cout'][i] = g(p, 'arc_lat', 'clr_out_cm')
        A['pwm'][i] = g(p, 'pwm'); A['x'][i] = g(p, 'lap_pos', 'since_anchor_m'); A['turned_cm'][i] = g(p, 'curve_map', 'turned_deg'); A['valid'][i] = 1. if p.get('valid') else 0.
        A['yin'][i] = g(p, 'yaw_inner', 'term_us', default=0.); A['gk'][i] = g(p, 'gain_schedule', 'k', default=1.)
        A['gs_in'][i] = g(p, 'gain_schedule', 'pid_steer_us') - C; A['cs_out'][i] = g(p, 'curve_smooth', 'steer_us') - C; A['yin_in'][i] = g(p, 'yaw_inner', 'pid_steer_us') - C
        cf = p.get('curve_ff') or {}; A['cffd'][i] = (cf.get('floor_us') - cf.get('pid_steer_us')) if cf.get('applied') else 0.; A['cff_in'][i] = g(cf, 'pid_steer_us') - C
        lf = p.get('lap_ff') or {}; A['lffd'][i] = (lf.get('steer_us') - lf.get('pid_steer_us')) if lf.get('applied') else 0.; A['lff_in'][i] = g(lf, 'pid_steer_us') - C
        ct = p.get('curve_trim') or {}; A['ctr'][i] = (g(ct, 'pid_steer_us') - g(ct, 'steer_us')) if ('steer_us' in ct and 'pid_steer_us' in ct) else (ct.get('trim_us') or 0.)
        A['pcdir'][i] = g(p, 'play_comp', 'direction'); A['ccomp'][i] = g(p, 'cmd_comp', 'comp_us', default=0.)
    enc = np.array([g(p, 'inputs', 'enc_b') for p in Q], float); A['v'] = lib.speed(A['t'], enc); A['enc'] = enc
    A['anc'] = np.array([g(p, 'lap_pos', 'anchor', default='') or '' for p in Q]); A['lap'] = np.array([p['_lap'] for p in Q])
    A['seg'] = np.array([g(p, 'lap_pos', 'seg', default='') or '' for p in Q]); A['odo'] = np.array([g(p, 'lap_pos', 'odo_m') for p in Q], float)
    A['cmr'] = np.array([g(p, 'curve_map', 'reason', default='') or '' for p in Q]); A['owner'] = np.array([p.get('owner') or '' for p in Q])
    A['yaw'] = np.concatenate([[0], np.cumsum(np.nan_to_num(A['gz'][1:]) * np.diff(A['t']))])
    return Q, A


def exits(A, an='C1e'):
    return [i for i in range(1, len(A['t'])) if A['anc'][i] == an and A['anc'][i - 1] != an and A['t'][-1] - A['t'][i] > 3.0]


def straight_ref(A, i_exit, back_s=(6.0, 3.3)):
    """mean integrated yaw on the straight before the curve (a time window before the exit anchor) -> tb = ref - yaw (deg, right +)."""
    t = A['t']; m = (t >= t[i_exit] - back_s[0]) & (t <= t[i_exit] - back_s[1]) & (np.abs(A['gz']) < 8)
    return np.nanmean(A['yaw'][m]) if m.sum() > 5 else np.nan
