# -*- coding: utf-8 -*-
"""開迴路省時估計（一階車速模型，c4fast 同表同時間常數）：
  vss(pwm)＝speed_guard.json overspeed_watch.table（≤50）＋架空比例（55～70）＋落地 100%＝6492 cps；
  dv/dt＝(k·vss(pwm)−v)/τ（加速 τ_up 0.405 s、減速 τ_down 0.631 s）；k 逐圈逐區以基準重建擬合（吸收彎道阻力、坡度）。
  油門以「距離→該處那一格的油門」重建；候選規則把每格改成新油門（模仿設定改動後各層的結果，不模擬 speed_adapt 內部狀態）。
  輸出：每圈 模型基準時間、候選時間、差；入彎點車速變化。只估計，不是重播；閉迴路（誤差、路線變化）完全沒有模擬。
用法：python sim.py <step名稱|all> [run ...]"""
import sys, json, collections
import numpy as np
from common import *

SNAP = r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments\orin-web-snapshot-20261122c'
SG = json.load(open(SNAP + r'\speed_guard.json', encoding='utf8'))['overspeed_watch']
TAB = [tuple(x) for x in SG['table']]; V50 = TAB[-1][1]
HI = [(55., 1.034 * V50), (60., 1.057 * V50), (65., 1.079 * V50), (70., 1.098 * V50), (100., 6492.)]
PW = np.array([x[0] for x in TAB] + [x[0] for x in HI]); CP = np.array([x[1] for x in TAB] + [x[1] for x in HI])
TU, TD = float(SG['tau_up_s']), float(SG['tau_down_s'])
PATH_LOW = ('ERROR_LARGE', 'CURVE_AHEAD', 'STEERING_LARGE', 'ERROR_RATE_LARGE')


def vss(p): return float(np.interp(p, PW, CP)) * MPC


def ls_ok(r, eg, sg, allow_low=True):
    """lap_speed 區能否抬這一格（lap_speed.blocked 的近似：只看紀錄裡有的欄位）"""
    if r['reason'] != 'PROPOSAL_ONLY': return False
    if r['sa_r'] in ('SPECIAL_CONTROL', 'DOWNHILL', 'AFTER_CREST', 'NO_ERROR', None): return False
    if r['sa_land'] or r['sa_up'] is not None or r['sa_dhe'] or r['sa_crest']: return False
    if r['sa_l'] == 'LOW' and not (allow_low and r['sa_r'] in PATH_LOW and not r['sa_ws']): return False
    if r['e'] is None or abs(r['e']) > eg: return False
    if r['st_raw'] is None or abs(r['st_raw'] - 1475) > sg: return False
    if r['pitch'] is not None and abs(r['pitch']) > 5: return False
    return True


def pre_cap(r):
    """brake zone 之前的油門（被 brake zone 壓過的格用 pwm_before）"""
    return float(r['bz_bef']) if r['bz_bef'] is not None else float(r['pwm'] or 0)


# ---- 各步驟的規則：f(r, D) -> 新油門；D＝本圈各地標里程（含上一圈 C4e，key 'pC4e'）
def d_of(r, D, a):
    o = D.get(a)
    return None if o is None else r['x'] - o


def bz_recap(r, names, cap, shift=None, D=None):
    """把 names 這些煞車區的上限改成 cap（shift＝(錨, 舊起點, 新起點)：舊起點～新起點之間不再壓）"""
    if r['bz'] in names and r['bz_bef'] is not None:
        if shift is not None:
            a, f0, f1 = shift; d = d_of(r, D, a)
            if d is not None and f0 - 0.05 <= d < f1: return float(r['bz_bef'])
        return min(float(r['bz_bef']), cap)
    return None


STEPS = collections.OrderedDict()


def step(name, region, desc):
    def deco(f): STEPS[name] = (region, desc, f); return f
    return deco


@step('c1in_38', ('pC4e', 6.0, 'C1s', 3.0, 'C1s', 0.0), '彎1入口：BZ_c1 上限 32→38＋C4e+9.4～12.0 銜接區 38（allow_low、err 0.75、steer 400）')
def _(r, D):
    p = float(r['pwm'] or 0)
    q = bz_recap(r, ('BZ_c1',), 38.)
    if q is not None: p = q
    d = d_of(r, D, 'pC4e'); dc = d_of(r, D, 'C1s')
    if d is not None and 9.4 <= d <= 12.0 and (dc is None or dc <= 0.4) and ls_ok(r, 0.75, 400) and r['sa_c'] is None:
        p = max(p, 38.)
        if r['bz'] == 'BZ_c1' or (r['bz_cap'] is not None and r['bz'] == 'BZ_c1'): p = min(p, 38.)
    return p


@step('c1in_42', ('pC4e', 6.0, 'C1s', 3.0, 'C1s', 0.0), '彎1入口（積極）：BZ_c1 上限 32→42、起點後移 0.4 m＋銜接區 42')
def _(r, D):
    p = float(r['pwm'] or 0)
    q = bz_recap(r, ('BZ_c1',), 42., ('DOWN', 19.2, 19.6), D)
    if q is not None: p = q
    d = d_of(r, D, 'pC4e'); dc = d_of(r, D, 'C1s')
    if d is not None and 9.4 <= d <= 12.0 and (dc is None or dc <= 0.4) and ls_ok(r, 0.75, 400) and r['sa_c'] is None:
        p = max(p, 42.)
        if r['bz'] == 'BZ_c1': p = min(p, 42.)
    return p


@step('c2in_33', ('C1e', 7.0, 'C2s', 2.0, 'C2s', 0.0), '彎2入口：BZ_c2／BZ_c2_lap1 上限 30→33（位置不動）')
def _(r, D):
    q = bz_recap(r, ('BZ_c2', 'BZ_c2_lap1'), 33.)
    return float(r['pwm'] or 0) if q is None else q


@step('c2in_34s', ('C1e', 7.0, 'C2s', 2.0, 'C2s', 0.0), '彎2入口（積極）：上限 30→34、C1e 區起點 10.2→10.5')
def _(r, D):
    q = bz_recap(r, ('BZ_c2', 'BZ_c2_lap1'), 34., ('C1e', 10.2, 10.5), D)
    return float(r['pwm'] or 0) if q is None else q


def left(r, D, zp, cap):
    pre = pre_cap(r); p = float(r['pwm'] or 0)
    d = d_of(r, D, 'C2e'); d3 = d_of(r, D, 'C3s')
    raised = False
    if d is not None and 6.6 <= d <= 10.5 and (d3 is None or d3 < 0.3) and ls_ok(r, 0.8, 350):
        pre = max(pre, zp); raised = True
    if r['bz'] == 'BZ_c3entry': return min(pre, cap)
    return pre if raised else p


@step('left_34', ('C2e', 5.0, 'C3s', 2.0, 'C3s', 0.0), '左彎→彎3入口：新 lap_speed 區 C2e+6.6～10.5 油門 34（allow_low）＋BZ_c3entry 上限 30→34')
def _(r, D): return left(r, D, 34., 34.)


@step('left_38', ('C2e', 5.0, 'C3s', 2.0, 'C3s', 0.0), '左彎→彎3入口（積極）：區 38＋BZ_c3entry 上限 38')
def _(r, D): return left(r, D, 38., 38.)


def crest(r, D, zp):
    p = float(r['pwm'] or 0)
    if r['ls'] == 'LS_crest27' and not r['ls_blk']:
        q = max(p, zp)
        if r['llp'] is not None: q = min(q, float(r['llp']))
        if r['bz_cap'] is not None: q = min(q, float(r['bz_cap']))
        return q
    return p


@step('crest_33', ('RAMP', 1.5, 'DASHs', 0.6, 'DASHs', 0.0), '坡頂：LS_crest27 區油門 27→33')
def _(r, D): return crest(r, D, 33.)


@step('crest_36', ('RAMP', 1.5, 'DASHs', 0.6, 'DASHs', 0.0), '坡頂（積極）：27→36')
def _(r, D): return crest(r, D, 36.)


def c4(r, D, zp):
    p = float(r['pwm'] or 0)
    if r['ls'] == 'LS_c4_58' and not r['ls_blk'] and p >= 50: return max(p, zp) if p >= 57.5 else p + (zp - 58.) * max(0., (p - 48.) / 10.)
    return p


@step('c4_64', ('LAND8', 2.5, 'C4e', 1.5, 'C4e', 0.0), '彎4後半：LS_c4_58 油門 58→64')
def _(r, D): return c4(r, D, 64.)


@step('c4_70', ('LAND8', 2.5, 'C4e', 1.5, 'C4e', 0.0), '彎4後半（積極）：58→70（＝c4fast step70）')
def _(r, D): return c4(r, D, 70.)


def min28(r, D):
    p = float(r['pwm'] or 0)
    if r['reason'] == 'PROPOSAL_ONLY' and r['sa_l'] == 'LOW' and r['sa_r'] not in ('DOWNHILL',) and p < 27.9 and not r['sa_dhe']:
        lim = 28. if r['sa_up'] is None else min(28., float(r['sa_up']))
        if r['tpc'] is not None: lim = min(lim, float(r['tpc']))
        if r['bz_cap'] is not None: lim = min(lim, float(r['bz_cap']))
        return max(p, lim)
    return p


for _nm, _rg, _ds in (('min28_left', ('C2e', 5.0, 'C3s', 2.0, 'C3s', 0.0), '左彎'), ('min28_upper', ('DASHs', 0.0, 'DOWN', 0.5, 'DOWN', 0.0), '虛線轉彎後'),
                      ('min28_c1', ('pC4e', 6.0, 'C1s', 3.0, 'C1s', 0.0), '彎1入口'), ('min28_c2', ('C1e', 7.0, 'C2s', 2.0, 'C2s', 0.0), '彎2入口'),
                      ('min28_c3', ('C3s', 0.0, 'TOP', 0.0, 'RAMP', 0.0), '彎3～上坡'), ('min28_main', ('C4e', 0.0, 'C4e', 8.0, 'C4e', 8.0), '主直線'),
                      ('min28_diag', ('C1e', 0.0, 'C1e', 9.0, 'C1e', 9.0), '斜線'), ('min28_circ', ('C2e', 0.0, 'C2e', 5.0, 'C2e', 5.0), '圓圈直線')):
    STEPS[_nm] = (_rg, '全圈 min_pwm 24→28 的一部分：' + _ds, min28)


def c2br(r, D, zp, cap):
    pre = pre_cap(r); p = float(r['pwm'] or 0); raised = False
    d = d_of(r, D, 'C1e'); d2 = d_of(r, D, 'C2s')
    if d is not None and 10.6 <= d <= 12.8 and (d2 is None or d2 <= 0.58) and r['sa_c'] is None and ls_ok(r, 0.75, 400):
        pre = max(pre, zp); raised = True
    if r['bz'] in ('BZ_c2', 'BZ_c2_lap1'): return min(pre, cap)
    return pre if raised else p


@step('c2br_33', ('C1e', 7.0, 'C2s', 2.0, 'C2s', 0.0), '彎2入口：BZ_c2 上限 30→33＋C1e+10.6～12.8 銜接區 33（allow_low）')
def _(r, D): return c2br(r, D, 33., 33.)


@step('tail_end17', ('C3s', 3.0, 'TOP', 0.0, 'RAMP', 0.8), '坡腳：BZ_c3tail 終點 C2e+17.4→17.0（坡腳最後約 0.4 m 交回 speed_adapt 上坡下限 40）')
def _(r, D):
    p = float(r['pwm'] or 0)
    d = d_of(r, D, 'C2e')
    if r['bz'] == 'BZ_c3tail' and r['bz_bef'] is not None and d is not None and d > 17.0: return float(r['bz_bef'])
    return p


@step('ramp_rate200', ('RAMP', -0.5, 'TOP', 0.0, 'RAMP', 1.2), '（需換 lap_speed.py r3）上坡 100 區每區爬升率 200 PWM/s（近似：區內未擋格直接 100）')
def _(r, D):
    p = float(r['pwm'] or 0)
    if r['ls'] == 'LS_ramp100' and not r['ls_blk']: return 100.
    return p


def prep_rate(R):
    """每格記下目前這段「同一 lap_speed 區、連續未被擋」的起始時間 _t0 與區目標 _tg（rate 加倍的近似用）"""
    t0 = None; z = None
    for r in R:
        if r['ls'] and not r['ls_blk'] and r['reason'] == 'PROPOSAL_ONLY':
            if z != r['ls'] or t0 is None: t0 = r['t']; z = r['ls']
            r['_t0'] = t0
        else:
            t0 = None; z = None; r['_t0'] = None


TGT = {'LS_main100': 100, 'LS_diag100': 100, 'LS_circ100': 100, 'LS_ramp100': 100, 'LS_c4_58': 58, 'LS_c3front48': 48, 'LS_crest27': 27,
       'LS_main50': 50, 'LS_diag50': 50, 'LS_circ50': 50}


def rate120(r, D):
    p = float(r['pwm'] or 0)
    if r.get('_t0') is not None and r['ls'] in TGT and not r['ls_tp']:
        tg = TGT[r['ls']]
        if p < tg - 0.01:
            q = min(tg, p + 60. * (r['t'] - r['_t0']))
            if r['bz_cap'] is not None: q = min(q, float(r['bz_cap']))
            if r['llp'] is not None: q = min(q, float(r['llp']))
            return max(p, q)
    return p


rate120.prep = prep_rate
for _nm, _rg, _ds in (('rate120_main', ('C4e', 0.0, 'C4e', 4.0, 'C4e', 2.0), '主直線進區'), ('rate120_diag', ('C1e', 0.0, 'C1e', 4.0, 'C1e', 2.0), '斜線進區'),
                      ('rate120_circ', ('C2e', 0.0, 'C2e', 4.0, 'C2e', 2.0), '圓圈直線進區'), ('rate120_ramp', ('RAMP', -0.5, 'TOP', 0.0, 'RAMP', 1.2), '上坡 100 區（動到上坡）'),
                      ('rate120_c3f', ('C3s', 0.0, 'C3s', 4.3, 'C3s', 2.0), '彎3前段 48 區'), ('rate120_c4', ('LAND8', 3.0, 'C4e', 0.5, 'C4e', 0.0), '彎4後半 58 區')):
    STEPS[_nm] = (_rg, 'lap_speed.json 全域 rate_pwm_s 60→120 的一部分：' + _ds, rate120)


@step('bottom_40', ('DOWN', 1.5, 'LAND8', 3.4, 'LAND8', 1.5), '落地：BZ_bottom 上限 36→40')
def _(r, D):
    q = bz_recap(r, ('BZ_bottom',), 40.)
    return float(r['pwm'] or 0) if q is None else q


@step('post_46', ('DASHe', 0.0, 'DOWN', 0.5, 'DOWN', 0.0), '上層轉彎後直線：upper_wide post_pwm 40→46（下坡口上限 34 不動）')
def _(r, D):
    p = float(r['pwm'] or 0)
    if r['llp'] == 40.0 and abs(p - 40.) < 0.01 and r['bz_bef'] is None and r['tpc'] is None:
        d = d_of(r, D, 'DASHe')
        if d is not None and d >= 0: return 46.
    return p


def region_bounds(lap, prev, reg):
    a0, f0, a1, f1 = reg[:4]
    def get(a):
        if a == 'pC4e': return prev['X'].get('C4e') if prev else None
        if a == 'C1s+': return lap['next_C1s']
        return lap['X'].get(a)
    o0 = get(a0); o1 = get(a1)
    if o0 is None or o1 is None: return None
    return o0 + f0, o1 + f1


def simulate(xs, ps, x0, x1, v0, k, dt=0.005):
    """距離域積分：回傳 (時間, 函數 v(x) 取樣)"""
    x = x0; v = max(0.2, v0); t = 0.; i = int(np.searchsorted(xs, x, 'right')) - 1
    X = [x]; V = [v]
    while x < x1 and t < 30:
        while i + 1 < len(xs) and xs[i + 1] <= x: i += 1
        tgt = k * vss(ps[max(i, 0)]); tau = TU if tgt > v else TD
        v += (tgt - v) * (1 - np.exp(-dt / tau)); x += v * dt; t += dt
        X.append(x); V.append(v)
    return t, np.array(X), np.array(V)


def run_step(name, runs, verbose=True):
    reg, desc, f = STEPS[name]
    out = []
    for run in runs:
        R = load(run); L, _ = laps(R)
        if hasattr(f, 'prep'): f.prep(R)
        xs_all = np.array([r['x'] for r in R])
        for li, lap in enumerate(L):
            if lap['lap'] == 0 or lap['t1'] is None and reg[2] == 'C1s+': continue
            prev = L[li - 1] if li > 0 else None
            b = region_bounds(lap, prev, reg)
            if b is None: continue
            o0, o1 = b
            D = dict(lap['X']); D['pC4e'] = prev['X'].get('C4e') if prev else None
            if 'DOWN' not in D and prev: D['DOWN'] = prev['X'].get('DOWN')
            if reg[0] == 'pC4e' and prev and 'DOWN' in prev['X']: D['DOWN'] = prev['X']['DOWN']
            F = [r for r in R if o0 - 1.0 <= r['x'] <= o1 + 0.2]
            if len(F) < 10: continue
            xs = np.array([r['x'] for r in F]); p0 = np.array([float(r['pwm'] or 0) for r in F])
            p1 = np.array([f(r, D) for r in F])
            vm = [(r['x'], r['v']) for r in F if r['v'] is not None and o0 <= r['x'] <= o1]
            v0 = next((r['v'] for r in F if r['x'] >= o0 and r['v'] is not None), None)
            if v0 is None or len(vm) < 5: continue
            best = None
            for k in np.arange(0.80, 1.30, 0.01):
                t, X, V = simulate(xs, p0, o0, o1, v0, k)
                err = np.mean([(np.interp(x, X, V) - v) ** 2 for x, v in vm])
                if best is None or err < best[0]: best = (err, k, t)
            err, k, t0 = best
            t1, X1, V1 = simulate(xs, p1, o0, o1, v0, k)
            _, X0, V0 = simulate(xs, p0, o0, o1, v0, k)
            meas_t = t_at(*XT_of(R), o1) - t_at(*XT_of(R), o0)
            changed = int(np.sum(np.abs(p1 - p0) > 0.01))
            vmax_d = float(np.max(np.interp(X1, X0, V0, right=V0[-1]) * 0 + V1) - np.max(V0))
            pr = None
            if len(reg) > 4:
                po = (D.get(reg[4]) if reg[4] != 'pC4e' else D.get('pC4e'))
                if po is not None:
                    xp = po + reg[5]
                    if X0[0] <= xp <= X0[-1]: pr = float(np.interp(xp, X1, V1) - np.interp(xp, X0, V0))
            out.append(dict(run=run, lap=lap['lap'], k=k, dv_probe=pr, rms=err ** .5, T_meas=meas_t, T0=t0, T1=t1, gain=t0 - t1, changed=changed, n=len(F),
                            dv_end=float(V1[-1] - V0[-1]), dv_max=vmax_d, X0=X0, V0=V0, X1=X1, V1=V1, o0=o0))
    if verbose:
        print('=' * 120); print('%s：%s  區間 %s' % (name, desc, reg))
        for o in out:
            print('  %s L%d  k %.2f 擬合均方根 %.3f m/s  實測 %.2f s  模型基準 %.2f  候選 %.2f  省 %+.3f s  改變 %d/%d 格  區尾車速 %+.3f  觀測點車速 %s' % (
                o['run'], o['lap'], o['k'], o['rms'], o['T_meas'], o['T0'], o['T1'], o['gain'], o['changed'], o['n'], o['dv_end'], '—' if o['dv_probe'] is None else '%+.3f' % o['dv_probe']))
        g = [o['gain'] for o in out]
        pv = [o['dv_probe'] for o in out if o['dv_probe'] is not None]
        if g: print('  → 每次省 中位 %.3f s（範圍 %.3f～%.3f，n %d）；觀測點 %s%+.1f 車速變化 中位 %s 最大 %s' % (np.median(g), min(g), max(g), len(g), reg[4] if len(reg) > 4 else '', reg[5] if len(reg) > 5 else 0, ('%+.3f' % np.median(pv)) if pv else '—', ('%+.3f' % max(pv)) if pv else '—'))
    return out


def XT_of(R):
    import common
    return common.XT


if __name__ == '__main__':
    name = sys.argv[1]; runs = sys.argv[2:] or ['e69d', '9af6', '54cb']
    for n in (STEPS if name == 'all' else [name]): run_step(n, runs)
