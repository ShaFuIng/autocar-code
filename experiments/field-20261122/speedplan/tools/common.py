# -*- coding: utf-8 -*-
"""speedplan 共用：讀 extract.py 的 pickle、重算 MCU 車速、找地標通過里程、切路段。"""
import os, pickle, bisect
import numpy as np

SCR = os.environ.get('SP_SCR', r'C:\Users\User\AppData\Local\Temp\claude\1eeee7db7bd1a8b1\02d05eda-955f-4b7f-90df-1bfa92030e70\scratchpad\sp')
MPC = 0.000222   # m / count（lap_pos.json m_per_count；6492 cps = 1.44 m/s）
VMAX = 1.44
ORDER = ['C1s', 'C1e', 'C2s', 'C2e', 'C3s', 'RAMP', 'TOP', 'DASHs', 'DASHe', 'DOWN', 'LAND8', 'C4s', 'C4e']

# 路段＝(名稱, 起點地標, 起點偏移 m, 終點地標, 終點偏移 m)。終點地標 'C1s+' = 下一圈的 C1s。
SEGS = [
    ('彎1',        'C1s', 0.0, 'C1e', 0.0),
    ('斜線',       'C1e', 0.0, 'C2s', 0.0),
    ('彎2',        'C2s', 0.0, 'C2e', 0.0),
    ('圓圈直線',   'C2e', 0.0, 'C2e', 6.8),
    ('左彎',       'C2e', 6.8, 'C3s', 0.0),
    ('彎3前段',    'C3s', 0.0, 'C3s', 4.3),
    ('彎3拐角',    'C3s', 4.3, 'C3s', 5.5),
    ('彎3尾段',    'C3s', 5.5, 'RAMP', 0.0),
    ('上坡',       'RAMP', 0.0, 'TOP', 0.0),
    ('坡頂',       'TOP', 0.0, 'DASHs', 0.0),
    ('虛線轉彎',   'DASHs', 0.0, 'DASHe', 0.0),
    ('上層直線',   'DASHe', 0.0, 'DOWN', 0.0),
    ('下坡',       'DOWN', 0.0, 'LAND8', 0.0),
    ('落地0-1.5',  'LAND8', 0.0, 'LAND8', 1.5),
    ('彎4前段',    'LAND8', 1.5, 'LAND8', 3.4),
    ('彎4後半',    'LAND8', 3.4, 'C4e', 0.0),
    ('主直線',     'C4e', 0.0, 'C1s+', 0.0),
]


def load(run):
    with open(os.path.join(SCR, run + '.pkl'), 'rb') as f: R = pickle.load(f)
    R = [r for r in R if r['t'] is not None]
    R.sort(key=lambda r: r['t'])
    mcu_speed(R)
    cont_odo(R)
    return R


def cont_odo(R):
    """連續里程 r['x']（lap_pos odo 單位）：MCU 狀態第一次出現的那格 Orin 時間 t 與 enc*MPC+常數，逐格線性內插。
    lap_pos 的 odo_m 與 enc_b*MPC 只差常數（e69d：−1323.8411±0.0005 m），但它是 5 Hz 階梯。"""
    off = [r['odo'] - r['enc'] * MPC for r in R if r['odo'] is not None and r['enc'] is not None]
    c = float(np.median(off)) if off else 0.
    ts = []; xs = []; last = None
    for r in R:
        if r['ms'] is None or r['enc'] is None or r['ms'] == last: continue
        last = r['ms']; ts.append(r['t']); xs.append(r['enc'] * MPC + c)
    ts = np.array(ts); xs = np.maximum.accumulate(np.array(xs))
    for r in R: r['x'] = float(np.interp(r['t'], ts, xs)) if len(ts) else r['odo']
    global XT
    XT = (xs, ts)


XT = None


def mcu_speed(R, half=0.2):
    """每個 MCU 狀態（ms 變化）一筆 (ms, enc)；車速＝以該筆為中心 ±half 秒的計數差（離線可用置中）。結果寫入 r['v']。"""
    S = []
    for r in R:
        if r['ms'] is None or r['enc'] is None: continue
        if not S or r['ms'] != S[-1][0]: S.append((r['ms'], r['enc']))
    if len(S) < 3:
        for r in R: r['v'] = None
        return
    ms = np.array([s[0] for s in S], float) / 1000.; en = np.array([s[1] for s in S], float)
    V = np.full(len(S), np.nan)
    for i in range(len(S)):
        a = bisect.bisect_left(ms, ms[i] - half); b = bisect.bisect_right(ms, ms[i] + half) - 1
        if b > a and ms[b] - ms[a] > 0.1: V[i] = (en[b] - en[a]) * MPC / (ms[b] - ms[a])
    d = {S[i][0]: V[i] for i in range(len(S))}
    for r in R:
        x = d.get(r['ms']); r['v'] = None if x is None or np.isnan(x) else float(x)


def crossings(R):
    """時間序的地標通過清單 [(name, odo_cross, t_cross)]；odo_cross＝odo − since_anchor_m（同一次通過取中位），t 以 t 對 odo 內插。"""
    ev = []; cur = None; buf = []
    for r in R:
        a = r['anc'];
        if a is None or r['odo'] is None or r['sam'] is None: continue
        if a != cur:
            if cur is not None and buf: ev.append((cur, float(np.median(buf))))
            cur = a; buf = []
        buf.append(r['odo'] - r['sam'])
    if cur is not None and buf: ev.append((cur, float(np.median(buf))))
    # 合併：相鄰同名（lost 中斷）
    out = []
    for n, o in ev:
        if out and out[-1][0] == n and abs(out[-1][1] - o) < 1.0: continue
        out.append((n, o))
    Om, T = XT   # MCU 狀態樣本（里程, 到達時間）：在樣本間線性內插，不受 5 Hz 階梯量化
    return [(n, o, t_at(Om, T, o)) for n, o in out], (Om, T)


def t_at(Om, T, o):
    i = bisect.bisect_left(Om, o)
    if i <= 0: return float(T[0])
    if i >= len(Om): return float(T[-1])
    o0, o1 = Om[i - 1], Om[i]
    if o1 <= o0: return float(T[i])
    return float(T[i - 1] + (T[i] - T[i - 1]) * (o - o0) / (o1 - o0))


def laps(R):
    """以 C1s 切圈：回傳 [dict(lap=k, X={地標: odo}, t0, t1, odo0, odo1)]；X 只收該圈 C1s 之後到下一個 C1s 之前的地標。
    lap 0＝起跑到第一個 C1s。"""
    ev, (Om, T) = crossings(R)
    L = []; cur = dict(lap=0, X={}, odo0=float(Om[0]), t0=float(T[0]))
    for n, o, t in ev:
        if n == 'C1s':
            cur['odo1'] = o; cur['t1'] = t; L.append(cur)
            cur = dict(lap=len(L), X={'C1s': o}, odo0=o, t0=t)
        else:
            if n not in cur['X']: cur['X'][n] = o
    cur['odo1'] = None; cur['t1'] = None; L.append(cur)
    for i, l in enumerate(L):
        l['next_C1s'] = L[i + 1]['X'].get('C1s') if i + 1 < len(L) else None
    return L, (Om, T)


def seg_bounds(lap, seg):
    name, a0, d0, a1, d1 = seg
    o0 = lap['X'].get(a0)
    o1 = lap['next_C1s'] if a1 == 'C1s+' else lap['X'].get(a1)
    if o0 is None or o1 is None: return None
    o0 += d0; o1 += d1
    if o1 <= o0: return None
    return o0, o1


def frames_in(R, o0, o1):
    return [r for r in R if r.get('x') is not None and o0 <= r['x'] < o1]


def pct(a, q):
    a = [x for x in a if x is not None]
    return float(np.percentile(a, q)) if a else None
