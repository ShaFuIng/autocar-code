"""A. inventory: which layer decides steering / PWM in each zone; 5 runs, 24 slope passes (23 landings + 46c9 lap 2)."""
from desc import *
import statistics as S, collections as C
R = all_desc()
ZON = [('上層直線末段（坡口前 1.5～0 m）', None), ('坡口（俯仰<-5 起～其後 0.4 m）', None), ('坡上（坡口後 0.4～1.5 m）', None), ('坡末（坡口後 1.5 m～落地）', None),
       ('落地 x 0～0.5', (0, 0.5)), ('落地 x 0.5～1.0', (0.5, 1.0)), ('落地 x 1.0～1.5', (1.0, 1.5)), ('落地 x 1.5～2.3', (1.5, 2.3)), ('落地 x 2.3～2.8', (2.3, 2.8)),
       ('落地 x 2.8～3.5', (2.8, 3.5)), ('落地 x 3.5～5.0', (3.5, 5.0))]
Z = [C.defaultdict(list) for _ in ZON]
LAY = ('down_straight', 'jump_guard', 'downhill_gz', 'downhill_speed', 'downhill_dlimit', 'downhill_assist', 'downhill_dcap', 'arc2_land', 'arc2_land_lost', 'bottom_owner',
       'lap_ff4', 'curve_ff', 'lap_ff4_lost', 'downhill_bottom', 'bottom_left_floor', 'yaw_inner', 'upper_single', 'curve_map', 'bottom_guide')


def zone(d, n):
    a = d['a']; im = d['i_mouth'] - a; il = None if d['i_land'] is None else d['i_land'] - a
    if n < im:
        dn = d['dist'](d['t'][n])
        dm = d['dist'](d['t'][im]) - dn if dn is not None else None
        return 0 if dm is not None and dm <= 1.5 else None
    if il is None or n < il:
        dd = d['dist'](d['t'][n]) - d['dist'](d['t'][im])
        return 1 if dd < 0.4 else (2 if dd < 1.5 else 3)
    x = d['x'][n]
    for k, (nm, rg) in enumerate(ZON):
        if rg and rg[0] <= x < rg[1]: return k
    return None


def acted(l, v):
    if l == 'arc2_land': return bool(v.get('applied') or (v.get('land') or {}).get('applied') or (v.get('pulse') or {}).get('applied'))
    if l == 'lap_ff4': return bool(v.get('applied'))
    if l == 'bottom_owner': return v.get('cap') is not None or v.get('mode') == 'RATE_HOLD'
    if l == 'bottom_guide': return v.get('mode') == 'PROFILE_HOLD'
    if l == 'yaw_inner': return v.get('reason') == 'OK' and abs(v.get('term_us') or 0) > 0
    if l == 'upper_single': return bool(v.get('active'))
    if l == 'jump_guard': return 'held_us' in v
    if l == 'curve_map': return v.get('reason') in ('FLOOR', 'RELEASED')
    return True


for d in R:
    for n, p in enumerate(d['F']):
        k = zone(d, n)
        if k is None: continue
        z = Z[k]; z['n'].append(1); val = p.get('reason') == 'PROPOSAL_ONLY'; z['valid'].append(val)
        z['owner'].append(p.get('owner'))
        if p.get('steer_us') is not None: z['st'].append(p['steer_us'] - C0); z['sent'].append(sent(p) - C0)
        z['pwm'].append(p.get('pwm'))
        for l in LAY:
            v = p.get(l)
            if isinstance(v, dict) and acted(l, v): z['L_' + l].append(1)
        b = g(p, 'lap_ff4', 'band', 'state')
        if b: z['band_' + b].append(1)
        sa = p.get('speed_adapt') or {}; bz = (p.get('brake_zones') or {})
        if not val: src = '丟線列（沿用）'
        elif bz.get('cap_pwm') is not None and p.get('pwm') is not None and abs(p['pwm'] - bz['cap_pwm']) < 0.01: src = '煞車區上限 %g' % bz['cap_pwm']
        elif isinstance(p.get('lap_speed'), dict) and p['lap_speed'].get('applied'): src = 'lap_speed 位置區'
        else: src = 'speed_adapt %s%s' % (sa.get('reason'), '（落地規則）' if 'landing' in sa else '')
        z['pwmsrc'].append(src)


def pct(v, q): v = sorted(v); return v[min(len(v) - 1, int(q * len(v)))]


for k, (nm, _) in enumerate(ZON):
    z = Z[k]; n = len(z['n'])
    print('== %s　格數 %d　有效格 %.0f%%' % (nm, n, 100 * sum(z['valid']) / n))
    print('   最後寫入舵量者：', '、'.join('%s %.0f%%' % (o, 100 * c / n) for o, c in C.Counter(z['owner']).most_common(6)))
    print('   有動作的層（占格數）：', '、'.join('%s %.0f%%' % (l[2:], 100 * len(z[l]) / n) for l in sorted([x for x in z if x.startswith('L_')], key=lambda l: -len(z[l]))))
    bs = [x for x in z if x.startswith('band_')]
    if bs: print('   彎 4 帶狀（lap_ff4.band）狀態：', '、'.join('%s %.0f%%' % (l[5:], 100 * len(z[l]) / n) for l in bs))
    print('   補償前舵量−中心 P10／中位／P90：%.0f／%.0f／%.0f；實送−中心：%.0f／%.0f／%.0f' % (pct(z['st'], .1), pct(z['st'], .5), pct(z['st'], .9), pct(z['sent'], .1), pct(z['sent'], .5), pct(z['sent'], .9)))
    pw = [x for x in z['pwm'] if x is not None]
    print('   油門 P10／中位／P90：%.0f／%.0f／%.0f；來源：' % (pct(pw, .1), pct(pw, .5), pct(pw, .9)), '、'.join('%s %.0f%%' % (o, 100 * c / n) for o, c in C.Counter(z['pwmsrc']).most_common(5)))

print('\n== 交接處的舵量不連續（補償前指令，相鄰兩格的差；每次下坡取一筆）')


def st(p): return p.get('steer_us')


def jump(name, fn):
    v = [x for x in (fn(d) for d in R) if x is not None]
    if v: print('  %-40s n %2d　|差| 中位 %3.0f　P90 %3.0f　最大 %3.0f　（有號中位 %+.0f）' % (name, len(v), S.median([abs(x) for x in v]), pct([abs(x) for x in v], .9), max(abs(x) for x in v), S.median(v)))


def j_mouth_max(d):
    F = d['F']; im = d['i_mouth'] - d['a']; return max((st(F[n]) - st(F[n - 1]) for n in range(im, im + 6)), key=abs)


def j_land(d):
    if d['i_land'] is None: return None
    F = d['F']; il = d['i_land'] - d['a']; return max((st(F[n]) - st(F[n - 1]) for n in range(il, il + 8)), key=abs)


def j_lost(d):
    F = d['F']; a = d['a']; im = d['i_mouth'] - a; il = (d['i_land'] - a) if d['i_land'] is not None else len(F) - 1; m = None
    for n in range(im + 1, min(len(F), il + 3)):
        if (F[n].get('reason') == 'PROPOSAL_ONLY') != (F[n - 1].get('reason') == 'PROPOSAL_ONLY'):
            x = st(F[n]) - st(F[n - 1]); m = x if m is None or abs(x) > abs(m) else m
    return m


def j_at(key):
    def f(d):
        if d['i_land'] is None: return None
        F = d['F']; il = d['i_land'] - d['a']
        for n in range(il + 1, len(F)):
            if key(F[n]) and not key(F[n - 1]): return st(F[n]) - st(F[n - 1])
        return None
    return f


jump('坡口：俯仰<-5 起 6 格內最大一步', j_mouth_max); jump('坡上：有效↔丟線切換的最大一步', j_lost); jump('落地偵測後 8 格內最大一步', j_land)
jump('bottom_owner CAP→RAMP', j_at(lambda p: g(p, 'bottom_owner', 'phase') == 'RAMP')); jump('彎 4 帶狀 BEFORE_X0→ON', j_at(lambda p: g(p, 'lap_ff4', 'band', 'state') == 'ON'))
jump('arc2 落地帶狀結束（HANDED_OVER）', j_at(lambda p: g(p, 'arc2_land', 'state') == 'END')); jump('彎道地圖 FLOOR（bottom_owner 範圍結束）', j_at(lambda p: g(p, 'curve_map', 'reason') == 'FLOOR'))


def xat(key):
    v = []
    for d in R:
        if d['i_land'] is None: continue
        il = d['i_land'] - d['a']
        for n in range(il, len(d['F'])):
            if key(d['F'][n]): v.append(d['x'][n]); break
    return '%.2f～%.2f（中位 %.2f，n %d）' % (min(v), max(v), S.median(v), len(v))


print('\n== 各層交接位置 x（落地偵測後的公尺數，後輪編碼器內插）')
print('  bottom_owner CAP 結束→RAMP：', xat(lambda p: g(p, 'bottom_owner', 'phase') == 'RAMP')); print('  bottom_owner OPEN：', xat(lambda p: g(p, 'bottom_owner', 'phase') == 'OPEN'))
print('  彎 4 帶狀開始作用（state ON）：', xat(lambda p: g(p, 'lap_ff4', 'band', 'state') == 'ON')); print('  彎 4 帶狀全權（w=1）：', xat(lambda p: (g(p, 'lap_ff4', 'band', 'w') or 0) >= 1))
print('  arc2 落地帶狀開始交還（HAND）：', xat(lambda p: g(p, 'arc2_land', 'state') == 'HAND')); print('  arc2 落地帶狀結束（END）：', xat(lambda p: g(p, 'arc2_land', 'state') == 'END'))
print('  彎道地圖 FLOOR（＝bottom_owner 範圍結束、彎 4 專用油門開始）：', xat(lambda p: g(p, 'curve_map', 'reason') == 'FLOOR'))
def xat2(key, x0):
    v = []
    for d in R:
        if d['i_land'] is None: continue
        il = d['i_land'] - d['a']
        for n in range(il, len(d['F'])):
            if d['x'][n] >= x0 and key(d['F'][n]): v.append(d['x'][n]); break
    return '%.2f～%.2f（中位 %.2f，n %d）' % (min(v), max(v), S.median(v), len(v))


print('  speed_adapt 落地規則結束（x>=1 之後第一個沒有 landing 欄的有效格）：', xat2(lambda p: p.get('reason') == 'PROPOSAL_ONLY' and isinstance(p.get('speed_adapt'), dict) and 'landing' not in p['speed_adapt'], 1.0))
print('  煞車區「坡底→彎 4」上限 31 結束（x>=0.5 之後第一個不被它夾住的有效格）：', xat2(lambda p: p.get('reason') == 'PROPOSAL_ONLY' and not ((p.get('brake_zones') or {}).get('cap_pwm') == 31 and abs((p.get('pwm') or 0) - 31) < 0.01), 0.5))
print('  油門 ≥45：', xat(lambda p: (p.get('pwm') or 0) >= 45))
# band: how often it changes the command in 2.3..3.5
nb = 0; na = 0
for d in R:
    if d['i_land'] is None: continue
    for n in range(d['i_land'] - d['a'], len(d['F'])):
        p = d['F'][n]; x = d['x'][n]
        if x is not None and 2.6 <= x <= 3.5 and g(p, 'lap_ff4', 'band', 'state') == 'ON':
            nb += 1; na += bool(g(p, 'lap_ff4', 'applied'))
print('\n  x 2.6～3.5 彎 4 帶狀 ON 的格 %d，其中帶狀實際改動舵量 %d 格（%.0f%%）' % (nb, na, 100. * na / max(1, nb)))
