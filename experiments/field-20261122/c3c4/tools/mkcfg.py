"""c3c4: build every on/ config from the car snapshot 20261122 (and from the car's lap_ff.json 14c5e82c of 11-22 16:47 = field-20261122/ecturn/on/lap_ff.c2_ecturn.json).
Only the listed keys change.   usage: python mkcfg.py   (writes ../on/...; prints each file's sha256[:8] and the keys that differ from its base)"""
import copy, hashlib, json
from pathlib import Path

HERE = Path(__file__).resolve().parent; ON = HERE.parent / 'on'; EXP = HERE.parents[2]
SNAP = EXP / 'orin-web-snapshot-20261122'; ECT = EXP / 'field-20261122' / 'ecturn' / 'on' / 'lap_ff.c2_ecturn.json'

# ---------------- curve 4 (lap_ff.json c4.band) ----------------
# kappa_turn: the car's table up to 40 deg (landing corner, land4 owns x < 1.2 m), then one level 20 deg/m (controller units: the logged mean
# command over x 2.4..6.9 m is +126 us = 19-20 deg/m through the gain table) instead of the bumps 28 @72 / 21 @90-112 / 29 @140 / 20 @157 that were
# the medians of older passes; 14-15 deg/m at 45-55 deg keeps today's command (+70..+90) while the car sheds the inward heading left by the landing
# corner (arc_lat: -8 deg at x 1.2 m); the last four points (exit taper, 170 deg on) are the car's.
K4 = [[0.0, 5.0], [10.0, 30.0], [15.0, 46.0], [30.0, 45.0], [40.0, 30.0], [45.0, 15.0], [50.0, 14.0], [55.0, 17.0], [61.0, 20.0], [157.0, 20.0], [170.0, 19.0], [178.0, 16.0], [186.0, 8.0], [193.0, 0.0]]
# camera near error of a car on the lane centre, tangent to the lane (least squares of 26 passes on turned-angle bins, position and relative heading
# from arc_lat: proof/A3): the reading dips at 40-60, 100-110 and 150-170 deg (the static hand-placed measurement shows the same dips at 106 / 142-168).
F4 = [[45.0, 0.291], [55.0, 0.319], [65.0, 0.470], [75.0, 0.444], [85.0, 0.483], [95.0, 0.450], [105.0, 0.344], [115.0, 0.443], [125.0, 0.438], [135.0, 0.435], [145.0, 0.443], [155.0, 0.335], [165.0, 0.323]]
B4 = 0.0198   # error per cm of lateral position (same fit)
OWN4 = dict(enabled=True, x0_m=1.5, ramp_m=0.6, e_max=0.85, e_band=0.15)


def ec4(y_cm):
    """e_c_turn: below 44 deg the old formula's values (0.44 * kappa / 22 of the car table), 45..165 deg = F4 + B4 * y (y < 0 = inside),
    from 172 deg the old formula's values of the exit taper (0.44 * kappa / 22: 0.34 / 0.32 / 0.16 / 0)."""
    t = [[0.0, 0.90], [30.0, 0.90], [40.0, 0.60]]
    t += [[a, round(f + B4 * y_cm, 3)] for a, f in F4]
    t += [[172.0, 0.34], [178.0, 0.32], [186.0, 0.16], [193.0, 0.0]]
    return t


def c4_cfg(base, step):
    c = copy.deepcopy(base); b = c['c4']['band']
    b['kappa_turn'] = copy.deepcopy(K4)
    if step == 's0':
        b['e_c'] = 0.40
        c['note_c3c4'] = ('c3c4-20261122 c4 step 0 (config only, any lap_ff.py from 0f739ce5 on): c4.band.kappa_turn flattened to 20 deg/m between 61 and 157 deg (+ 14-15 at 45-55 deg), '
                          'c4.band.e_c 0.44 -> 0.40 (error centre 0.36 at kappa 20). Back = the base file.')
        return c
    own = step.startswith('own'); y = {'s1': -5.0, 'own': -5.0, 'own_out': -3.0, 'own_in': -7.0}[step]
    b['e_c_turn'] = ec4(y)
    if own:
        b['own'] = dict(OWN4); b['floor_below_us'] = 40.0
        c['note_c3c4'] = ('c3c4-20261122 c4 OWN (needs lap_ff.py adcde919 = band-own-r1): tables as step 1 (kappa_turn flattened, e_c_turn = centred reading %+.0f cm); c4.band.own = the band output is the band '
                          'target itself from 1.5 m after the landing (ramp 0.6 m; the share falls to 0 between near error 0.70 and 0.85 = the old rule returns when the car is far outside); '
                          'floor_below_us 40 (the target may go 40 us left of the centre). Back = the base file.' % y)
    else:
        c['note_c3c4'] = ('c3c4-20261122 c4 step 1 (needs lap_ff.py 1a1ea129 or adcde919): c4.band.kappa_turn as step 0, c4.band.e_c_turn = camera reading of a centred car per turned angle '
                          '(26 passes, arc_lat position / heading regression) %+.0f cm; e_c stays 0.44 (unused while e_c_turn is present). Still floor + top (the PID chain can add). Back = the base file.' % y)
    return c


# ---------------- curve 3, config only (arc2.json c3.kappa_table + curve_trim.json c3): no program change ----------------
# the lane of curve 3 is a polyline (proof/B2): bend 1 at C3s+1.2..2.2 m, straight 2.3..3.1 m, bend 2 3.2..4.0 m, corner 4.5..5.1 m. The car keeps turning on
# the straight (I term +40, the c3 floor +14 us on 75 % of the frames, lag) and reaches bend 2 pointing 10 deg inside, 18 cm inside the lane centre.
# Paired change: (1) less command on the straight: c3 floor table lowered at 2.4-2.7 m and a trim window 2.2..3.15 m that may go below the centre;
# (2) the floor follows bend 2 with the plant lead (lane 43-45 deg/m at 3.2-3.9 m; the car's table has 16 there).
C3_STEPS = {
    's1': dict(trim=25.0, min_rel=-10.0, kap={2.425: 16.0, 2.675: 6.0, 2.925: 20.0, 3.175: 40.0, 3.425: 42.0, 3.675: 38.0}),
    's2': dict(trim=40.0, min_rel=-20.0, kap={2.425: 10.0, 2.675: 0.0, 2.925: 30.0, 3.175: 52.0, 3.425: 54.0, 3.675: 46.0}),
}


def c3_cfg(arc2, trim, step):
    S = C3_STEPS[step]; a = copy.deepcopy(arc2); t = copy.deepcopy(trim)
    for q in a['c3']['kappa_table']:
        if q[0] in S['kap']: q[1] = S['kap'][q[0]]
    a['c3']['note_c3c4'] = ('c3c4-20261122 c3 config-only %s: kappa_table at 2.425 / 2.675 m lowered (lane straight there), at 2.925..3.675 m raised to follow bend 2 (lane 43-45 deg/m at 3.2-3.9 m); '
                            'nothing after 3.9 m changed (corner 4.2-5.6 m, ramp foot, top). Use together with the curve_trim.json of the same folder. Back = arc2.json d61e3f58.' % step)
    t['curves']['c3'] = dict(enabled=True, anchor='C3s', table=[[2.2, 0.0], [2.4, S['trim']], [3.0, S['trim']], [3.15, 0.0]], e_release=0.9, min_rel_us=S['min_rel'], fade_us=6.0)
    t['note_c3c4'] = ('c3c4-20261122 c3 config-only %s: the c3 block of the car (C3s+1.1..3.9 m, 30 us, floor centre+60) is replaced by a window on the straight piece only (C3s+2.2..3.15 m, %.0f us, '
                      'may go down to centre %+.0f us); c1 block unchanged. Use together with the arc2.json of the same folder. Back = curve_trim.json 1b163ea0 (car) or 865a8d7d (c1 only).' % (step, S['trim'], S['min_rel']))
    return a, t


# ---------------- curve 3, distance band that owns the command (arc2.json c3.xband; needs lap_ff.py adcde919 + live_pid.py 3b74ddc4) ----------------
def c3_own(arc2, trim, pr, name, shadow=False):
    import plan_c3 as PL
    a = copy.deepcopy(arc2); xb = PL.xband(PL.plan(**pr))
    if shadow: xb['shadow'] = True
    xb['note'] = ('c3c4-20261122 %s: distance band x-band-r1 (needs lap_ff.py adcde919 + live_pid.py 3b74ddc4)%s. Owns C3s+0.9..4.05 m (ramps 0.6-0.9 / 4.05-4.35 m): command = kappa_x/g(v) + k_e*(near - e_c_x) + gyro term; '
                  'the corner (4.35 m on), the ramp foot and the c3 top stay with the present control. Tables = tools/plan_c3.py (lane heading rate smoothed over %.1f m, path aimed %+.0f cm from the lane centre). '
                  'The share falls to 0 between near error 0.80 and 0.90. Delete the xband key = the car.' % (name, ' SHADOW: logged only, command unchanged' if shadow else '', pr['smooth'], pr['y_t']))
    a['c3']['xband'] = xb
    t = copy.deepcopy(trim)
    if not shadow: t['curves']['c3']['enabled'] = False   # the trim layer runs after the c3 layer: it would subtract from the band's command
    return a, t


def dump(p, o):
    s = json.dumps(o, ensure_ascii=False, indent=1) + '\n'; p.parent.mkdir(parents=True, exist_ok=True); p.write_bytes(s.encode('utf8')); return hashlib.sha256(s.encode('utf8')).hexdigest()[:8]


def diff(a, b, pre=''):
    out = []
    for k in sorted(set(a) | set(b)):
        if a.get(k) == b.get(k): continue
        if isinstance(a.get(k), dict) and isinstance(b.get(k), dict): out += diff(a[k], b[k], pre + k + '.')
        else: out.append(pre + k)
    return out


def main():
    lf = json.loads((SNAP / 'lap_ff.json').read_text(encoding='utf8')); a2 = json.loads((SNAP / 'arc2.json').read_text(encoding='utf8')); ct = json.loads((SNAP / 'curve_trim.json').read_text(encoding='utf8'))
    C4 = (('s0', 'c4_step0_tables_norestart'), ('s1', 'c4_step1_tables_ecturn'), ('own', 'c4_step2_own'), ('own_out', 'c4_step2_own_dial_out2cm'), ('own_in', 'c4_step2_own_dial_in2cm'))
    for step, name in C4:
        c = c4_cfg(lf, step); print(name + '/lap_ff.json', dump(ON / name / 'lap_ff.json', c), 'differs from the snapshot file f6fe9210 in:', diff(lf, c))
    if ECT.exists():   # the same curve-4 keys on top of the car's file since 11-22 16:47 (curve-2 e_c_turn table)
        e2 = json.loads(ECT.read_text(encoding='utf8'))
        for step, name in C4:
            c = c4_cfg(e2, step); print('on_car_14c5e82c/' + name + '/lap_ff.json', dump(ON / 'on_car_14c5e82c' / name / 'lap_ff.json', c), 'differs from lap_ff.json 14c5e82c in:', diff(e2, c))
    for step in ('s1', 's2'):
        a, t = c3_cfg(a2, ct, step); d = ON / ('c3_cfgonly_%s' % step[1])
        print(d.name + '/arc2.json', dump(d / 'arc2.json', a), 'differs in:', diff(a2, a)); print(d.name + '/curve_trim.json', dump(d / 'curve_trim.json', t), 'differs in:', diff(ct, t))
    t0 = copy.deepcopy(ct); t0['curves']['c3']['enabled'] = False
    print('c3_trim_off/curve_trim.json', dump(ON / 'c3_trim_off' / 'curve_trim.json', t0), 'differs in:', diff(ct, t0))
    for name, pr, sh in (('c3_own_step0_shadow', dict(smooth=0.7, y_t=-10.0), True), ('c3_own_step1_in10', dict(smooth=0.7, y_t=-10.0), False), ('c3_own_step2_in7', dict(smooth=0.7, y_t=-7.0), False)):
        a, t = c3_own(a2, ct, pr, name, sh); d = ON / name
        print(name + '/arc2.json', dump(d / 'arc2.json', a), 'differs in:', diff(a2, a))
        if not sh: print(name + '/curve_trim.json', dump(d / 'curve_trim.json', t), 'differs in:', diff(ct, t))


if __name__ == '__main__':
    main()
