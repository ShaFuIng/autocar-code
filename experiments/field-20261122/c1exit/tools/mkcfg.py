"""c1exit: build the on/ step files from the car's current json files, changing ONLY the keys listed below (prints every changed key path).
  python mkcfg.py [--lap-ff <car lap_ff.json>] [--i-ramp <car i_ramp.json>] [--out <folder>]
Defaults: ../base/lap_ff.car_052be3dc.json (the car since 11-22 17:50, rebuilt byte-exact), snapshot 20261122 i_ramp.json (b231aa8e), ../on.
If the main session changed the car's lap_ff.json (curve 2 / curve 4 parts) meanwhile: run this again with --lap-ff <that file>; the c1 block is added whole,
the c2 steps touch only c2.band.kappa_turn (last two points) / e_c_turn (last three points) / end_deg / own_lo_us / v_ref and curves.c2.max_m.
Output format = json.dumps(indent=1, ensure_ascii=False) + newline (the format of the car file)."""
import sys, json, copy, hashlib, os
from pathlib import Path
HERE = Path(__file__).resolve().parent
EXP = Path(os.environ.get('C1X_EXP', r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments'))


def arg(k, d):
    return Path(sys.argv[sys.argv.index(k) + 1]) if k in sys.argv else d


LAPFF = arg('--lap-ff', HERE.parent / 'base' / 'lap_ff.car_052be3dc.json'); IRAMP = arg('--i-ramp', EXP / 'orin-web-snapshot-20261122' / 'i_ramp.json'); OUT = arg('--out', HERE.parent / 'on')

# ---------------------------------------------------------------- curve 1 band (profile.py / ident.py, 34 passes of 11-22)
# profile: prof = c_eff + kappa/g(v). On the car single_centre.json makes c_eff = the centre (1475), so the profile is centre + kappa/g (checked in the logs: curve-2 band prof - centre = +263 = 42.4/0.161).
#   needed pre-play command for the lane = 44.9 deg/m / 0.19 - 6 = +231 us from the centre (ident.py; the mean of the logged medians at tb 20..120 is +235)
#   -> kappa = 231 * g(1.10 m/s) = 231 * 0.1568 = 36.2 (gain-table units; NOT the tape curvature).  +4.7 (= 30 us) inside the curve_trim c1 window
#   (C1s + 0.8..2.3 m = tb ~46..113 deg, fades 28..46 / 113..127): curve_trim runs after lap_ff and takes 30 us off; the table gives it back so the net command is the lane's.
#   tail: the no-slam passes unwind +182 -> +114 -> +90 -> +47 -> -11 us over tb 115..155 and end at 152.8 deg (p10 151.4, p90 154.2).
#   beyond the diagonal heading the value goes negative (only used with own_lo_us: straight point = centre - 7 at 153 deg, ~7 us per deg further).
K1 = [[0.0, 36.2], [28.0, 36.2], [46.0, 40.9], [100.0, 40.9], [110.0, 37.0], [115.0, 32.5], [125.0, 19.0], [135.0, 14.0], [145.0, 7.4], [153.0, -1.1], [165.0, -15.0]]
# error target = centred/tangent error (ident.py f(tb), static measurement 11-22: 0.66..0.73 mid-curve incl. the 18 px offset) minus ~0.08 (= 3-4 cm inside, the side the user accepts)
E1 = [[0.0, 0.30], [10.0, 0.45], [20.0, 0.58], [40.0, 0.62], [110.0, 0.62], [125.0, 0.52], [135.0, 0.38], [145.0, 0.18], [152.0, 0.08], [170.0, 0.05]]
C1_BAND = dict(enabled=True, kappa_turn=K1, e_c_turn=E1, ref_win=[3.0, 8.0], ref_min_n=3, turn0_deg=0.0, e_c=0.62, k_e_us=220.0, e_lo_us=250.0, e_hi_us=150.0, k_gz_us=1.5, gz_lim_us=60.0,
               m_hi_us=140.0, cap_us=1950.0, end_deg=147.0, trig_head_min_deg=-4.0, lost_hold=True, lost_max_s=1.0, own_after_deg=0.0, own_us=0.0,
               note="c1exit-r1 (field-20261122/c1exit, offline candidate). Curve 1 full-authority band = the structure of the curve-2 band: target = centre + kappa(tb)/g(v) + clip(220*(near - e_c(tb)), -250, +150) "
                    "+ clip(1.5*(gz + v*kappa), +-60), output = target from the trigger (own_after_deg 0). tb = IMU turn since the main straight (C4e + 3..8 m). kappa_turn in gain-table units (36.2 = +231 us = "
                    "the lane 44.9 deg/m at the identified 0.19 deg/m per us; +4.7 at tb 46..100 gives back the 30 us curve_trim c1 takes off after this layer; with single_centre the profile is centre + kappa/g). Ends at tb 147 + 0.5 m fade. The FAR trigger comes 0.44-0.93 m (median 0.67) before the car has turned 10 deg = ~0.45 m before the arc: delay_m 0.30 + ramp_m 0.20 start the band at the arc. "
                    "Lap from the start: no C4e anchor -> not armed (today's chain) unless curves.c1.start + lap_ff.py ec52cb21. Off = curves.c1.enabled false.")
C1 = dict(enabled=True, anchor='C4e', gap_m=10.74, win_m=[-1.8, 1.2], pos_m=None, kappa_need=40.8, end_deg=147.0, max_m=4.6, cap_us=1800.0, check_m=1.3, pitch_max=None, delay_m=0.30, ramp_m=0.20, band=C1_BAND)


def c1_step1(d):
    d['curves']['c1'] = copy.deepcopy(C1); return ['curves.c1 (whole block; was enabled false, no band)']


def c1_step2(d):   # needs lap_ff.py ec52cb21: hold through the exit (ends by distance), may steer left of the centre, fixed profile speed
    ch = c1_step1(d); b = d['curves']['c1']['band']
    b['own_lo_us'] = -120.0; b['v_ref'] = 1.10; b['end_deg'] = 400.0; d['curves']['c1']['max_m'] = 4.7
    b['note'] += " step2 (needs lap_ff.py ec52cb21): own_lo_us -120 (the target may go left of the centre: needed to unwind an over-rotation), v_ref 1.10 (profile no longer follows the 5 Hz speed noise), " \
                 "end_deg 400 + max_m 4.7 = the band keeps the output through the exit (C1e is 3.4-4.2 m after the trigger, median 3.8 -> about C1e + 0.9 m, then 0.5 m fade) instead of ending at tb 147."
    return ch + ['curves.c1.band.own_lo_us -120', 'curves.c1.band.v_ref 1.10', 'curves.c1.band.end_deg 147 -> 400', 'curves.c1.max_m 4.6 -> 4.7']


def c1_step3(d):   # + lap from the start
    ch = c1_step2(d)
    d['curves']['c1']['start'] = dict(gap_m=3.2, win_m=[-1.2, 1.0], pos_m=None, ref_win=[0.4, 1.8])
    return ch + ['curves.c1.start {gap_m 3.2, win_m [-1.2, 1.0], ref_win [0.4, 1.8]}']


# ---------------------------------------------------------------- curve 2 exit (profile.py C2e, 14 passes e0cd / 6f61 / c6f3)
# the band ends at tb 196-198.6 while the diagonal ends at 207.8 deg (p10 205.7, p90 209.8): the last ~10 deg are driven by the PID at 9-12 deg/m with e +0.20..+0.24,
# then the integral (+40 us, never released after C2e) holds e -0.15..-0.2 for > 2 s.
# car logs (proof/B4): 15 of 18 passes end by curves.c2.max_m 4.8 m at tb 190-195 (not by end_deg 196); with the band alive the lane end (~208 deg) is ~5.25 m after the trigger -> max_m 5.8 (tail) / 6.1 (hold).
def c2_tail(d, shift):
    b = d['curves']['c2']['band']; ch = []
    kt = [list(x) for x in b['kappa_turn']]; ec = [list(x) for x in b['e_c_turn']]
    for k in (-2, -1): kt[k][0] = kt[k][0] + shift
    for k in (-3, -2, -1): ec[k][0] = ec[k][0] + shift
    assert kt[-3][0] < kt[-2][0] and ec[-4][0] < ec[-3][0]
    ch.append('curves.c2.band.kappa_turn last two points %s -> %s' % (b['kappa_turn'][-2:], kt[-2:])); ch.append('curves.c2.band.e_c_turn last three points %s -> %s' % (b['e_c_turn'][-3:], ec[-3:]))
    ch.append('curves.c2.band.end_deg %s -> %s' % (b['end_deg'], b['end_deg'] + shift)); ch.append('curves.c2.max_m %s -> %s' % (d['curves']['c2']['max_m'], 5.8))
    b['kappa_turn'] = kt; b['e_c_turn'] = ec; b['end_deg'] = b['end_deg'] + shift; d['curves']['c2']['max_m'] = 5.8
    b['note_c1exit'] = "c1exit-r1 (field-20261122/c1exit): tail %+g deg (the band ended at tb 190-196, 15 of 18 passes by max_m 4.8 m, while the lane ends at ~208; 11-22). Back = previous kappa_turn / e_c_turn / end_deg / max_m." % shift
    return ch


def c2_hold(d, shift):   # needs lap_ff.py ec52cb21
    ch = c2_tail(d, shift); b = d['curves']['c2']['band']
    kt = b['kappa_turn']; kt[-1] = [208.0, -1.1]; kt.append([220.0, -15.0]); b['own_lo_us'] = -120.0; b['v_ref'] = 1.02
    ec = b['e_c_turn']; ec[-1] = [ec[-1][0], 0.08]; ec.append([230.0, 0.05])
    b['end_deg'] = 400.0; d['curves']['c2']['max_m'] = 6.1
    return ch + ['curves.c2.band.kappa_turn end [208, -1.1], [220, -15]', 'curves.c2.band.e_c_turn end -> 0.08 / [230, 0.05]', 'curves.c2.band.own_lo_us -120', 'curves.c2.band.v_ref 1.02',
                 'curves.c2.band.end_deg -> 400', 'curves.c2.max_m -> 6.1']


def iramp_c2e(d):
    d['exit']['C2e'] = dict(enabled=True, start_m=0.3, end_m=3.0, tau_s=0.2, cap_us=None); return ['exit.C2e {start_m 0.3, end_m 3.0, tau_s 0.2} (same as exit.C1e)']


def dump(d): return (json.dumps(d, ensure_ascii=False, indent=1) + '\n').encode('utf8')


def sha(b): return hashlib.sha256(b).hexdigest()[:8]


def main():
    raw = LAPFF.read_bytes(); base = json.loads(raw); iraw = IRAMP.read_bytes(); ibase = json.loads(iraw)
    print('base lap_ff.json %s (%s)   i_ramp.json %s (%s)' % (sha(raw), LAPFF.name, sha(iraw), IRAMP.name))
    if dump(base) != raw: print('  note: the base lap_ff.json is not in indent=1 format; the outputs are')
    steps = [('c1_step1_band', 'lap_ff.json', base, c1_step1), ('c1_step2_exit_hold', 'lap_ff.json', base, c1_step2), ('c1_step3_lap1', 'lap_ff.json', base, c1_step3),
             ('c2_step1_tail6', 'lap_ff.json', base, lambda d: c2_tail(d, 6.0)), ('c2_step1_tail10', 'lap_ff.json', base, lambda d: c2_tail(d, 10.0)),
             ('c2_step2_exit_hold', 'lap_ff.json', base, lambda d: c2_hold(d, 10.0)),
             ('combo_c1_step1__c2_tail10', 'lap_ff.json', base, lambda d: c1_step1(d) + c2_tail(d, 10.0)),
             ('combo_c1_step3__c2_step2', 'lap_ff.json', base, lambda d: c1_step3(d) + c2_hold(d, 10.0)),
             ('i_ramp_c2e', 'i_ramp.json', ibase, iramp_c2e)]
    for name, fn, b0, f in steps:
        d = copy.deepcopy(b0); ch = f(d); out = dump(d) if fn == 'lap_ff.json' else (json.dumps(d) + '').encode('utf8')
        p = OUT / name; p.mkdir(parents=True, exist_ok=True); (p / fn).write_bytes(out)
        print('\n%s/%s  %s' % (name, fn, sha(out)))
        for c in ch: print('   ' + c)
        if fn == 'lap_ff.json':   # check: nothing else differs
            for k in set(base) | set(d):
                if k != 'curves': assert base.get(k) == d.get(k), k
            for cn in ('c1', 'c2', 'c3'):
                if base['curves'].get(cn) != d['curves'].get(cn): print('   (differs from the base: curves.%s)' % cn)


if __name__ == '__main__':
    main()
