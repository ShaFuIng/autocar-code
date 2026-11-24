"""on/ config files for landing4 (run after mktab.py; needs tables.json). Base files = orin-web-snapshot-20261121c (arc2.json 863721c4, lap_ff.json)."""
import json, hashlib, sys, os
from pathlib import Path

EXP = Path(os.environ.get('L4_EXP', r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments'))
SNAP = EXP / 'orin-web-snapshot-20261121c'
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / 'on'
T = json.load(open(Path(__file__).with_name('tables.json')))
X_HAND = 1.2; HAND_M = 0.3


def land4(slope, land, track=False):
    d = {
        "revision": "land4-r1", "enabled": True,
        "note": "field-20261122/landing4 (offline candidate, NOT deployed; needs land4.py + live_pid.py with the land4 hooks). File absent / enabled false = off (frame-identical to the car). slope / land each have their own enabled; track_only true with both stages off = shadow logging only (proposal.land4.shadow_us, no command change).",
        "ref_s": 1.2, "ref_min_n": 8, "ref_spread_deg": 8.0, "slope_len_m": 2.20, "x_shift_m": 0.0, "lead_s": 0.25, "kappa_max": 48.0, "off_us": 0.0,
        "spike_on": True, "spike_deg": 3.0,
        "note_tables": "theta = median absolute heading (deg, left +, relative to the mean IMU yaw of the last 1.2 s of the upper straight) of the %d good landings of %d (11-21 e1b4, 11-22 65ea / 3197 / 8350 / 46c9; good = outer-line clearance >= -4 cm over x <= 3.0 m and inner >= -9 cm over x <= 3.5 m, arc_lat), u = metres since the landing detection (rear encoder; negative = slope); slope part replaced by 0 until -1.6 m then a straight ramp to the landing value; smoothed +-0.1 m. kappa = -d(theta)/du (deg/m, right +). e_ref = the same landings' median camera near error." % (len(T['good']), T['n_all']),
        "theta": T['theta'], "kappa": T['kappa'], "e_ref": T['e_ref'],
        "slope": {"enabled": bool(slope), "gain": 0.14, "k_psi": 10.0, "psi_lim": 120.0, "k_gz": 1.5, "gz_lim": 60.0, "k_cam": 15.0, "cam_lim": 5.0, "cam_from_m": 0.4,
                  "lim_right_us": 200.0, "lim_left_us": 150.0, "max_step_us": 40.0, "out_frames": 8},
        "land": {"enabled": bool(land), "gain": 0.186, "k_psi": 10.0, "psi_lim": 120.0, "k_gz": 1.5, "gz_lim": 60.0, "k_cam": 12.0, "cam_lim": 8.0,
                 "lim_right_us": 260.0, "lim_left_us": 120.0, "max_step_us": 60.0, "x_end": X_HAND, "hand_m": HAND_M, "t_max_s": 3.0},
    }
    if track: d["track_only"] = True
    return d


def write(p, obj):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(obj, indent=1, ensure_ascii=False) + '\n', encoding='utf8', newline='\n')
    print('%-44s %s' % (str(p.relative_to(OUT)), hashlib.sha256(p.read_bytes()).hexdigest()[:8]))


write(OUT / 'step0_shadow' / 'land4.json', land4(False, False, track=True))
write(OUT / 'step1_slope' / 'land4.json', land4(True, False))
write(OUT / 'step2_slope_land' / 'land4.json', land4(True, True))
write(OUT / 'step3_early_c4band' / 'land4.json', land4(True, True))
a2 = json.loads((SNAP / 'arc2.json').read_text(encoding='utf8'))
a2['land']['x_end'] = X_HAND; a2['land']['hand_m'] = HAND_M
a2['note_landing4'] = "landing4 step 3: land.x_end 2.3 -> %.1f, hand_m 0.5 -> %.1f (the arc2 landing band hands over at %.1f..%.1f m; the curve-4 band of lap_ff.json starts there). Back = arc2.json 863721c4." % (X_HAND, HAND_M, X_HAND, X_HAND + HAND_M)
write(OUT / 'step3_early_c4band' / 'arc2.json', a2)
lf = json.loads((SNAP / 'lap_ff.json').read_text(encoding='utf8'))
lf['c4']['band']['x0_m'] = X_HAND; lf['c4']['band']['ramp_m'] = HAND_M
lf['c4']['band']['note_landing4'] = "landing4 step 3: x0_m 2.3 -> %.1f (ramp %.1f m): the curve-4 band owns from %.1f m after the landing. Back = lap_ff.json 2e121d02 (x0_m 2.3)." % (X_HAND, HAND_M, X_HAND + HAND_M)
write(OUT / 'step3_early_c4band' / 'lap_ff.json', lf)
# step 3 alone (no land4): the two json files only
write(OUT / 'step3_only_json' / 'arc2.json', a2); write(OUT / 'step3_only_json' / 'lap_ff.json', lf)
