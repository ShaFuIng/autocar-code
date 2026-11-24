"""c1lap1b: feed the recorded per-frame inputs through the car's lap_ff.py (snapshot-20261122c, 80976389) twice -- base lap_ff.json vs a
candidate lap_ff.json -- and list where the outputs differ. Inputs rebuilt from the log (same for both configs, so differences come
only from the config): lp = previous frame's lap_pos (lap, anchor, anchor_d = odo - since_anchor, since_anchor_m, lost, odo_m) +
yaw_imu / gz_imu (integrated proposal.inputs.gz / inputs.gz; the 50 Hz IMU values are not logged), far = turn_preview far error,
near = image_error, v = yaw_inner speed, pitch = imu_boost pitch, yaw = the same integrated heading, steer = lap_ff pid_steer_us
(else proposal steer), centre 1475 (single_centre rewrites c_eff_us to the centre), lo/hi 780/2050, eligible = logged lap_ff.eligible
(else True). This is NOT a reproduction of the car's commands (the IMU heading is approximated); it shows which frames a config change
can touch. Usage: python replay_cfg.py CAND_JSON RUN [RUN ...]"""
import sys, json, copy, importlib.util
from pathlib import Path
from lib import load, num
EXP = Path(__file__).resolve().parents[3]
SNAP = EXP / 'orin-web-snapshot-20261122c'
sys.path.insert(0, str(SNAP))


def mod():
    spec = importlib.util.spec_from_file_location('lap_ff_snap', SNAP / 'lap_ff.py'); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m


def prep_cfg(m, raw):
    c = dict(m.DEFAULTS); c.update(raw); c['c_eff_us'] = 1475.
    return c


def run_one(m, cfg, F):
    L = m.LapFF(copy.deepcopy(cfg)); out = []
    for q in range(len(F)):
        f = F[q]; pv = F[q - 1] if q > 0 else None
        lp = None
        if pv is not None and pv['lap'] is not None:
            sa = pv['sa']; an = pv['anchor']
            lp = dict(lap=pv['lap'], anchor=an, anchor_d=None if (an is None or not num(sa) or not num(pv['odo'])) else round(pv['odo'] - sa, 3),
                      since_anchor_m=sa if an is not None else None, lost=bool(pv['raw'].get('lap_pos', {}).get('lost')), odo_m=pv['odo'])
            if getattr(L, 'band_on', False): lp['yaw_imu'] = f['hd']; lp['gz_imu'] = f['gz'] if num(f['gz']) else 0.
        st = f['lf_pid'] if num(f['lf_pid']) else (f['pre'] if num(f['pre']) else 1475.)
        el = f['lf_elig'] if isinstance(f['lf_elig'], bool) else True
        try:
            o, inf = L.step(lp, f['far'], f['near'], True, f['v'], None, f['hd'], st, 1475., 780., 2050., el)
        except Exception as e:
            o, inf = None, dict(err=repr(e))
        out.append((o, json.dumps(inf, sort_keys=True, default=str) if inf is not None else None, f))
    return out


if __name__ == '__main__':
    m = mod()
    base = prep_cfg(m, json.load(open(SNAP / 'lap_ff.json', encoding='utf8')))
    cand = prep_cfg(m, json.load(open(sys.argv[1], encoding='utf8')))
    tot = 0; diff = 0; where = {}
    for run in sys.argv[2:]:
        F = load(run)
        A = run_one(m, base, F); B = run_one(m, cand, F)
        q4 = next((q for q, g in enumerate(F) if g['anchor'] == 'C4e'), len(F))
        for qq, ((oa, ia, f), (ob, ib, _)) in enumerate(zip(A, B)):
            tot += 1
            if oa != ob or ia != ib:
                diff += 1
                first_lap = qq < q4
                tb = f['tb']
                key = ('lap1' if first_lap else 'later') + (' tb>=100' if num(tb) and tb >= 100 else (' tb<100' if num(tb) else ' no-band-frame'))
                where[key] = where.get(key, 0) + 1
                if oa is not None and ob is not None: where.setdefault(key + ' max|dsteer|', 0); where[key + ' max|dsteer|'] = max(where[key + ' max|dsteer|'], round(abs(oa - ob), 1))
        print(run, 'frames', len(F), flush=True)
    print('TOTAL frames', tot, '| differing', diff)
    for k in sorted(where): print('  ', k, where[k])
