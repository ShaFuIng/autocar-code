"""c4fast-20261121: drive the car's REAL lap_speed.py (LapSpeed.apply, snapshot 20261121c, 3ad2272b) with the recorded frames of a run.

Not a live_pid replay: per recorded sample frame the inputs of LapSpeed.apply are rebuilt from the log
  p            = the recorded proposal with pwm set back to speed_adapt's own PWM (proposal.speed_adapt.pwm = lap_speed's pwm_before)
  lap_pos      = stub with D = lap_pos.odo_m, lap, lost and an events list (name, d_cross = odo_m - since_anchor_m, lap) built
                 whenever the recorded anchor changes; taken from the PREVIOUS frame's record (the car applies lap_speed before the
                 frame's lap_pos update; checked on the recorded rel_m)
  F.dr / F.pr  = vision.dash_route / its pitch_rel_deg ; F.special = False
  lp.speed     = stub (c.hard_cap from speed_adapt.json, cur = speed_adapt.pwm, t = proposal.t)
After lap_speed the recorded later caps are re-applied in the simplest way: a recorded brake-zone cap caps the new PWM; frames whose
recorded final PWM is below the recorded lap_speed PWM by another cap keep that recorded final value.
NOT modelled: speed_adapt continuing from its raised internal PWM (<= hard_cap 50) on the following frames, and any closed-loop
reaction of the car. Open loop: steering is not an output of this layer (gain_schedule k is at k_min for every PWM >= 34.4).

usage: python emul.py <lap_speed.json> <run dir> [...]   -> list of per-frame dicts via run(); as a script prints the fidelity
of the emulation against the recorded lap_speed log for that config (meaningful only with the config the run was driven with)."""
import copy, importlib.util, json, sys, types
from pathlib import Path

EXP = Path(__file__).resolve().parents[3]
SNAP = EXP / 'orin-web-snapshot-20261121c'


def _mod():
    sys.path.insert(0, str(SNAP))   # lap_speed imports throttle_cap (reads throttle_cap.json next to it: max_pct 100)
    sp = importlib.util.spec_from_file_location('lap_speed_car', str(SNAP / 'lap_speed.py')); m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m)
    return m


LS = _mod()
HARD_CAP = float(json.loads((SNAP / 'speed_adapt.json').read_text(encoding='utf8'))['hard_cap'])


class Lap(object):
    def __init__(self): self.D = 0.; self.lap = None; self.lost = True; self.events = []


def frames(run):
    out = []
    for l in open(str(Path(run) / 'pid.jsonl'), encoding='utf8'):
        if '"kind": "sample"' not in l[:40]: continue
        x = json.loads(l); p = x.get('proposal') or {}
        if p.get('t') is None: continue
        out.append((p, (x.get('vision') or {}).get('dash_route') or {}))
    return out


def run(cfg, rundir, module=None):
    """returns per frame: dict(t, lap, anchor, since, seg, rec_pwm, sa_pwm, rec_ls, new_ls (log or None), new_pwm, e, steer, gz, enc)"""
    M = module or LS
    c = dict(M.DEFAULTS); c.update(cfg); g = M.LapSpeed(c, 'emul')
    lap = Lap(); last_anchor = None; res = []; pend = None
    lp = types.SimpleNamespace(run=dict(parameters=dict(center_us=1475.)), _lpos=lap, speed=types.SimpleNamespace(c=dict(hard_cap=HARD_CAP), cur=None, t=None))
    for p, dr in frames(rundir):
        lpz = p.get('lap_pos') or {}; sa = p.get('speed_adapt') or {}
        if 'center_us' in (p.get('single_centre') or {}): lp.run['parameters']['center_us'] = float(p['single_centre']['center_us'])
        prev = pend   # the car applies lap_speed BEFORE this frame's lap_pos update: it sees the lap_pos state of the previous frame
        pend = lpz
        if prev:
            lap.D = float(prev.get('odo_m') or 0.); lap.lap = prev.get('lap'); lap.lost = bool(prev.get('lost'))
            a = prev.get('anchor'); s = prev.get('since_anchor_m')
            if a is not None and s is not None and (a, lap.lap) != last_anchor:
                lap.events.append(dict(name=a, d_cross=lap.D - float(s), lap=lap.lap)); last_anchor = (a, lap.lap)
        q = copy.deepcopy(p); sa_pwm = sa.get('pwm')
        rec_ls = p.get('lap_speed'); new_ls = None; new_pwm = p.get('pwm')
        if g.on and prev and isinstance(sa_pwm, (int, float)):
            q['pwm'] = float(sa_pwm); lp.speed.cur = float(sa_pwm); lp.speed.t = p['t']
            F = types.SimpleNamespace(dr=dr, pr=dr.get('pitch_rel_deg', (p.get('imu_boost') or {}).get('pitch_rel_deg')), special=False)
            new_ls = g.apply(lp, F, q, p['t'])
            v = float(q['pwm'])
            bz = (p.get('brake_zones') or {}).get('cap_pwm')
            if isinstance(bz, (int, float)): v = min(v, float(bz))
            rl = (rec_ls or {}).get('pwm') if isinstance(rec_ls, dict) else None
            base_after = rl if isinstance(rl, (int, float)) else float(sa_pwm)   # the value the recorded later caps started from
            if isinstance(p.get('pwm'), (int, float)) and p['pwm'] < base_after - 0.02: v = min(v, float(p['pwm']))   # 0.02: the logged PWMs are rounded to 2 decimals   # another recorded cap (upper / guard / turn) was active
            new_pwm = v
        res.append(dict(t=p['t'], lap=lpz.get('lap'), anchor=lpz.get('anchor'), since=lpz.get('since_anchor_m'), seg=lpz.get('seg'), odo=lpz.get('odo_m'),
                        rec_pwm=p.get('pwm'), sa_pwm=sa_pwm, sa_reason=sa.get('reason'), sa_level=sa.get('level'), rec_ls=rec_ls, new_ls=new_ls, new_pwm=new_pwm,
                        e=p.get('image_error'), steer=p.get('steer_us'), gz=(p.get('inputs') or {}).get('gz'), enc=(p.get('inputs') or {}).get('enc_b'),
                        cm=(p.get('curve_map') or {}).get('reason'), bo=bool(p.get('bottom_owner')), landing=bool(sa.get('landing'))))
    return res


if __name__ == '__main__':
    cfg = json.loads(Path(sys.argv[1]).read_text(encoding='utf8'))
    for rd in sys.argv[2:]:
        R = run(cfg, rd); n = same = both = zone_same = 0; worst = 0.
        for r in R:
            a = r['rec_ls']; b = r['new_ls']
            if a is None and b is None: continue
            n += 1
            if isinstance(a, dict) and isinstance(b, dict):
                both += 1; zone_same += a.get('zone') == b.get('zone')
                d = abs(float(a.get('pwm', 0)) - float(b.get('pwm', 0))); worst = max(worst, d); same += d <= 0.011 and a.get('blocked') == b.get('blocked')
        fin = [abs(r['rec_pwm'] - r['new_pwm']) for r in R if isinstance(r['rec_pwm'], (int, float)) and isinstance(r['new_pwm'], (int, float))]
        print('%s: frames %d | frames with a lap_speed log (recorded or emulated) %d, both %d, same zone %d, same pwm+blocked %d (%.1f%%), worst pwm diff %.2f | final pwm identical %d/%d (%.1f%%), max diff %.2f' % (
            Path(rd).name, len(R), n, both, zone_same, same, 100. * same / max(1, both), worst, sum(d <= 0.011 for d in fin), len(fin), 100. * sum(d <= 0.011 for d in fin) / max(1, len(fin)), max(fin) if fin else 0.))
