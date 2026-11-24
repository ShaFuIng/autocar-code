"""landing4 replay driver (laptop only). Builds the per-run base config, then runs harness/replay.py once per (run, side).
  python run_replay.py --tag off --old <snapshot live_pid.py> --new <candidate live_pid.py> [--cfg <folder of json overrides for the NEW side>] [--cfg-old <folder>] [--runs e1b4,65ea,...]
Outputs <L4_OUT>/<tag>/<run>.<side>.jsonl. Work dirs under env L4_SCRATCH (default: system temp)."""
import argparse, json, os, shutil, subprocess, sys, tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXP = Path(os.environ.get('L4_EXP', r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments'))
SNAP = EXP / 'orin-web-snapshot-20261121c'
SCR = Path(os.environ.get('L4_SCRATCH', Path(tempfile.gettempdir()) / 'l4'))
RUN_CFG = {   # what the car had on each run relative to the snapshot (AGENTS_LOG 11-22)
    'e1b4': dict(flat=2.05, lap_speed=None),
    '65ea': dict(flat=2.05, lap_speed=None),
    '3197': dict(flat=1.1, lap_speed=EXP / 'field-20261121' / 'c4fast' / 'on' / 'lap_speed.step58.json'),
    '8350': dict(flat=1.1, lap_speed=EXP / 'field-20261122' / 'lap_speed.c4_58_c3s1.json'),
    '46c9': dict(flat=1.1, lap_speed=EXP / 'field-20261122' / 'lap_speed.c4_58_c3s2.json'),
}


def base_for(run):
    d = SCR / 'base' / run
    if d.exists(): return d
    d.mkdir(parents=True)
    for p in SNAP.iterdir():
        if p.suffix in ('.py', '.json'): shutil.copyfile(p, d / p.name)
    rc = RUN_CFG[run]
    ib = json.loads((d / 'imu_boost.json').read_text(encoding='utf8')); ib['flat_pitch_deg'] = rc['flat']
    (d / 'imu_boost.json').write_text(json.dumps(ib), encoding='utf8')
    if rc['lap_speed'] is not None: shutil.copyfile(rc['lap_speed'], d / 'lap_speed.json')
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', required=True); ap.add_argument('--old'); ap.add_argument('--new'); ap.add_argument('--cfg', default=''); ap.add_argument('--cfg-old', default='')
    ap.add_argument('--runs', default='e1b4,65ea,3197,8350,46c9'); ap.add_argument('--teacher', default='clock,start'); ap.add_argument('--restart', action='store_true')
    a = ap.parse_args()
    out = Path(os.environ.get('L4_OUT', SCR / 'out')) / a.tag; out.mkdir(parents=True, exist_ok=True)
    for run in a.runs.split(','):
        procs = []
        for side, lp, cfg in (('old', a.old, a.cfg_old), ('new', a.new, a.cfg)):
            if not lp: continue
            if lp == 'base': lp = str(base_for(run) / 'live_pid.py')   # the snapshot live_pid inside the per-run base (its sibling files = that base)
            env = dict(os.environ, L4_BASE=str(base_for(run)), DH_WORK=str(SCR / 'work'), L4_IMU='rec', L4_SYNTH_VEH='1', PYTHONIOENCODING='utf8')
            if cfg: env['L4_CFG_DIR'] = str(Path(cfg).resolve())
            else: env.pop('L4_CFG_DIR', None)
            cmd = [sys.executable, '-B', str(HERE / 'replay.py'), '--live-pid', str(Path(lp).resolve()), '--run', run, '--out', str(out / ('%s.%s.jsonl' % (run, side))), '--tag', '%s_%s_%s' % (a.tag, run, side)]
            if a.teacher: cmd += ['--teacher', a.teacher]
            if a.restart: cmd += ['--restart']
            procs.append((side, subprocess.Popen(cmd, env=env)))
        for side, p in procs:
            rc = p.wait(); print(run, side, 'exit', rc, flush=True)


if __name__ == '__main__':
    main()
