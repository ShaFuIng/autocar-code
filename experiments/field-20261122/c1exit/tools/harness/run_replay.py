"""c1exit replay driver (copy of the c3c4 driver; base = snapshot 20261122 + the car programs / configs of each run)
[c3c4 replay driver (copy of the landing4 driver; base = snapshot 20261122, runs d60e / ccf1) (laptop only). Builds the per-run base config, then runs harness/replay.py once per (run, side).
  python run_replay.py --tag off --old <snapshot live_pid.py> --new <candidate live_pid.py> [--cfg <folder of json overrides for the NEW side>] [--cfg-old <folder>] [--runs e1b4,65ea,...]
Outputs <L4_OUT>/<tag>/<run>.<side>.jsonl. Work dirs under env L4_SCRATCH (default: system temp)."""
import argparse, json, os, shutil, subprocess, sys, tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXP = Path(os.environ.get('L4_EXP', r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments'))
SNAP = EXP / 'orin-web-snapshot-20261122'   # c3c4-20261122: the car as of run ccf1
SCR = Path(os.environ.get('L4_SCRATCH', Path(tempfile.gettempdir()) / 'c1exit'))
C1X = HERE.parent.parent   # field-20261122/c1exit
F22 = EXP / 'field-20261122'
CAR_PY = {'lap_ff.py': C1X / 'base' / 'lap_ff.car_adcde919.py', 'live_pid.py': C1X / 'base' / 'live_pid.car_3b74ddc4.py'}   # the car's programs since 11-22 17:50
RUN_CFG = {   # c1exit: what the car had on each run relative to the snapshot 20261122 (AGENTS_LOG 11-22 17:32 / 17:50 / 18:10). e0cd / 1eab / 4bbe ran lap_ff.py 1a1ea129 +
              # live_pid af2d6b68; here they are replayed with the car's CURRENT programs on both sides (c3c4 proof: identical without the new keys), so old/new differ only by this patch.
    'c6f3': dict(files=dict(CAR_PY, **{'lap_ff.json': C1X / 'base' / 'lap_ff.car_052be3dc.json', 'arc2.json': F22 / 'c3c4' / 'on' / 'c3_own_step1_in10' / 'arc2.json', 'curve_trim.json': F22 / 'c3c4' / 'on' / 'c3_trim_off' / 'curve_trim.json'})),
    '6f61': dict(files=dict(CAR_PY, **{'lap_ff.json': C1X / 'base' / 'lap_ff.car_052be3dc.json', 'arc2.json': F22 / 'c3c4' / 'on' / 'c3_own_step0_shadow' / 'arc2.json', 'curve_trim.json': F22 / 'c3c4' / 'on' / 'c3_trim_off' / 'curve_trim.json'})),
    'e0cd': dict(files=dict(CAR_PY, **{'lap_ff.json': F22 / 'c2shape_c4step1' / 'lap_ff.json'})),
    '1eab': dict(files=dict(CAR_PY, **{'lap_ff.json': F22 / 'c2shape_c4step1' / 'lap_ff.car.json'})),
    '4bbe': dict(files=dict(CAR_PY, **{'lap_ff.json': F22 / 'ecturn' / 'on' / 'lap_ff.c2_ecturn.json'})),
}


def base_for(run):
    d = SCR / 'base' / run
    if d.exists(): return d
    d.mkdir(parents=True)
    for p in SNAP.iterdir():
        if p.suffix in ('.py', '.json'): shutil.copyfile(p, d / p.name)
    for n, src in RUN_CFG[run]['files'].items(): shutil.copyfile(src, d / n)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', required=True); ap.add_argument('--old'); ap.add_argument('--new'); ap.add_argument('--cfg', default=''); ap.add_argument('--cfg-old', default='')
    ap.add_argument('--runs', default='c6f3,6f61,e0cd'); ap.add_argument('--teacher', default='clock,start'); ap.add_argument('--restart', action='store_true')
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
