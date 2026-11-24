"""c3refit replay driver (from the c3c4 driver; adds runs 6f61 / c6f3 with the car's program of that evening)
c3c4 replay driver (copy of the landing4 driver; base = snapshot 20261122, runs d60e / ccf1) (laptop only). Builds the per-run base config, then runs harness/replay.py once per (run, side).
  python run_replay.py --tag off --old <snapshot live_pid.py> --new <candidate live_pid.py> [--cfg <folder of json overrides for the NEW side>] [--cfg-old <folder>] [--runs e1b4,65ea,...]
Outputs <L4_OUT>/<tag>/<run>.<side>.jsonl. Work dirs under env L4_SCRATCH (default: system temp)."""
import argparse, json, os, shutil, subprocess, sys, tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXP = Path(os.environ.get('L4_EXP', r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments'))
SNAP = EXP / 'orin-web-snapshot-20261122'   # c3c4-20261122: the car as of run ccf1
SCR = Path(os.environ.get('L4_SCRATCH', Path(tempfile.gettempdir()) / 'c3refit'))
C1_ONLY = EXP / 'curves-20261120' / 'on' / 'curve_trim.c1.json'   # 865a8d7d = the car's curve_trim.json before the curve-3 block was added (runs up to d60e)
C3C4 = EXP / 'field-20261122' / 'c3c4'; HERE_T = HERE.parent
CAR = {'lap_ff.py': C3C4 / 'candidate' / 'lap_ff.py', 'live_pid.py': C3C4 / 'candidate' / 'live_pid.py', 'lap_ff.json': HERE_T / 'base' / 'lap_ff.car_052be3dc.json',
       'curve_trim.json': C3C4 / 'on' / 'c3_trim_off' / 'curve_trim.json'}   # the car since 17:50: lap_ff.py adcde919, live_pid.py 3b74ddc4, lap_ff.json 052be3dc, curve_trim.json 40078635
RUN_CFG = {   # what the car had on each run relative to the snapshot 20261122 (AGENTS_LOG 11-22)
    'ccf1': dict(files={}),                                   # = the snapshot
    'd60e': dict(files={'curve_trim.json': C1_ONLY}),         # snapshot without the curve-3 trim (old program on the car; the base keeps the snapshot program)
    'd60e_new': dict(run='d60e', files={'curve_trim.json': C1_ONLY, 'lap_ff.py': CAR['lap_ff.py'], 'live_pid.py': CAR['live_pid.py']}),   # d60e's input with the car's present program
    '6f61': dict(files=dict(CAR, **{'arc2.json': C3C4 / 'on' / 'c3_own_step0_shadow' / 'arc2.json'})),    # x-band r1 shadow (f4573960)
    'c6f3': dict(files=dict(CAR, **{'arc2.json': C3C4 / 'on' / 'c3_own_step1_in10' / 'arc2.json'})),      # x-band r1 owns (2b81e9b2)
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
    ap.add_argument('--runs', default='d60e,ccf1'); ap.add_argument('--teacher', default='clock,start'); ap.add_argument('--restart', action='store_true')
    a = ap.parse_args()
    out = Path(os.environ.get('L4_OUT', SCR / 'out')) / a.tag; out.mkdir(parents=True, exist_ok=True)
    for key in a.runs.split(','):
        run = RUN_CFG[key].get('run', key); procs = []
        for side, lp, cfg in (('old', a.old, a.cfg_old), ('new', a.new, a.cfg)):
            if not lp: continue
            if lp == 'base': lp = str(base_for(key) / 'live_pid.py')   # the snapshot live_pid inside the per-run base (its sibling files = that base)
            env = dict(os.environ, L4_BASE=str(base_for(key)), DH_WORK=str(SCR / 'work'), L4_IMU='rec', L4_SYNTH_VEH='1', PYTHONIOENCODING='utf8')
            if cfg: env['L4_CFG_DIR'] = str(Path(cfg).resolve())
            else: env.pop('L4_CFG_DIR', None)
            cmd = [sys.executable, '-B', str(HERE / 'replay.py'), '--live-pid', str(Path(lp).resolve()), '--run', run, '--out', str(out / ('%s.%s.jsonl' % (key, side))), '--tag', '%s_%s_%s' % (a.tag, key, side)]
            if a.teacher: cmd += ['--teacher', a.teacher]
            if a.restart: cmd += ['--restart']
            procs.append((side, subprocess.Popen(cmd, env=env)))
        for side, p in procs:
            rc = p.wait(); print(key, side, 'exit', rc, flush=True)


if __name__ == '__main__':
    main()
