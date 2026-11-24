"""straightfast-20261121 proof A: candidate/lap_speed.py (lap-speed-r3) == the car's lap_speed.py (3ad2272b) whenever no zone has the new keys.
Module level, frame by frame: both modules are driven with the same recorded frames (tools/emul.py) and the same config; the per-frame
lap_speed log dict and the PWM must be identical. Why module level is enough: speed_policy.lap_profile calls LapSpeed.apply and stores
its return value; the module has no other entry point that touches a frame (get() only builds the object), and the patch changes
nothing outside lap_speed.py.
usage: python proof_a.py > ../proof/A_key_absent.txt"""
import sys, json, importlib.util
from pathlib import Path
from common import *
import emul, evalc

FIELD = EXP / 'field-20261121'
CFGS = (('car lap_speed.json 77ffbe32', SNAP / 'lap_speed.json'),
        ('c4fast step70 637c8f9c', FIELD / 'c4fast' / 'on' / 'lap_speed.step70.json'),
        ('c3front s2_48 2d340b41', FIELD / 'c3front' / 'on' / 'lap_speed.c3front_s2_48.json'),
        ('straightfast A2 (no new keys)', HERE.parent / 'on' / 'lap_speed.A2_relax_rate120.json'))


def main():
    cm = evalc.cand_module(); tot = 0; same = 0; bad = 0
    for name, path in CFGS:
        cfg = json.loads(Path(path).read_text(encoding='utf8'))
        for run in RUNS:
            A = emul.run(cfg, str(EXP / run)); B = emul.run(cfg, str(EXP / run), module=cm); assert len(A) == len(B)
            n = sum(1 for a, b in zip(A, B) if a['new_ls'] == b['new_ls'] and a['new_pwm'] == b['new_pwm'])
            logs = sum(1 for a in A if a['new_ls'] is not None)
            tot += len(A); same += n; bad += len(A) - n
            print('%-32s %s frames %5d | frames with a lap_speed log %5d | log + PWM identical %5d | different %d' % (name, run[4:8], len(A), logs, n, len(A) - n))
    print('TOTAL frames %d identical %d different %d all_identical %s' % (tot, same, bad, bad == 0))


if __name__ == '__main__':
    main()
