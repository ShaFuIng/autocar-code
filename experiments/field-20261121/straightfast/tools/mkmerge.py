"""straightfast-20261121: build one lap_speed.json from the car file 77ffbe32 + the pieces of the three offline items.
usage: python mkmerge.py <out.json> [c4=58|70] [c3=s1|s2] [st=A1|A2|B1|B2a|B2]
Zone order (lap_speed takes the FIRST zone that contains the position):
  0 main 100, 1 diagonal 100, 2 circle 100, 3 ramp 100  (car order; the straightfast keys are added to 0-2 in place)
  4 c4 back half (LAND8+3.4..9.0)          - c4fast puts it here, before the PWM-50 zones; the main 100 zone (index 0) still wins from C4e+0.8 m
  5-7 the three PWM-50 zones, 8 crest 27   (car zones 4-7)
  9 c3 front (C3s+0.3..)                    - c3front appends it; no other zone covers C3s+0.3..2.0
The pieces do not overlap in position, so each keeps its own frames; only st=A2 changes the others (global rate_pwm_s 120 also ramps
the c4 / c3 / ramp zones twice as fast; B* use per-zone rates and leave them alone)."""
import json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent; EXP = HERE.parents[2]; FIELD = EXP / 'field-20261121'
CAR = EXP / 'orin-web-snapshot-20261121c' / 'lap_speed.json'
C4 = {'58': FIELD / 'c4fast/on/lap_speed.step58.json', '70': FIELD / 'c4fast/on/lap_speed.step70.json'}
C3 = {'s1': FIELD / 'c3front/on/lap_speed.c3front_s1_40.json', 's2': FIELD / 'c3front/on/lap_speed.c3front_s2_48.json'}
ST = {'A1': 'lap_speed.A1_relax.json', 'A2': 'lap_speed.A2_relax_rate120.json', 'B1': 'lap_speed.B1_two_level_main.json', 'B2a': 'lap_speed.B2a_two_level_main_diag.json', 'B2': 'lap_speed.B2_two_level_all.json'}


def load(p): return json.loads(Path(p).read_text(encoding='utf8'))


def build(c4=None, c3=None, st=None):
    car = load(CAR); out = json.loads(json.dumps(car)); notes = []
    if st:
        s = load(HERE.parent / 'on' / ST[st]); assert len(s['zones']) == len(car['zones'])
        out['zones'] = json.loads(json.dumps(s['zones']))
        for k in s:
            if k != 'zones' and not k.startswith('note') and s[k] != car.get(k): out[k] = s[k]
        notes.append('straightfast ' + st)
    if c4:
        a = load(C4[c4]); new = [z for z in a['zones'] if z not in car['zones']]; assert len(new) == 1 and a['zones'].index(new[0]) == 4
        out['zones'].insert(4, new[0]); notes.append('c4fast step' + c4)
    if c3:
        a = load(C3[c3]); new = [z for z in a['zones'] if z not in car['zones']]; assert len(new) == 1 and a['zones'][-1] == new[0]
        out['zones'].append(new[0]); notes.append('c3front ' + c3)
    out['note_merged_20261121'] = 'merged by straightfast/tools/mkmerge.py from the car lap_speed.json 77ffbe32: ' + ' + '.join(notes) + '. Back = 77ffbe32.'
    return out


if __name__ == '__main__':
    kw = dict(a.split('=', 1) for a in sys.argv[2:])
    d = build(**kw); Path(sys.argv[1]).write_text(json.dumps(d, ensure_ascii=False, indent=1), encoding='utf8')
    print(sys.argv[1], 'zones', len(d['zones']), [(z.get('anchor'), z.get('from_m'), z.get('to_m'), z.get('pwm'), 'two_level' if z.get('two_level') else '') for z in d['zones']])
