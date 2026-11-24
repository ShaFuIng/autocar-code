"""直線衝刺設定產生器（field-20261124/sprint，筆電用，不連車）。
用法：
  python mksprint.py --base <車上檔資料夾> --back 7.0 [--finish 8.0] [--brake-before 3.44] [--up-rate 60] --out <資料夾>
      base ＝車上現況的五個檔（lap_pos／brake_zones／lap_ff／start_bias／speed_adapt.json，任一起點皆可，起點由 lap_pos start_s 判斷）。
      輸出 <out>/normal/（該起點的正常設定＝手機「起點位置」套用 back 的結果）與 <out>/sprint/（再開直線衝刺）＋ MANIFEST.txt。
  python mksprint.py --off --base <衝刺檔資料夾> --out <資料夾>
      把衝刺設定拿掉（＝手機「關閉直線衝刺」），輸出五個檔。
規則與手機服務完全相同：起點規則用 mobile_service_candidate/start_cfg.py，衝刺用 sprint_cfg.py（同一份程式）。"""
import argparse, hashlib, json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'mobile_service_candidate'))
import start_cfg as SC, sprint_cfg as SP   # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--base', required=True); ap.add_argument('--out', required=True); ap.add_argument('--back', type=float, default=7.0)
ap.add_argument('--finish', type=float, default=None); ap.add_argument('--brake-before', type=float, default=None); ap.add_argument('--up-rate', type=float, default=None)
ap.add_argument('--off', action='store_true')
a = ap.parse_args()
files = SP.read_all(a.base); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
h = lambda b: hashlib.sha256(b).hexdigest()


def write(d, objs, src):
    d.mkdir(parents=True, exist_ok=True); lines = []
    for n in SP.ALL:
        b = SC.dumps(objs[n], src[n]['style'] or SC.DEFAULT_STYLE).encode('utf8') if n in objs else src[n]['raw']
        assert json.loads(b.decode('utf8')) == (objs[n] if n in objs else src[n]['obj'])
        (d / n).write_bytes(b); lines.append('%s  %s' % (h(b), n))
    return lines


if a.off:
    sa, bz = SP.build_off(files['speed_adapt.json']['obj'], files['brake_zones.json']['obj'])
    lines = write(out, {'speed_adapt.json': sa, 'brake_zones.json': bz}, files)
    (out / 'MANIFEST.txt').write_text('sprint OFF from %s\n%s\n' % (a.base, '\n'.join(lines)), encoding='utf8'); print('\n'.join(lines)); sys.exit(0)

over = {k: v for k, v in (('brake_before_c1s_m', a.brake_before), ('up_rate', a.up_rate)) if v is not None}
p = SP.params(over)
for k, v in over.items():
    if p[k] != v: sys.exit('%s=%s 超出範圍 %s' % (k, v, SP.LIMITS[k]))
b0 = SC.back_of_start_s(files['lap_pos.json']['obj'].get('start_s'))
if b0 is None: sys.exit('base 的 lap_pos start_s 不是起點規則產生的值')
if SP.judge(files, None, p)['on']: sys.exit('base 已經是衝刺設定；先用 --off')
back, code = SC.parse_back(a.back)
if code: sys.exit(code)
ref = SC.ref_from_files(files, b0)                       # flag-spot values of the shifted keys, from the base files themselves
norm = SC.build(files, ref, back)                        # = phone "start position" set back
L1 = write(out / 'normal', {n: norm[n] for n in SC.FILES}, files)
if back < p['min_back_m']: sys.exit('back %.1f < %.1f：直線衝刺要起步左舵關（旗子往後 2.6 m 以上）' % (back, p['min_back_m']))
finish, code = SP.parse_finish(a.finish, SC.d_of(back), p)
if code: sys.exit('%s（finish 上限 %.1f m）' % (code, SC.d_of(back) - p['coast_m']))
sa, bz = SP.build_on(files['speed_adapt.json']['obj'], norm['brake_zones.json'], back, p, finish)
L2 = write(out / 'sprint', {**{n: norm[n] for n in SC.FILES}, 'speed_adapt.json': sa, 'brake_zones.json': bz}, files)
st = SP.judge({n: dict(obj=(sa if n == 'speed_adapt.json' else bz if n == 'brake_zones.json' else norm[n])) for n in SP.ALL}, ref, p)
assert st['on'] and st['consistent'] and st['back_m'] == back, st
zones, info = SP.plan(back, p, finish)
txt = ('base %s (back %.1f)\nback %.1f m -> D %.2f m; full throttle until start + %.2f m (%.2f m before C1s); after-cap %g; up_rate %g; finish %s\n\nnormal/\n%s\n\nsprint/\n%s\n'
       % (a.base, b0, back, info['D'], info['brake_from_m'], p['brake_before_c1s_m'], p['after_cap_pwm'], p['up_rate'], finish, '\n'.join(L1), '\n'.join(L2)))
(out / 'MANIFEST.txt').write_text(txt, encoding='utf8'); print(txt)
