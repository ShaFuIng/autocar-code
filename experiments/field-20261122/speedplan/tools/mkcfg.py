# -*- coding: utf-8 -*-
"""speedplan 設定檔產生器（只改設定、不改程式）。
用法：
  python mkcfg.py --base <放車上現行 json 的資料夾> --steps left34,c1in38,... --out <輸出資料夾> [--check]
  python mkcfg.py --chain <on 資料夾>      依 CHAIN 產生 on/stepNN_<名稱>/（累積：每一步以前一步的結果為底），並逐檔驗證只動列出的鍵。
--base 預設＝experiments/orin-web-snapshot-20261122c。到場時請以「車上實際讀回的檔」當 --base 重產（車上檔與快照不同時，只會套用本工具列出的改動）。
每一步只動下表列出的鍵；每檔另加一條 note_speedplan_<步驟> 說明。輸出 JSON 為 indent=1、ASCII（與車上檔相同的讀法：json.loads）。"""
import argparse, copy, hashlib, json, os, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SNAP = HERE.parents[2] / 'orin-web-snapshot-20261122c'
FILES = ('lap_speed.json', 'brake_zones.json', 'speed_adapt.json', 'upper_wide.json')


def zone(cfg, prefix):
    z = [x for x in cfg['zones'] if str(x.get('name', '')).startswith(prefix)]
    if len(z) != 1: raise SystemExit('zone prefix %r matched %d zones' % (prefix, len(z)))
    return z[0]


def upsert_zone(cfg, prefix, z):
    m = [x for x in cfg['zones'] if str(x.get('name', '')).startswith(prefix)]
    if len(m) > 1: raise SystemExit('zone prefix %r matched %d zones' % (prefix, len(m)))
    if m: m[0].clear(); m[0].update(z)
    else: cfg['zones'].append(z)


# ---- 步驟：每個函式改 C（dict：檔名→內容），回傳 [(檔名, 鍵路徑, 舊, 新)] 供說明；note 另外加。
def s_left(C, zp, cap, tag):
    ls = C['lap_speed.json']; bz = C['brake_zones.json']
    upsert_zone(ls, 'left bend -> curve 3 entry', {
        'name': 'left bend -> curve 3 entry %d (speedplan-20261122 %s: the left bend C2e+6.6..8.0 ran at PWM 24-30 and 0.85-0.94 m/s; raise MID / path-LOW frames, weave_slow LOW still not raised (lap_speed rule); ends before c3 front 48 at C3s+0.3)' % (zp, tag),
        'anchor': 'C2e', 'from_m': 6.6, 'to_m': 10.5, 'pwm': zp, 'allow_low': True, 'err_guard': 0.8, 'steer_guard_us': 350.0})
    z = zone(bz, 'curve 3 entry'); old = z['cap_pwm']; z['cap_pwm'] = cap
    return [('lap_speed.json', 'zones[+left bend -> curve 3 entry]', None, 'C2e+6.6..10.5 pwm %d allow_low err 0.8 steer 350' % zp),
            ('brake_zones.json', 'zones[curve 3 entry].cap_pwm', old, cap)]


def s_c1in(C, zp, cap, shift, tag):
    ls = C['lap_speed.json']; bz = C['brake_zones.json']; ch = []
    upsert_zone(ls, 'curve 1 entry bridge', {
        'name': 'curve 1 entry bridge %d (speedplan-20261122 %s: between the curve-1 brake zone and the curve_map FLOOR (curve key c1) the PWM was speed_adapt LOW 24-30 and the car slowed to 1.02 m/s at C1s, then reached 1.28 m/s inside curve 1 at PWM 44; C4e-anchored = laps 2+ only; stops when C1s is decided (~C1s+0.4))' % (zp, tag),
        'anchor': 'C4e', 'from_m': 9.4, 'to_m': 12.0, 'pwm': zp, 'allow_low': True, 'err_guard': 0.75, 'steer_guard_us': 400.0})
    ch.append(('lap_speed.json', 'zones[+curve 1 entry bridge]', None, 'C4e+9.4..12.0 pwm %d allow_low err 0.75 steer 400' % zp))
    z = zone(bz, 'curve 1 after the slope'); ch.append(('brake_zones.json', 'zones[curve 1 after the slope].cap_pwm', z['cap_pwm'], cap)); z['cap_pwm'] = cap
    if shift:
        ch.append(('brake_zones.json', 'zones[curve 1 after the slope].lap_from_m', z['lap_from_m'], round(z['lap_from_m'] + shift, 2)))
        ch.append(('brake_zones.json', 'zones[curve 1 after the slope].from_m', z['from_m'], round(z['from_m'] + shift, 2)))
        z['lap_from_m'] = round(z['lap_from_m'] + shift, 2); z['from_m'] = round(z['from_m'] + shift, 2)
    return ch


def s_crest(C, p, tag):
    z = zone(C['lap_speed.json'], 'crest (RAMP'); old = z['pwm']; z['pwm'] = p
    return [('lap_speed.json', 'zones[crest (RAMP].pwm', old, p)]


def s_c4(C, p, tag):
    z = zone(C['lap_speed.json'], 'c4 back half'); old = z['pwm']; z['pwm'] = p
    return [('lap_speed.json', 'zones[c4 back half].pwm', old, p)]


def s_post(C, p, tag):
    u = C['upper_wide.json']; old = u['post_pwm']; u['post_pwm'] = p
    return [('upper_wide.json', 'post_pwm', old, p)]


def s_min(C, p, tag):
    s = C['speed_adapt.json']; old = s['min_pwm']; s['min_pwm'] = p
    return [('speed_adapt.json', 'min_pwm', old, p)]


def s_rate(C, p, tag):
    ls = C['lap_speed.json']; old = ls['rate_pwm_s']; ls['rate_pwm_s'] = p
    return [('lap_speed.json', 'rate_pwm_s', old, p)]


def s_c2br(C, p, tag):
    ls = C['lap_speed.json']; bz = C['brake_zones.json']; ch = []
    upsert_zone(ls, 'curve 2 entry bridge', {
        'name': 'curve 2 entry bridge %d (speedplan-20261122 %s: C2s-0.4 ran speed_adapt LOW 24-26 before the curve key c2; lifts non-weave path-LOW / MID frames to the entry cap)' % (p, tag),
        'anchor': 'C1e', 'from_m': 10.6, 'to_m': 12.8, 'pwm': p, 'allow_low': True, 'err_guard': 0.75, 'steer_guard_us': 400.0})
    ch.append(('lap_speed.json', 'zones[+curve 2 entry bridge]', None, 'C1e+10.6..12.8 pwm %d allow_low err 0.75 steer 400' % p))
    for pre in ('curve 2 entry', 'curve 2 on lap 1'):
        z = zone(bz, pre); ch.append(('brake_zones.json', 'zones[%s].cap_pwm' % pre, z['cap_pwm'], p)); z['cap_pwm'] = p
    return ch


def s_bottom(C, p, tag):
    z = zone(C['brake_zones.json'], 'slope bottom'); old = z['cap_pwm']; z['cap_pwm'] = p
    return [('brake_zones.json', 'zones[slope bottom].cap_pwm', old, p)]


def s_tail(C, to, tag):
    z = zone(C['brake_zones.json'], 'curve 3 tail'); old = z['lap_to_m']; z['lap_to_m'] = to
    return [('brake_zones.json', 'zones[curve 3 tail].lap_to_m', old, to)]


TWO = {'e_trouble': 0.4, 'e_ok': 0.3, 'steer_trouble_us': 300.0, 'gz_trouble_dps': 25.0, 'gz_tau_s': 0.25, 'mid_pwm': 70.0, 'rate_up_pwm_s': 200.0, 'rate_down_pwm_s': 120.0}


def s_two(C, which, tag):
    ls = C['lap_speed.json']; ch = []
    for pre in which:
        z = zone(ls, pre); z['two_level'] = dict(TWO); ch.append(('lap_speed.json', 'zones[%s].two_level' % pre, None, 'straightfast TWO (needs lap_speed.py 7bbeb350)'))
    return ch


def s_ramprate(C, p, tag):
    z = zone(C['lap_speed.json'], 'A ramp 100'); z['rate_pwm_s'] = p
    return [('lap_speed.json', 'zones[A ramp 100].rate_pwm_s', None, '%s (needs lap_speed.py 7bbeb350; r1 ignores it)' % p)]


STEPS = {
    'left34':   (lambda C: s_left(C, 34, 34, 'left34'), 'left bend -> curve 3 entry: new lap_speed zone C2e+6.6..10.5 PWM 34 + curve-3 entry brake cap 30 -> 34'),
    'left38':   (lambda C: s_left(C, 38, 38, 'left38'), 'left bend second step: zone 38, curve-3 entry cap 38'),
    'c1in38':   (lambda C: s_c1in(C, 38, 38, 0.0, 'c1in38'), 'curve 1 entry: brake cap 32 -> 38 + bridge zone C4e+9.4..12.0 PWM 38'),
    'c1in42':   (lambda C: s_c1in(C, 42, 42, 0.4, 'c1in42'), 'curve 1 entry second step: cap 42, zone start 0.4 m later, bridge 42'),
    'crest33':  (lambda C: s_crest(C, 33, 'crest33'), 'crest zone RAMP+2.3..4.4 PWM 27 -> 33'),
    'crest36':  (lambda C: s_crest(C, 36, 'crest36'), 'crest zone PWM 36'),
    'c4_64':    (lambda C: s_c4(C, 64, 'c4_64'), 'curve 4 back half zone PWM 58 -> 64'),
    'c4_70':    (lambda C: s_c4(C, 70, 'c4_70'), 'curve 4 back half zone PWM 70'),
    'post46':   (lambda C: s_post(C, 46, 'post46'), 'upper straight after the dashed turn: upper_wide post_pwm 40 -> 46 (slope-mouth cap 34 unchanged)'),
    'min28':    (lambda C: s_min(C, 28, 'min28'), 'speed_adapt min_pwm 24 -> 28 (whole lap: LOW frames, hold frames)'),
    'rate120':  (lambda C: s_rate(C, 120.0, 'rate120'), 'lap_speed global rate_pwm_s 60 -> 120 (whole lap; also the ramp 100 zone)'),
    'c2br33':   (lambda C: s_c2br(C, 33, 'c2br33'), 'curve 2 entry: both curve-2 brake caps 30 -> 33 + bridge zone C1e+10.6..12.8 PWM 33'),
    'bottom40': (lambda C: s_bottom(C, 40, 'bottom40'), 'slope bottom -> curve 4 brake cap 36 -> 40'),
    'tail170':  (lambda C: s_tail(C, 17.0, 'tail170'), 'curve 3 tail brake zone ends C2e+17.4 -> 17.0'),
    'R_B1':     (lambda C: s_two(C, ('A main straight',), 'R_B1'), '(restart: lap_speed.py 7bbeb350) two-level on the main straight 100 zone'),
    'R_B2a':    (lambda C: s_two(C, ('A main straight', 'A diagonal'), 'R_B2a'), '(restart) two-level main + diagonal'),
    'R_B2':     (lambda C: s_two(C, ('A main straight', 'A diagonal', 'A circle'), 'R_B2'), '(restart) two-level on all three straights'),
    'R_ramp200': (lambda C: s_ramprate(C, 200.0, 'R_ramp200'), '(restart, touches the climb) ramp 100 zone own rate 200 PWM/s'),
}

# 建議的累積順序（到場階梯表）；同一個 run 欄相同者可同一輪
CHAIN = [
    ('01', 'left34', 'A'), ('02', 'c1in38', 'A'), ('03', 'crest33', 'A'),
    ('04', 'c4_64', 'B'), ('05', 'post46', 'B'),
    ('06', 'left38', 'C'), ('07', 'c1in42', 'C'), ('08', 'crest36', 'C'), ('09', 'c4_70', 'C'),
    ('10', 'min28', 'D'),
    ('11', 'rate120', 'E'),
    ('12', 'c2br33', 'F'), ('13', 'bottom40', 'F'), ('14', 'tail170', 'F'),
]


def sha(b): return hashlib.sha256(b).hexdigest()


def dump(d): return (json.dumps(d, indent=1, ensure_ascii=True) + '\n').encode('ascii')


def load_base(base):
    return {f: json.loads(Path(base, f).read_text(encoding='utf8')) for f in FILES}


def flat(d, p=''):
    out = {}
    if isinstance(d, dict):
        for k, v in d.items(): out.update(flat(v, p + '/' + str(k)))
    elif isinstance(d, list) and d and all(isinstance(x, dict) and 'name' in x for x in d):
        for x in d: out.update(flat(x, p + '[' + str(x['name'])[:28] + ']'))
    else: out[p] = json.dumps(d, sort_keys=True)
    return out


def apply(C, names):
    log = []
    for n in names:
        f, desc = STEPS[n]
        ch = f(C)
        for fn in sorted({c[0] for c in ch}):
            C[fn]['note_speedplan_' + n] = 'speedplan-20261122 step %s: %s. Generated by field-20261122/speedplan/tools/mkcfg.py.' % (n, desc)
        log.extend([(n,) + c for c in ch])
    return log


def verify(A, B, log, names):
    """B 相對 A：只允許 log 列出的鍵（＋note_speedplan_*、新增區的整段）改變。回傳文字列。"""
    out = []
    for fn in FILES:
        fa, fb = flat(A[fn]), flat(B[fn])
        diff = sorted(k for k in set(fa) | set(fb) if fa.get(k) != fb.get(k))
        if not diff: continue
        allowed = []
        for n, f, path, old, new in log:
            if f != fn: continue
            if path.startswith('zones[+'): allowed.append('/zones[' + path[7:-1][:28])
            elif path.startswith('zones['):
                nm = path[6:path.index(']')]
                key = path[path.index(']') + 2:]
                allowed.append(('/zones[', nm, '/' + key))
            else: allowed.append('/' + path)
        bad = []
        for k in diff:
            ok = k.startswith('/note_speedplan_')
            for a in allowed:
                if isinstance(a, tuple):
                    if k.startswith(a[0]) and a[1][:28] in k and (k.endswith(a[2]) or (a[2] + '/') in k): ok = True
                elif k.startswith(a): ok = True
            if not ok: bad.append(k)
        out.append('  %s：改變 %d 個鍵；不在清單內 %d 個%s' % (fn, len(diff), len(bad), ('：' + ', '.join(bad)) if bad else ''))
        for k in diff: out.append('      %s : %s -> %s' % (k, fa.get(k, '(無)')[:70], fb.get(k, '(無)')[:70]))
        if bad: raise SystemExit('\n'.join(out) + '\nFAIL: keys outside the list')
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default=str(SNAP)); ap.add_argument('--steps', default=''); ap.add_argument('--out'); ap.add_argument('--chain')
    a = ap.parse_args()
    base = load_base(a.base); braw = {f: Path(a.base, f).read_bytes() for f in FILES}
    print('base %s' % a.base)
    for f in FILES: print('  %s %s' % (f, sha(braw[f])[:8]))
    if a.chain:
        root = Path(a.chain); prev = copy.deepcopy(base); prev_bytes = dict(braw); allnames = []
        lines = []
        for num, name, run in CHAIN:
            cur = copy.deepcopy(prev); log = apply(cur, [name]); allnames.append(name)
            v = verify(prev, cur, log, [name])
            d = root / ('step%s_%s' % (num, name)); d.mkdir(parents=True, exist_ok=True)
            changed = sorted({c[1] for c in log})
            rb = {fn: sha(prev_bytes[fn])[:8] for fn in changed}
            readme = ['step%s %s（同輪群組 %s）：%s' % (num, name, run, STEPS[name][1]), '累積：以 step%s 之前的結果為底（%s）' % (num, ' + '.join(allnames)), '']
            for fn in changed:
                b = dump(cur[fn]); (d / fn).write_bytes(b)
                readme.append('%s  新 %s  退回（前一步）%s  base %s' % (fn, sha(b)[:8], sha(prev_bytes[fn])[:8], sha(braw[fn])[:8]))
                prev_bytes[fn] = b
            readme.append(''); readme.append('改動：')
            for n, f, path, old, new in log: readme.append('  %s %s: %s -> %s' % (f, path, old, new))
            readme.append(''); readme.append('驗證（相對前一步，只動列出的鍵）：'); readme.extend(v)
            (d / 'README.txt').write_text('\n'.join(readme) + '\n', encoding='utf8')
            lines.append('step%s_%s [%s] ' % (num, name, run) + '；'.join('%s %s（退 %s）' % (fn, sha(dump(cur[fn]))[:8], rb[fn]) for fn in changed))
            prev = cur
        # 重啟後才有效的鍵：以整串 chain 的最後結果為底，另放 restart/
        for name in ('R_B1', 'R_B2a', 'R_B2', 'R_ramp200'):
            cur = copy.deepcopy(prev); log = apply(cur, [name]); v = verify(prev, cur, log, [name])
            d = root / 'restart' / name; d.mkdir(parents=True, exist_ok=True)
            b = dump(cur['lap_speed.json']); (d / 'lap_speed.json').write_bytes(b)
            (d / 'README.txt').write_text('%s：%s\n底＝step14 累積結果的 lap_speed.json（%s）。只在 lap_speed.py 7bbeb350（straightfast lap-speed-r3）上有效；舊程式 3ad2272b 會忽略新鍵（行為＝底）。\n實際上車時請用 mkcfg.py --base <車上檔> --steps <已採用的步驟>,%s 重產。\n\n%s\n' % (name, STEPS[name][1], sha(dump(prev['lap_speed.json']))[:8], name, '\n'.join(v)), encoding='utf8')
            lines.append('restart/%s lap_speed.json %s' % (name, sha(b)[:8]))
        (root / 'CHAIN.txt').write_text('\n'.join(lines) + '\n', encoding='utf8')
        print('\n'.join(lines))
        return
    names = [x for x in a.steps.split(',') if x]
    cur = copy.deepcopy(base); log = apply(cur, names); v = verify(base, cur, log, names)
    print('\n'.join(v))
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        for fn in sorted({c[1] for c in log}):
            b = dump(cur[fn]); Path(a.out, fn).write_bytes(b); print('wrote %s %s（base %s）' % (fn, sha(b)[:8], sha(braw[fn])[:8]))


if __name__ == '__main__':
    main()
