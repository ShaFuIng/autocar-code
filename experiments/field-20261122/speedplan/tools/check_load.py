# -*- coding: utf-8 -*-
"""以車上程式（快照 c 的 lap_speed.py／brake_zones.py／speed_adapt.py／upper_wide.py，唯讀匯入）載入每一步的設定檔，
並用假的 lap_pos 事件檢查「哪個區在哪個位置生效」——確認新區不搶走既有區、煞車區上限如預期。只讀檔，不寫入快照。
用法：python check_load.py <on 資料夾>"""
import sys, json, glob, os
from pathlib import Path
SNAP = Path(__file__).resolve().parents[3] / 'orin-web-snapshot-20261122c'
sys.path.insert(0, str(SNAP))
import lap_speed, brake_zones, speed_adapt, upper_wide   # noqa: E402

MPC = 0.000222
# 名義地標里程（lap_pos.json nominal_m；只用相對距離）
NOM = dict(C1s=0.0, C1e=3.33, C2s=15.27, C2e=19.77, C3s=30.21, RAMP=36.97, TOP=39.86, DASHs=42.82, DASHe=43.68, DOWN=47.05, LAND8=49.37, C4s=49.54, C4e=56.98)
LAP = 67.72


class Lap:
    def __init__(self, D, lap=2):
        self.D = D; self.lap = lap; self.lost = False; self.enc0 = 0; self.mpc = MPC
        ev = []
        base = (lap - 1) * LAP
        for k, v in NOM.items():
            if base + v <= D: ev.append(dict(name=k, lap=lap, d_cross=base + v))
        # 上一圈的地標（C4e 之後、C1s 之前的位置用得到）
        for k, v in NOM.items():
            if base - LAP + v <= D and k != 'C1s': ev.insert(0, dict(name=k, lap=lap - 1, d_cross=base - LAP + v))
        ev.sort(key=lambda e: e['d_cross']); self.events = ev


def pos(name, off, lap=2):
    if name == 'C1s+': return (lap) * LAP + off
    return (lap - 1) * LAP + NOM[name] + off


PROBES = [('C4e', 5.0), ('C4e', 9.6), ('C4e', 10.6), ('C1s+', 0.2), ('C1e', 11.0), ('C1e', 12.4), ('C2e', 3.0), ('C2e', 6.0), ('C2e', 7.0), ('C2e', 9.0), ('C2e', 10.6),
          ('C3s', 1.0), ('C3s', 5.0), ('C3s', 6.6), ('RAMP', 1.0), ('RAMP', 3.0), ('LAND8', 1.0), ('LAND8', 5.0)]


def ls_at(cfg, D):
    g = lap_speed.LapSpeed(cfg, 'x')
    if pos_is_after_c1s(D):   # C1s 已判定：lap 遞增，lap_speed 只認本圈事件
        lp = Lap(D, 3)
    else: lp = Lap(D, 2)
    z, rel = g.zone_at(lp)
    return None if z is None else '%s(%s) rel %.2f pwm %s' % (z['name'][:22], z['anchor'], rel, z['pwm'])


def pos_is_after_c1s(D):
    return D >= 2 * LAP + 0.4


def bz_at(cfg, D):
    b = brake_zones.BrakeZones(cfg)
    lp = Lap(D, 3 if pos_is_after_c1s(D) else 2)
    v0 = {'enc_b': {'forward': 0, 'reverse': 0}}
    b.update(v0, lap=lp)   # zero
    cap, info = b.update({'enc_b': {'forward': int(round(D / MPC)), 'reverse': 0}}, lap=lp, pitch_rel=0.0)
    z = (info or {}).get('zone') or {}
    return None if cap is None else '%s cap %s' % (str(z.get('name'))[:22], cap)


def main():
    root = Path(sys.argv[1])
    dirs = sorted([d for d in root.iterdir() if d.is_dir() and d.name.startswith('step')]) + sorted((root / 'restart').iterdir())
    cur = {f: json.loads((SNAP / f).read_text(encoding='utf8')) for f in ('lap_speed.json', 'brake_zones.json', 'speed_adapt.json', 'upper_wide.json')}
    rows = []
    def snap(tag):
        ls = dict(lap_speed.DEFAULTS); ls.update(cur['lap_speed.json'])
        bz = dict(brake_zones.DEFAULTS); bz.update(cur['brake_zones.json'])
        sa = speed_adapt.SpeedAdapt(dict(speed_adapt.DEFAULTS, **cur['speed_adapt.json']))
        uw = upper_wide.UpperWide(upper_wide.load_config(None) if False else cur['upper_wide.json']) if hasattr(upper_wide, 'UpperWide') else None
        out = ['== %s' % tag, '  speed_adapt min/mid/max %s/%s/%s  upper_wide on %s post_pwm %s turn_pwm %s err %s  lap_speed rate %s' % (
            sa.c['min_pwm'], sa.c['mid_pwm'], sa.c['max_pwm'], getattr(uw, 'on', None), getattr(uw, 'post_pwm', None), getattr(uw, 'turn_pwm', None), getattr(uw, 'error', None), ls.get('rate_pwm_s'))]
        for n, o in PROBES:
            D = pos(n, o)
            out.append('  %-6s%+5.1f | LS %-55s | BZ %s' % (n, o, ls_at(ls, D), bz_at(bz, D)))
        return out
    rows += snap('base（快照 c）')
    for d in dirs:
        for f in cur:
            if (d / f).exists(): cur[f] = json.loads((d / f).read_text(encoding='utf8'))
        rows += snap(str(d.relative_to(root)))
        if d.parent.name == 'restart':   # restart 的檔不累積
            cur['lap_speed.json'] = json.loads((root / 'step14_tail170' / '..' / 'step12_c2br33' / 'lap_speed.json').read_text(encoding='utf8'))
    print('\n'.join(rows))


if __name__ == '__main__':
    main()
