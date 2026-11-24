"""sprint-cfg-r1 (field-20261124/sprint, offline candidate, NOT deployed): "straight-line sprint" mode for the phone service.
Standard library only; runs on the Orin inside the phone service (or on the laptop through tools/mksprint.py). No car control module is imported.

Goal (user 2026-11-24): the car stands on the main straight about 7 m behind the flag spot and runs down the straight as fast as possible;
lap time does not matter. Switched from the phone page; the car control (orin-dashboard) is NOT restarted: only web/*.json files that the car
control reads at every run start are changed.

Why these two files (read per run, see the car code of snapshot 20261123d):
  speed_adapt.json  live_pid.start() builds SpeedAdapt() per run (live_pid.py line 575) -> speed_adapt.load_config() reads the file.
  brake_zones.json  live_pid.start() builds brake_zones.BrakeZones() per run (live_pid.py line 575) -> brake_zones.load_config().
Why not lap_speed.json: a lap_speed zone only acts after its anchor landmark was crossed in the current lap (lap_speed.LapSpeed.crossing,
  lap_speed.py lines 74-79); before curve 1 of lap 0 the lap_pos event list is empty, so no zone can act on the start straight. Above 50 only
  lap_speed zones or speed_adapt (hard_cap) can command PWM (throttle_cap.py docstring); so the start straight needs speed_adapt hard_cap 100.

What sprint ON changes (and nothing else):
  speed_adapt.json  hard_cap 50 -> 100, max_pwm 50 -> 100 (the HIGH level = full throttle while the path is steady), up_rate 30 -> P['up_rate']
                    (default 60 PWM/s, the rate of lap_speed.json rate_pwm_s and speed_adapt uphill_up_rate already used on the car);
                    + key "sprint_mode" = {orig: the three original values, back_m, D, brake_from_m, finish_m, after_cap_pwm, up_rate, ...}.
                    speed_adapt.load_config() does c.update(file) (speed_adapt.py line 71), so the extra key is carried along unused and is
                    written into the run header of pid.jsonl (live_pid.py line 597: speed_adapt=dict(self.speed.c,...)) = the log says sprint.
  brake_zones.json  appended start-anchored zones (key "sprint": true; brake_zones.py ignores unknown keys):
                      after-cap  from D - brake_before_c1s_m to 100000 m: cap after_cap_pwm (50)  -> full throttle ends there, and every
                                 later flat frame of the run is <= 50 again (otherwise speed_adapt HIGH 100 would also act in the curves)
                      finish     (optional) from finish_m to 100000 m: cap 0 -> the car coasts to a stop and stays (soft stop, see below)
                    start-anchored = odometer metres since the run start (brake_zones.py lines 46-53, 99-101); measured even on lap 0.
OFF removes exactly these: speed_adapt keys back to "orig", "sprint_mode" deleted, the "sprint" zones deleted. Each file keeps its own JSON layout
(start_cfg.detect / dumps), so ON -> OFF ends byte-identical.

Write order (a cut write never leaves "speed_adapt 100 without the after-cap"): ON writes brake_zones.json first, then speed_adapt.json;
OFF writes speed_adapt.json first, then brake_zones.json. Any failure puts the original bytes back (SPRINT_WRITE_FAILED).

Soft stop (finish_m): a brake-zone cap 0 applies to valid PID frames only (speed_policy.caps: not on WARMUP frames, and lost-path hold frames use
speed_adapt.safe_pwm = 24); the run stays active with steering on until the user presses stop. It is not a run end."""
import copy, hashlib, json, os, time
from pathlib import Path
import start_cfg as SC

REVISION = 'sprint-cfg-r1'
FILES = ('speed_adapt.json', 'brake_zones.json')          # written by sprint mode (brake_zones first on ON)
ALL = SC.FILES + ('speed_adapt.json',)                     # read for the state (start position + sprint)
SA_KEY = 'sprint_mode'
ZONE_KEY = 'sprint'
FAR_M = 100000.0
SA_KEYS = ('hard_cap', 'max_pwm', 'up_rate')
DEFAULTS = dict(brake_before_c1s_m=3.44, up_rate=60.0, max_pwm=100.0, after_cap_pwm=50.0, coast_m=1.3, min_back_m=2.6, finish_min_m=1.0)
LIMITS = dict(brake_before_c1s_m=(2.0, 6.0), up_rate=(30.0, 120.0), max_pwm=(50.0, 100.0), after_cap_pwm=(30.0, 50.0), coast_m=(0.8, 3.0),
              min_back_m=(2.6, 7.0), finish_min_m=(0.5, 5.0))


class Refused(SC.Refused):
    pass


def params(over=None):
    """DEFAULTS updated by service.json "sprint" (dict); values outside LIMITS -> the default (never a half-valid set)."""
    p = dict(DEFAULTS)
    if isinstance(over, dict):
        for k, (lo, hi) in LIMITS.items():
            v = over.get(k)
            if SC.num(v) and lo <= float(v) <= hi: p[k] = float(v)
    return p


def parse_finish(v, D, p):
    """(finish_m or None, None) or (None, code). None / '' = no automatic stop. A number with at most one decimal,
    finish_min_m .. D - coast_m (the car must have stopped before curve 1: coast from 1.44 m/s ~1.0-1.3 m, see SUMMARY)."""
    if v is None or v == '': return None, None
    if isinstance(v, bool) or not isinstance(v, (int, float)): return None, 'SPRINT_BAD_INPUT'
    v = float(v)
    if v != v or v in (float('inf'), float('-inf')): return None, 'SPRINT_BAD_INPUT'
    if abs(v * 10 - round(v * 10)) > 1e-6: return None, 'SPRINT_STEP'
    hi = round(D - p['coast_m'], 1)
    if not p['finish_min_m'] - 1e-9 <= v <= hi + 1e-9: return None, 'SPRINT_FINISH_RANGE'
    return round(v, 1) + 0., None


def plan(back, p, finish=None):
    """the sprint zones (list of dicts, in order) and the summary numbers for back (pure)."""
    D = SC.d_of(back); bf = round(D - p['brake_before_c1s_m'], 3)
    zones = []
    if finish is None or finish > bf:
        zones.append({'name': 'sprint after-cap: full throttle ends %.2f m before C1s (start + %.2f m); every later flat frame <= %g (field-20261124 sprint)' % (p['brake_before_c1s_m'], bf, p['after_cap_pwm']),
                      'anchor': 'start', 'from_m': bf, 'to_m': FAR_M, 'cap_pwm': p['after_cap_pwm'], ZONE_KEY: True})
    if finish is not None:
        zones.append({'name': 'sprint finish: PWM 0 from start + %.1f m (soft stop, the run stays active; field-20261124 sprint)' % finish,
                      'anchor': 'start', 'from_m': finish, 'to_m': FAR_M, 'cap_pwm': 0.0, ZONE_KEY: True})
    return zones, dict(back_m=back, D=D, brake_from_m=bf, finish_m=finish)


def is_zone(z): return isinstance(z, dict) and z.get(ZONE_KEY) is True


def build_on(sa, bz, back, p, finish=None):
    """(new speed_adapt, new brake_zones) objects (pure). sa / bz must be sprint-free."""
    if SA_KEY in sa or any(is_zone(z) for z in bz.get('zones') or []): raise Refused('SPRINT_ALREADY_ON')
    for k in SA_KEYS:
        if not SC.num(sa.get(k)): raise Refused('SPRINT_FILE_ERROR', ['speed_adapt.json 沒有數值 %s' % k])
    if not isinstance(bz.get('zones'), list): raise Refused('SPRINT_FILE_ERROR', ['brake_zones.json 沒有 zones 清單'])
    zones, info = plan(back, p, finish)
    sa2 = copy.deepcopy(sa); bz2 = copy.deepcopy(bz)
    orig = {k: sa[k] for k in SA_KEYS}
    sa2['hard_cap'] = max(p['max_pwm'], float(sa['hard_cap'])); sa2['max_pwm'] = p['max_pwm']; sa2['up_rate'] = p['up_rate']
    sa2[SA_KEY] = dict(revision=REVISION, orig=orig, back_m=back, D=info['D'], brake_before_c1s_m=p['brake_before_c1s_m'], brake_from_m=info['brake_from_m'],
                       finish_m=finish, after_cap_pwm=p['after_cap_pwm'], up_rate=p['up_rate'], max_pwm=p['max_pwm'],
                       note='Straight-line sprint mode (phone page). OFF = these keys back to orig, this key and the brake_zones "sprint" zones removed.')
    bz2['zones'] = list(bz2['zones']) + zones
    return sa2, bz2


def build_off(sa, bz):
    """(new speed_adapt, new brake_zones) objects with every sprint trace removed (pure). Works on partial states too."""
    sa2 = copy.deepcopy(sa); bz2 = copy.deepcopy(bz)
    sm = sa2.pop(SA_KEY, None)
    if isinstance(sm, dict) and isinstance(sm.get('orig'), dict):
        for k in SA_KEYS:
            if k in sm['orig']: sa2[k] = sm['orig'][k]
    if isinstance(bz2.get('zones'), list): bz2['zones'] = [z for z in bz2['zones'] if not is_zone(z)]
    return sa2, bz2


def read_all(cfg_dir):
    """{name: dict(raw, obj, style)} of the five files (start_cfg.read_files + speed_adapt.json)."""
    out = SC.read_files(cfg_dir); bad = []
    p = Path(cfg_dir) / 'speed_adapt.json'
    try:
        raw = p.read_bytes(); obj = json.loads(raw.decode('utf8'))
        if not isinstance(obj, dict): raise ValueError('not an object')
        out['speed_adapt.json'] = dict(raw=raw, obj=obj, style=SC.detect(raw.decode('utf8'), obj))
    except FileNotFoundError: bad.append('speed_adapt.json 不存在')
    except (OSError, ValueError, UnicodeDecodeError) as e: bad.append('speed_adapt.json 讀不到或不是正確的 JSON（%s）' % type(e).__name__)
    if bad: raise Refused('SPRINT_FILE_ERROR', bad)
    return out


def judge(files, ref, p):
    """sprint state from the parsed files (pure): dict(on, partial, consistent, reason, problems, back_m, D, brake_from_m, finish_m, start_ok, ...)."""
    sa = files['speed_adapt.json']['obj']; bz = files['brake_zones.json']['obj']
    st = SC.judge(files, ref)
    sm = sa.get(SA_KEY); zs = [z for z in (bz.get('zones') or []) if is_zone(z)]
    out = dict(revision=REVISION, on=False, partial=False, consistent=True, reason=None, problems=[], back_m=st['back_m'], start_ok=bool(st['consistent']),
               D=None if st['back_m'] is None else SC.d_of(st['back_m']), brake_from_m=None, finish_m=None, max_pwm=None, up_rate=None, after_cap_pwm=None,
               brake_before_c1s_m=p['brake_before_c1s_m'], finish_max_m=None if st['back_m'] is None else round(SC.d_of(st['back_m']) - p['coast_m'], 1),
               can_on=st['back_m'] is not None and st['back_m'] >= p['min_back_m'] and bool(st['consistent']), min_back_m=p['min_back_m'])
    if sm is None and not zs: return out
    out['on'] = True
    if not isinstance(sm, dict) or not zs:
        out.update(partial=True, consistent=False, reason='SPRINT_PARTIAL',
                   problems=['speed_adapt.json 有衝刺設定（油門上限 100），但 brake_zones.json 沒有衝刺減速區' if zs == [] else 'brake_zones.json 有衝刺減速區，但 speed_adapt.json 沒有衝刺設定'])
        return out
    out.update(brake_from_m=sm.get('brake_from_m'), finish_m=sm.get('finish_m'), max_pwm=sa.get('max_pwm'), up_rate=sa.get('up_rate'), after_cap_pwm=sm.get('after_cap_pwm'),
               brake_before_c1s_m=sm.get('brake_before_c1s_m'), set_back_m=sm.get('back_m'))
    probs = []
    try:
        pp = dict(p, brake_before_c1s_m=float(sm['brake_before_c1s_m']), after_cap_pwm=float(sm['after_cap_pwm']), up_rate=float(sm['up_rate']), max_pwm=float(sm['max_pwm']))
        want, _ = plan(float(sm['back_m']), pp, sm.get('finish_m'))
        if [{k: v for k, v in z.items() if k != 'name'} for z in zs] != [{k: v for k, v in z.items() if k != 'name'} for z in want]:
            probs.append('brake_zones.json 的衝刺減速區和 speed_adapt.json 記錄的參數對不上')
        if not (SC.same(sa.get('max_pwm'), pp['max_pwm']) and SC.same(sa.get('up_rate'), pp['up_rate']) and SC.num(sa.get('hard_cap')) and float(sa['hard_cap']) >= pp['max_pwm']):
            probs.append('speed_adapt.json 的 hard_cap／max_pwm／up_rate 不是衝刺值')
        if st['back_m'] is None or not SC.same(sm['back_m'], st['back_m']):
            probs.append('衝刺設定是依起點「旗子往後 %s m」算的，但目前起點是 %s' % (sm.get('back_m'), '不明' if st['back_m'] is None else '%.1f m' % st['back_m']))
    except Exception as e: probs.append('衝刺設定結構不符預期（%s）' % type(e).__name__)
    if probs: out.update(consistent=False, reason='SPRINT_INCONSISTENT', problems=probs)
    return out


def state(cfg_dir, ref_path, p):
    try: files = read_all(cfg_dir)
    except SC.Refused as e: return dict(revision=REVISION, on=None, partial=False, consistent=False, reason=e.code if e.code.startswith('SPRINT') else 'SPRINT_FILE_ERROR', problems=e.problems, can_on=False)
    try: return judge(files, SC.load_ref(ref_path), p)
    except Exception as e: return dict(revision=REVISION, on=None, partial=False, consistent=False, reason='SPRINT_FILE_ERROR', problems=['檔案結構不符預期（%s）' % type(e).__name__], can_on=False)


def _bytes(files, new):
    data = {}
    for n in FILES:
        data[n] = SC.dumps(new[n], files[n]['style'] or SC.DEFAULT_STYLE).encode('utf8')
        if json.loads(data[n].decode('utf8')) != new[n]: raise Refused('SPRINT_SELF_CHECK', ['%s 重新解析與預期不同' % n])
    return data


def make(cfg_dir, ref_path, on, finish_in=None, p=None):
    """(files, new objects, new bytes, write order, info) or Refused. Nothing is written."""
    p = p or params(); files = read_all(cfg_dir); ref = SC.load_ref(ref_path); now = judge(files, ref, p)
    sa = files['speed_adapt.json']['obj']; bz = files['brake_zones.json']['obj']
    if on:
        if now['on']: raise Refused('SPRINT_ALREADY_ON' if now['consistent'] else now['reason'], now['problems'])
        st = SC.judge(files, ref)
        if not st['consistent']: raise Refused('SPRINT_START_BAD', st['problems'] or [SC_reason(st)])
        if st['back_m'] < p['min_back_m'] - 1e-9: raise Refused('SPRINT_BACK_TOO_NEAR', ['目前起點旗子往後 %.1f m；直線衝刺要 %.1f m 以上（起步左舵要是關的）' % (st['back_m'], p['min_back_m'])])
        finish, code = parse_finish(finish_in, SC.d_of(st['back_m']), p)
        if code: raise Refused(code)
        sa2, bz2 = build_on(sa, bz, st['back_m'], p, finish); order = ('brake_zones.json', 'speed_adapt.json')
        info = dict(on=True, back_m=st['back_m'], finish_m=finish, **{k: v for k, v in plan(st['back_m'], p, finish)[1].items() if k not in ('back_m', 'finish_m')})
    else:
        sa2, bz2 = build_off(sa, bz); order = ('speed_adapt.json', 'brake_zones.json'); info = dict(on=False)
    new = {'speed_adapt.json': sa2, 'brake_zones.json': bz2}
    data = _bytes(files, new)
    after = judge(dict(files, **{n: dict(obj=new[n]) for n in FILES}), ref, p)
    if bool(after['on']) != bool(on) or (on and not after['consistent']): raise Refused('SPRINT_SELF_CHECK', after['problems'] or ['套用後讀不回預期狀態'])
    # only the sprint keys may change: everything else of both files is identical after removing the sprint traces
    if build_off(sa2, bz2) != build_off(sa, bz): raise Refused('SPRINT_SELF_CHECK', ['會改到衝刺以外的鍵'])
    return files, new, data, order, info


def SC_reason(st): return '起點設定不一致（%s）' % st.get('reason')


def _write(path, data):
    tmp = path.with_name(path.name + '.sprintcfg.tmp')
    with open(tmp, 'wb') as f: f.write(data); f.flush(); os.fsync(f.fileno())
    return tmp


def apply(cfg_dir, ref_path, backups_dir, on, finish_in=None, p=None, log=None):
    """write sprint ON / OFF. Returns dict(on, changed files, backup, files hashes, ...). Raises Refused. Same pattern as start_cfg.apply:
    compute + verify -> back up the two originals -> write .tmp (fsync) -> replace in the safe order -> read back; failure -> originals back."""
    with SC.LOCK:
        files, new, data, order, info = make(cfg_dir, ref_path, on, finish_in, p)
        cfg = Path(cfg_dir); todo = [n for n in order if data[n] != files[n]['raw']]
        res = dict(info, changed=todo, backup=None, files={n: hashlib.sha256(data[n]).hexdigest()[:8] for n in FILES})
        if not todo: return res
        b = SC._backup_dir(backups_dir, 'sprintcfg_')
        for n in FILES: (b / n).write_bytes(files[n]['raw'])
        (b / 'README.txt').write_text('sprint-cfg: before sprint %s at %s\n%s\n' % ('ON' if on else 'OFF', time.strftime('%Y-%m-%d %H:%M:%S'), json.dumps(info, ensure_ascii=False)), encoding='utf8')
        res['backup'] = b.name; tmps = {}; done = []
        try:
            for n in todo: tmps[n] = _write(cfg / n, data[n])
            for n in todo: SC._replace(tmps[n], cfg / n); done.append(n)
            bad = [n for n in FILES if (cfg / n).read_bytes() != data[n]]
            if bad: raise IOError('read back differs: ' + ', '.join(bad))
        except Exception as e:
            restored, lost = [], []
            for n in reversed(done or todo):   # undo in the reverse order: the speed_adapt 100 goes first, the after-cap last
                try:
                    if (cfg / n).read_bytes() != files[n]['raw']: os.replace(_write(cfg / n, files[n]['raw']), cfg / n)
                    restored.append(n)
                except Exception: lost.append(n)
            for t in tmps.values():
                try: t.unlink()
                except OSError: pass
            if log: log('sprint-cfg %s FAILED (%s); restored %s; not restored %s; backup %s' % ('on' if on else 'off', type(e).__name__, restored, lost, b.name))
            raise Refused('SPRINT_WRITE_FAILED', ['寫入失敗（%s），已放回原檔：%s%s' % (type(e).__name__, ', '.join(restored) or '無', '；放不回：' + ', '.join(lost) + '（備份在 ' + b.name + '）' if lost else '')], dict(backup=b.name))
        if log: log('sprint-cfg %s: %s; files %s; backup %s' % ('ON' if on else 'OFF', json.dumps(info, ensure_ascii=False), todo, b.name))
        return res


if __name__ == '__main__':   # Orin shell: python3 sprint_cfg.py status   (on / off only through the page: it checks the run state)
    import sys
    CFG = Path.home() / 'orin_interface_v1/web'; REF = Path(__file__).resolve().with_name('start_ref.json')
    print(json.dumps(state(CFG, REF, params()), ensure_ascii=False, indent=1))
