"""start-cfg-r1 (mobile-service-r6, 2026-11-23): start position = "the car stands back_m metres behind the flag spot".
Standard library only; runs on the Orin inside the phone service. No car control module is imported.

Rule (user, 2026-11-23; the same as field-20261123/startcfg/mkstart_rule.py):
  D = 3.44 + back_m, minus 0.25 when back_m > 0 (built-in under-report); D = 3.44 for back_m = 0. back_m 0..7, step 0.1.  delta = D - 3.44.
  lap_pos.json      start_s = -D ; skip_after_by.C1s = 9.0 (always)
  brake_zones.json  every zone with anchor 'start' and from_m < 900: from_m / to_m = flag value + delta (the 999 dummy pieces stay)
  lap_ff.json       curves.c1.start.gap_m = flag value + delta (win_m / ref_win untouched)
  start_bias.json   enabled = (back_m <= 2.5); every other key untouched
Only these keys change; every other key keeps its value, and each file keeps its own JSON layout (indent, ensure_ascii, line ends --
detected from the file itself, so a round trip ends byte-identical).

Source of truth:
  - the present back_m is read from lap_pos.json start_s (the 0.1 m grid maps every D to exactly one back_m);
  - the flag-spot values of the shifted keys (each start zone's from_m / to_m by zone name, and the c1 gap_m) live in start_ref.json
    next to the phone service -- never in the car's files (no extra key for the car control to meet).
  - consistent = every shifted key equals flag value + delta of start_s, skip_after_by.C1s is 9.0 and start_bias.enabled follows the rule.
  - not consistent because of a shifted key (zone / gap_m / unknown start_s / no reference): set refuses; the page shows each
    difference and offers "rebuild the reference from the car's files" (= declare the present files correct; never automatic).
    Differences in the absolute keys only (skip_after_by.C1s, start_bias.enabled) are repaired by the next set.
sprint-cfg-r1 (field-20261124): brake zones with "sprint": true (written by sprint_cfg.py) are skipped by start_zones(); without such zones
every result is unchanged. The phone service refuses a start-position change while sprint mode is on."""
import copy, hashlib, json, os, threading, time
from pathlib import Path

REVISION = 'start-cfg-r1'
REF_D, UNDER_M, BACK_MIN, BACK_MAX, NEAR_MAX, C1S_SKIP = 3.44, 0.25, 0.0, 7.0, 2.5, 9.0
FILES = ('lap_pos.json', 'brake_zones.json', 'lap_ff.json', 'start_bias.json')
DEFAULT_STYLE = (1, False, '\n', True, None)
STYLES = [(ind, ea, nl, tr, sep) for ind in (1, None, 2, 4) for ea in (False, True) for nl in ('\n', '\r\n') for tr in (True, False)
          for sep in ((None, (',', ':')) if ind is None else (None,))]
LOCK = threading.Lock()
_replace = os.replace   # tests replace this to simulate a failing write


class Refused(Exception):
    def __init__(self, code, problems=None, detail=None):
        Exception.__init__(self, code); self.code = code; self.problems = problems or []; self.detail = detail or {}


# ----------------------------------------------------------------------------------------------------------- rule
def d_of(back): return round(REF_D + back - (UNDER_M if back > 0 else 0.), 2)


def delta_of(back): return round(d_of(back) - REF_D, 3)


def parse_back(v):
    """(back_m, None) or (None, code). A number 0..7 with at most one decimal."""
    if isinstance(v, bool) or not isinstance(v, (int, float)): return None, 'START_BAD_INPUT'
    v = float(v)
    if v != v or v in (float('inf'), float('-inf')): return None, 'START_BAD_INPUT'
    if not BACK_MIN - 1e-9 <= v <= BACK_MAX + 1e-9: return None, 'START_RANGE'
    if abs(v * 10 - round(v * 10)) > 1e-6: return None, 'START_STEP'
    return round(v, 1) + 0., None   # + 0. turns -0.0 into 0.0


def back_of_start_s(start_s):
    """the grid back_m whose D equals -start_s, or None."""
    if isinstance(start_s, bool) or not isinstance(start_s, (int, float)): return None
    hits = [i / 10. for i in range(int(BACK_MAX * 10) + 1) if abs(-d_of(i / 10.) - float(start_s)) < 1e-6]
    return hits[0] if len(hits) == 1 else None


def num(x): return isinstance(x, (int, float)) and not isinstance(x, bool)


def same(a, b): return num(a) and num(b) and round(float(a), 3) == round(float(b), 3)


# ----------------------------------------------------------------------------------------------------------- files
def dumps(obj, style):
    ind, ea, nl, tr, sep = style
    t = json.dumps(obj, ensure_ascii=ea, indent=ind, separators=sep)
    if nl != '\n': t = t.replace('\n', nl)   # json.dumps escapes newlines inside strings, so every real newline is layout
    return t + (nl if tr else '')


def detect(text, obj):
    for st in STYLES:
        if dumps(obj, st) == text: return st
    return None


def read_files(cfg_dir):
    """{name: dict(raw=bytes, obj=parsed, style=layout or None)}; Refused('START_FILE_ERROR') when a file is missing or unreadable."""
    out = {}; bad = []
    for n in FILES:
        p = Path(cfg_dir) / n
        try:
            raw = p.read_bytes(); obj = json.loads(raw.decode('utf8'))
            if not isinstance(obj, dict): raise ValueError('not an object')
            out[n] = dict(raw=raw, obj=obj, style=detect(raw.decode('utf8'), obj))
        except FileNotFoundError: bad.append('%s 不存在' % n)
        except (OSError, ValueError, UnicodeDecodeError) as e: bad.append('%s 讀不到或不是正確的 JSON（%s）' % (n, type(e).__name__))
    if bad: raise Refused('START_FILE_ERROR', bad)
    return out


def start_zones(bz):
    """[(key, zone)] for zones with anchor 'start' and from_m < 900; key = name (+ '#k' for the k-th zone with the same name)."""
    out = []; seen = {}
    for z in bz.get('zones') or []:
        if isinstance(z, dict) and z.get('sprint') is True: continue   # sprint-cfg-r1 (field-20261124): the sprint zones are not start-position zones
        if isinstance(z, dict) and z.get('anchor') == 'start' and num(z.get('from_m')) and float(z['from_m']) < 900:
            name = str(z.get('name') or '(no name)'); seen[name] = seen.get(name, 0) + 1
            out.append((name if seen[name] == 1 else '%s#%d' % (name, seen[name]), z))
    return out


def c1_start(lf):
    st = ((lf.get('curves') or {}).get('c1') or {}).get('start')
    return st if isinstance(st, dict) else None


def short(key): return key if len(key) <= 40 else key[:38] + '…'


# ----------------------------------------------------------------------------------------------------------- reference
def load_ref(ref_path):
    try: r = json.loads(Path(ref_path).read_text(encoding='utf8'))
    except FileNotFoundError: return None
    except (OSError, ValueError): return 'bad'
    ok = (isinstance(r, dict) and num(r.get('c1_gap_m')) and isinstance(r.get('zones'), list)
          and all(isinstance(z, dict) and isinstance(z.get('key'), str) and num(z.get('from_m')) and num(z.get('to_m')) for z in r['zones']))
    return r if ok else 'bad'


def ref_from_files(files, back):
    """flag-spot values = the present values - delta(back). Refused when a needed key is missing."""
    d = delta_of(back); bz = files['brake_zones.json']['obj']; st = c1_start(files['lap_ff.json']['obj'])
    if st is None or not num(st.get('gap_m')): raise Refused('START_FILE_ERROR', ['lap_ff.json 沒有 curves.c1.start.gap_m'])
    zones = []
    for key, z in start_zones(bz):
        if not num(z.get('to_m')): raise Refused('START_FILE_ERROR', ['brake_zones.json「%s」沒有 to_m' % short(key)])
        zones.append(dict(key=key, from_m=round(float(z['from_m']) - d, 3), to_m=round(float(z['to_m']) - d, 3)))
    return dict(revision=REVISION, c1_gap_m=round(float(st['gap_m']) - d, 3), zones=zones)


# ----------------------------------------------------------------------------------------------------------- state
def judge(files, ref):
    """dict(back_m, D, delta_m, left_bias_on, consistent, reason, problems, fixable) from parsed files + reference (pure)."""
    lp = files['lap_pos.json']['obj']; bz = files['brake_zones.json']['obj']; lf = files['lap_ff.json']['obj']; sb = files['start_bias.json']['obj']
    start_s = lp.get('start_s'); back = back_of_start_s(start_s)
    out = dict(back_m=back, D=None if not num(start_s) else round(-float(start_s), 3), delta_m=None if back is None else delta_of(back),
               left_bias_on=sb.get('enabled') is True, consistent=False, reason=None, problems=[], fixable=[])
    if back is None:
        out.update(reason='START_S_UNKNOWN', problems=['lap_pos.json start_s＝%s，不是本規則（旗子往後 0～7 m、0.1 m 一格）產生的值' % json.dumps(start_s)]); return out
    blocking, fixable = [], []
    sk = lp.get('skip_after_by'); c1s = sk.get('C1s') if isinstance(sk, dict) else None
    if not same(c1s, C1S_SKIP): fixable.append('lap_pos.json skip_after_by.C1s 應為 9.0，現在是 %s' % json.dumps(c1s))
    if sb.get('enabled') is not (back <= NEAR_MAX):
        fixable.append('start_bias.json enabled 應為 %s（旗子往後 %.1f m），現在是 %s' % ('true' if back <= NEAR_MAX else 'false', back, json.dumps(sb.get('enabled'))))
    if ref is None: blocking.append('手機服務沒有旗子參考值檔（start_ref.json）')
    elif ref == 'bad': blocking.append('旗子參考值檔（start_ref.json）壞掉或格式不對')
    else:
        d = delta_of(back); st = c1_start(lf)
        if st is None or not num(st.get('gap_m')): blocking.append('lap_ff.json 沒有 curves.c1.start.gap_m')
        elif not same(st['gap_m'], float(ref['c1_gap_m']) + d):
            blocking.append('lap_ff.json c1.start.gap_m 現在 %s，依參考應為 %s' % (st['gap_m'], round(float(ref['c1_gap_m']) + d, 3)))
        cur = dict(start_zones(bz)); want = {z['key']: z for z in ref['zones']}
        for k in cur:
            if k not in want: blocking.append('brake_zones.json 多了參考裡沒有的起跑區「%s」' % short(k))
        for k, z in want.items():
            if k not in cur: blocking.append('brake_zones.json 少了參考裡的起跑區「%s」' % short(k)); continue
            for f in ('from_m', 'to_m'):
                if not same(cur[k].get(f), float(z[f]) + d):
                    blocking.append('brake_zones.json「%s」%s 現在 %s，依參考應為 %s' % (short(k), f, cur[k].get(f), round(float(z[f]) + d, 3)))
    out['problems'] = blocking + fixable; out['fixable'] = fixable
    out['consistent'] = not out['problems']
    out['reason'] = None if out['consistent'] else ('START_NO_REF' if ref is None or ref == 'bad' else 'START_INCONSISTENT' if blocking else 'START_NEEDS_APPLY')
    return out


def state(cfg_dir, ref_path):
    try: files = read_files(cfg_dir)
    except Refused as e:
        return dict(back_m=None, D=None, delta_m=None, left_bias_on=None, consistent=False, reason=e.code, problems=e.problems, fixable=[])
    try: s = judge(files, load_ref(ref_path))
    except Exception as e:   # an unexpected structure (e.g. curves not an object) must not break the status of the whole page
        return dict(back_m=None, D=None, delta_m=None, left_bias_on=None, consistent=False, reason='START_FILE_ERROR', problems=['檔案結構不符預期（%s）' % type(e).__name__], fixable=[])
    s['styles'] = {n: f['style'] is not None for n, f in files.items()}
    return s


# ----------------------------------------------------------------------------------------------------------- plan / apply
def build(files, ref, back):
    """{name: new object} for back (pure). Only the rule keys are assigned, and only when the value differs (keeps 9 vs 9.0, 2 vs 2.0)."""
    d = delta_of(back); D = d_of(back); new = {n: copy.deepcopy(f['obj']) for n, f in files.items()}
    lp = new['lap_pos.json']; target = -REF_D if d == 0 else -round(D, 3)
    if not same(lp.get('start_s'), target): lp['start_s'] = target
    sk = lp.get('skip_after_by')
    if not isinstance(sk, dict): lp['skip_after_by'] = {'C1s': C1S_SKIP}
    elif not same(sk.get('C1s'), C1S_SKIP): sk['C1s'] = C1S_SKIP
    want = {z['key']: z for z in ref['zones']}
    for key, z in start_zones(new['brake_zones.json']):
        for f in ('from_m', 'to_m'):
            t = round(float(want[key][f]) + d, 3)
            if not same(z.get(f), t): z[f] = t
    st = c1_start(new['lap_ff.json']); t = round(float(ref['c1_gap_m']) + d, 3)
    if not same(st.get('gap_m'), t): st['gap_m'] = t
    sb = new['start_bias.json']
    if sb.get('enabled') is not (back <= NEAR_MAX): sb['enabled'] = back <= NEAR_MAX
    return new


def changes(old, new, path=''):
    """['path: old -> new'] (dicts by key, lists by index)."""
    out = []
    if isinstance(old, dict) and isinstance(new, dict):
        for k in list(old) + [k for k in new if k not in old]:
            if k not in old: out.append('%s.%s: (無) -> %s' % (path, k, json.dumps(new[k], ensure_ascii=False)))
            elif k not in new: out.append('%s.%s: 被刪除' % (path, k))
            else: out += changes(old[k], new[k], '%s.%s' % (path, k))
    elif isinstance(old, list) and isinstance(new, list) and len(old) == len(new):
        for i, (a, b) in enumerate(zip(old, new)): out += changes(a, b, '%s[%d]' % (path, i))
    elif old != new or type(old) is not type(new): out.append('%s: %s -> %s' % (path, json.dumps(old, ensure_ascii=False), json.dumps(new, ensure_ascii=False)))
    return out


def allowed(name, path, files):
    if name == 'lap_pos.json': return path in ('.start_s', '.skip_after_by', '.skip_after_by.C1s')
    if name == 'lap_ff.json': return path == '.curves.c1.start.gap_m'
    if name == 'start_bias.json': return path == '.enabled'
    zs = files['brake_zones.json']['obj'].get('zones') or []
    idx = {i for i, z in enumerate(zs) if any(z is y for _, y in start_zones(files['brake_zones.json']['obj']))}
    return any(path in ('.zones[%d].from_m' % i, '.zones[%d].to_m' % i) for i in idx)


def plan(cfg_dir, ref_path, back_in, force=False):
    """(back, files, new objects, new bytes, change list) or Refused. Nothing is written."""
    try: return _plan(cfg_dir, ref_path, back_in, force)
    except Refused: raise
    except Exception as e: raise Refused('START_FILE_ERROR', ['檔案結構不符預期（%s）' % type(e).__name__])


def _plan(cfg_dir, ref_path, back_in, force=False):
    back, code = parse_back(back_in)
    if code: raise Refused(code)
    files = read_files(cfg_dir); ref = load_ref(ref_path); now = judge(files, ref)
    # r6a (2026-11-23, user: a write cut by a power loss must be repairable from the phone): force = rewrite every rule key from start_ref.json + back,
    # whatever the files hold now. Needs a readable reference; lap-1 zone values tuned by hand since the last rebuild are overwritten (the page says so).
    if now['reason'] in ('START_S_UNKNOWN', 'START_NO_REF', 'START_INCONSISTENT') and not (force is True and isinstance(ref, dict) and now['reason'] != 'START_NO_REF'):
        raise Refused(now['reason'], now['problems'], dict(now=now))
    new = build(files, ref, back); data = {}; diff = []
    for n in FILES:
        st = files[n]['style'] or DEFAULT_STYLE
        data[n] = dumps(new[n], st).encode('utf8')
        if json.loads(data[n].decode('utf8')) != new[n]: raise Refused('START_SELF_CHECK', ['%s 重新解析與預期不同' % n])
        for line in changes(files[n]['obj'], new[n]):
            p = line.split(':', 1)[0]
            if not allowed(n, p, files): raise Refused('START_SELF_CHECK', ['%s 會改到規則以外的鍵 %s' % (n, p)])
            diff.append(n + line)
    after = judge({n: dict(obj=new[n]) for n in FILES}, ref)
    if not after['consistent'] or after['back_m'] != back: raise Refused('START_SELF_CHECK', after['problems'] or ['套用後讀不回 %.1f m' % back])
    return back, files, new, data, diff


def _write(path, data):
    tmp = path.with_name(path.name + '.startcfg.tmp')
    with open(tmp, 'wb') as f: f.write(data); f.flush(); os.fsync(f.fileno())
    return tmp


def _backup_dir(backups_dir, prefix):
    base = Path(backups_dir) / (prefix + time.strftime('%Y%m%d_%H%M%S')); b = base; k = 1
    while b.exists(): k += 1; b = base.with_name('%s_%d' % (base.name, k))
    b.mkdir(parents=True); return b


def apply(cfg_dir, ref_path, backups_dir, back_in, log=None, force=False):
    """write the four files for back_in. Returns dict(back_m, D, left_bias_on, changed, backup, files). Raises Refused.
    Order: compute + verify everything -> back up the four originals -> write every .tmp (fsync) -> replace one by one ->
    read back. Any failure after the first replace puts the original bytes back (START_WRITE_FAILED)."""
    with LOCK:
        back, files, new, data, diff = plan(cfg_dir, ref_path, back_in, force)
        cfg = Path(cfg_dir); res = dict(back_m=back, D=d_of(back), left_bias_on=back <= NEAR_MAX, changed=diff, backup=None,
                                        files={n: hashlib.sha256(data[n]).hexdigest()[:8] for n in FILES})
        todo = [n for n in FILES if data[n] != files[n]['raw']]
        if not todo: return res
        b = _backup_dir(backups_dir, 'startcfg_')
        for n in FILES: (b / n).write_bytes(files[n]['raw'])
        if Path(ref_path).exists(): (b / Path(ref_path).name).write_bytes(Path(ref_path).read_bytes())
        (b / 'README.txt').write_text('start-cfg: before setting back_m %.1f (D %.2f) at %s\nchanges:\n%s\n' % (back, d_of(back), time.strftime('%Y-%m-%d %H:%M:%S'), '\n'.join(diff)), encoding='utf8')
        res['backup'] = b.name; tmps = {}; done = []
        try:
            for n in todo: tmps[n] = _write(cfg / n, data[n])
            for n in todo: _replace(tmps[n], cfg / n); done.append(n)
            bad = [n for n in FILES if (cfg / n).read_bytes() != data[n]]
            if bad: raise IOError('read back differs: ' + ', '.join(bad))
            if judge({n: dict(obj=json.loads((cfg / n).read_bytes().decode('utf8'))) for n in FILES}, load_ref(ref_path))['back_m'] != back:
                raise IOError('read back: back_m differs')
        except Exception as e:
            restored, lost = [], []
            for n in done or todo:
                try:
                    if (cfg / n).read_bytes() != files[n]['raw']: os.replace(_write(cfg / n, files[n]['raw']), cfg / n)
                    restored.append(n)
                except Exception: lost.append(n)
            for t in tmps.values():
                try: t.unlink()
                except OSError: pass
            if log: log('start-cfg set %.1f FAILED (%s); restored %s; not restored %s; backup %s' % (back, type(e).__name__, restored, lost, b.name))
            raise Refused('START_WRITE_FAILED', ['寫入失敗（%s），已放回原檔：%s%s' % (type(e).__name__, ', '.join(restored) or '無',
                                                                                 '；放不回：' + ', '.join(lost) + '（備份在 ' + b.name + '）' if lost else '')], dict(backup=b.name))
        if log: log('start-cfg set%s back %.1f m (D %.2f, left bias %s): %d keys, backup %s' % (' FORCED' if force else '', back, d_of(back), 'on' if back <= NEAR_MAX else 'off', len(diff), b.name))
        return res


def rebuild_ref(cfg_dir, ref_path, backups_dir, log=None):
    """declare the car's present four files correct: start_ref.json = present values - delta(start_s). Returns dict(back_m, ref, problems)."""
    with LOCK:
        files = read_files(cfg_dir); back = back_of_start_s(files['lap_pos.json']['obj'].get('start_s'))
        if back is None: raise Refused('START_S_UNKNOWN', judge(files, None)['problems'])
        try: ref = ref_from_files(files, back)
        except Refused: raise
        except Exception as e: raise Refused('START_FILE_ERROR', ['檔案結構不符預期（%s）' % type(e).__name__])
        ref['built_at'] = time.strftime('%Y-%m-%d %H:%M:%S'); ref['built_from_back_m'] = back
        ref['note'] = 'flag-spot (back 0 m) values of the start-anchored keys; built from the car files at back %.1f m. Phone service only; the car control never reads this file.' % back
        p = Path(ref_path)
        if p.exists():
            b = _backup_dir(backups_dir, 'startref_'); (b / p.name).write_bytes(p.read_bytes())
        tmp = p.with_name(p.name + '.tmp'); tmp.write_text(json.dumps(ref, ensure_ascii=False, indent=1) + '\n', encoding='utf8'); os.replace(tmp, p)
        if log: log('start-cfg reference rebuilt from the car files at back %.1f m (%d zones, gap %.3f)' % (back, len(ref['zones']), ref['c1_gap_m']))
        return dict(back_m=back, ref=ref, problems=judge(files, ref)['problems'])


if __name__ == '__main__':   # Orin shell: python3 start_cfg.py status | plan <back_m> | rebuild-ref   (set only through the page: it checks the run state)
    import sys
    CFG = Path.home() / 'orin_interface_v1/web'; REF = Path(__file__).resolve().with_name('start_ref.json'); BK = Path.home() / 'orin_interface_v1/backups'
    a = sys.argv[1:] or ['status']
    try:
        if a[0] == 'status': print(json.dumps(state(CFG, REF), ensure_ascii=False, indent=1))
        elif a[0] == 'plan': r = plan(CFG, REF, float(a[1])); print('back %.1f m -> D %.2f\n' % (r[0], d_of(r[0])) + '\n'.join(r[4]))
        elif a[0] == 'rebuild-ref': print(json.dumps(rebuild_ref(CFG, REF, BK), ensure_ascii=False, indent=1))
        else: print(__doc__)
    except Refused as e: print('REFUSED', e.code, *e.problems, sep='\n'); sys.exit(1)
