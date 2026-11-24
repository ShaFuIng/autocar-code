"""single-centre-r1 (single-centre-20261117, offline candidate, NOT deployed): ONE steering centre for the whole car.

The one number is the shared steering centre the run already carries: params['center_us'] (= the saved servo profile centre;
LivePID.start() keeps its check "PID center must equal saved shared steering center"). No second "straight point" parameter.

What this module changes (only when web/single_centre.json says {"enabled": true}; file absent / enabled not true = nothing changes):
  (a) values that MEAN "the straight point / centre" but were written as numbers become the shared centre:
        lap_ff c_eff_us 1490 (LapFF, LapFF4 c4, arc2 c3), arc2 land straight_us 1513 / c_eff_us 1490, arc2 blind / blind_c2
        straight_us 1513 (also the v2 law's s0), bottom_guide straight_offset_us 33 (= 1513 - 1480) -> 0.
  (b) values that are an OFFSET from the centre but were written as absolute commands follow the centre:
        value -> centre + (value - DESIGN_CENTRE_US).  DESIGN_CENTRE_US = 1480 = the shared centre under which every one of these
        numbers was in use on the car (servo rev38 / PID document rev149, 11-13 15:0x .. 11-17). At centre 1480 they are unchanged.
        lap_ff curves.cN.cap_us (1800/1800/1750), curves.cN.band.cap_us (1950), c4.cap_us (1720), c4.band.cap_us (1780),
        arc2 c3.cap_us (1750), live_pid CURVE3_FF_CAP_US (1850), upper_path.json steer_us table (recorded route).
  (c) the servo mechanical ends (min_us / max_us = 780 / 2050, from the run parameters) are NOT moved.
Everything else in the control loop already works relative to params['center_us'].

Mechanism: LivePID.start() calls start(self, params['center_us']) once per run, after the per-run layer objects are built.
  - reads single_centre.json (absent / unreadable / enabled not true -> off: the module state is reset to OFF, nothing touched,
    returns None -> no start-record key);
  - on: sets the module state (used by shift() for the two module-level sites: curve3_cap in live_pid, upper_path ff) and rewrites
    the per-run config dicts of the already built objects in place (they are rebuilt from their json at every run start).
Standard library only; no clock / IMU / MCU access; never raises into the caller (any error -> off).
"""
import json, math
from pathlib import Path

REVISION = 'single-centre-r1'
CONFIG_FILE = Path(__file__).with_name('single_centre.json')
DESIGN_CENTRE_US = 1480.   # the centre under which the (b) absolute numbers were written / in use (servo rev38, PID doc rev149)

_ON = False
_C = None


def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x))


def load(path=None):
    """True only when the file exists, is a JSON object and has enabled true."""
    try:
        f = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
        return isinstance(f, dict) and f.get('enabled') is True
    except Exception:
        return False


def configure(on, centre=None):
    """module state; off -> shift() / as_centre() return their argument unchanged."""
    global _ON, _C
    if on and _num(centre): _ON, _C = True, float(centre)
    else: _ON, _C = False, None


def active():
    return _ON


def centre():
    return _C


def delta():
    """centre - DESIGN_CENTRE_US (0 when off)."""
    return (_C - DESIGN_CENTRE_US) if _ON else 0.


def shift(v):
    """(b): an absolute command written under DESIGN_CENTRE_US -> the same offset from the shared centre. Off -> v unchanged (same object)."""
    if not _ON or not _num(v): return v
    return float(v) + (_C - DESIGN_CENTRE_US)


def as_centre(v):
    """(a): a number that means 'the straight point' -> the shared centre. Off -> v unchanged."""
    if not _ON or not _num(v): return v
    return _C


def _set(d, k, fn, log, name):
    if isinstance(d, dict) and k in d and _num(d[k]):
        old = float(d[k]); d[k] = fn(d[k]); log.append([name, round(old, 1), round(float(d[k]), 1)])


def apply_lapff(o, log, name='lap_ff'):
    """LapFF (curves c1..c3 + c2 band) in place."""
    if o is None: return
    c = getattr(o, 'c', None)
    _set(c, 'c_eff_us', as_centre, log, name + '.c_eff_us')
    for k, cc in sorted(((c or {}).get('curves') or {}).items()):
        _set(cc, 'cap_us', shift, log, '%s.curves.%s.cap_us' % (name, k))
    for k, b in sorted((getattr(o, 'band', None) or {}).items()):
        _set(b, 'cap_us', shift, log, '%s.curves.%s.band.cap_us' % (name, k))


def apply_lapff4(o, log, name):
    """LapFF4 (lap_ff c4, arc2 c3) in place: merged block config c (_c_eff_us, cap_us) and the c4 band dict."""
    if o is None: return
    c = getattr(o, 'c', None)
    _set(c, '_c_eff_us', as_centre, log, name + '._c_eff_us')
    _set(c, 'cap_us', shift, log, name + '.cap_us')
    b = getattr(o, 'band', None)
    if isinstance(b, dict): _set(b, 'cap_us', shift, log, name + '.band.cap_us')


def apply_arc2(a, log):
    """arc2 objects dict (land / blind / blind_c2 / c3 / pre) in place."""
    if not isinstance(a, dict): return
    land = a.get('land')
    if land is not None:
        _set(land.c, 'straight_us', as_centre, log, 'arc2.land.straight_us')
        _set(land.c, 'c_eff_us', as_centre, log, 'arc2.land.c_eff_us')
    for k in ('blind', 'blind_c2'):
        b = a.get(k)
        if b is not None: _set(b.c, 'straight_us', as_centre, log, 'arc2.%s.straight_us' % k)
    apply_lapff4(a.get('c3'), log, 'arc2.c3')


def apply_bguide(g, log):
    if g is None: return
    _set(getattr(g, 'c', None), 'straight_offset_us', lambda v: 0. if _ON else v, log, 'bottom_guide.straight_offset_us')


def start(lp, centre_us, path=None):
    """Called once per run by LivePID.start() after the per-run objects exist. Returns the start-record info dict when on, else None.
    Off (any reason) resets the module state, so a previous on-run never leaks into this run."""
    try:
        on = load(path)
        configure(on, centre_us)
        if not _ON: return None
        log = []
        apply_lapff(getattr(lp, 'lff', None), log)
        apply_lapff4(getattr(lp, 'lff4', None), log, 'lap_ff.c4')
        apply_arc2(getattr(lp, 'arc2', None), log)
        apply_bguide(getattr(lp, 'bguide', None), log)
        return dict(revision=REVISION, centre_us=_C, design_centre_us=DESIGN_CENTRE_US, delta_us=round(delta(), 1), changed=log)
    except Exception as e:
        configure(False)
        return dict(revision=REVISION, error=type(e).__name__, state='OFF')
