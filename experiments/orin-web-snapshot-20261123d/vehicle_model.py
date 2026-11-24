"""vehicle-model-r1 (cleanup-20261117, offline candidate): the car constants in one place.

Mechanism only. web/vehicle_model.json has two parts:
  "use"        {name: number}  - values that override a module's own constant. EMPTY on delivery: every module keeps its own value.
  "inventory"  documentation of where each constant lives today and its value(s) there (never read by code).
A module asks get(name, default): it gets use[name] when that is a finite number, otherwise its own default (unchanged behaviour).
Read once at import (a change needs a car-control restart, like the other import-time configs). Missing / unreadable / malformed file,
non-numeric value -> default. Never raises. Standard library only, no clock / IMU / MCU access.

Wired in cleanup-r1: live_pid.py 'm_per_count' (encoder metres per count, the 2.22e-4 literal at three sites).
Not wired yet (design in SUMMARY_FOR_REVIEW.txt section B): centre / straight point, steering gain(v), delay, play.
"""
import json, math
from pathlib import Path

REVISION = 'vehicle-model-r1'
CONFIG_FILE = Path(__file__).with_name('vehicle_model.json')


def _load(path=None):
    try:
        f = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
        u = f.get('use') if isinstance(f, dict) else None
        if not isinstance(u, dict): return {}
        return {str(k): float(v) for k, v in u.items()
                if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v))}
    except Exception:
        return {}


USE = _load()


def get(name, default):
    """use[name] (finite number) or the caller's own default (returned as given, not converted)."""
    v = USE.get(name)
    return default if v is None else v


def overrides():
    """the active overrides (for a start-record line, if a caller wants one). {} on delivery."""
    return dict(USE)
