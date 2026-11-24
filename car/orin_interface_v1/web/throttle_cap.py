"""throttle-cap-r1 (speed-next-20261117 offline candidate, NOT deployed): the ONE Orin-side permission for PWM above 50.

web/throttle_cap.json  {"max_pct": 50}   absent / unreadable / invalid -> 50 = every check exactly as before (the old literals were 50).

What reads it (each call re-reads the file only when its mtime changed):
  manual_drive.start(_pid=True)     the PID run may start with right_pct / left_pct up to max_pct (manual page / settings stay -50..50)
  server.validate_manual_control    MANUAL frames up to max_pct (the manual page never sends more than its own 50 configuration)
  server.validate_motor_control     bench motor-test TEST frames up to max_pct
  motor_tests.test_config           bench motor-test settings up to max_pct (ground pulses stay <= 20 %)
  speed_adapt.SpeedAdapt.ceiling    the run's start ceiling is raised to max_pct only when max_pct > hard_cap (speed_adapt's own levels
                                    stay clamped to hard_cap 50)
  lap_speed                         a position zone's pwm is clamped to max_pct; speed_adapt's internal PWM is never raised above hard_cap
Only lap_speed zones can command PWM above 50; every other PWM decision stays <= speed_adapt.json hard_cap (50).
Bounds: max_pct is an integer clamped to [50, FIRMWARE_MAX] (70 = pwm70_v1 MAX_THROTTLE_PERCENT) and, when the MCU status is given,
to its reported throttle_limit_pct (a firmware rolled back to 50 can never be asked for more than 50).
Change the file only between runs (a lower value mid-run makes the running PID's higher frames fail validation -> the run ends).
Standard library only."""
import json, os
from pathlib import Path

REVISION = 'throttle-cap-r1'
CONFIG_FILE = Path(__file__).with_name('throttle_cap.json')
DEFAULT_PCT = 50
FIRMWARE_MAX = 100   # 2026-11-19 pwm100: was 70 (firmware pwm100_v1); the MCU-reported throttle_limit_pct still clamps it
_cache = {'key': None, 'val': DEFAULT_PCT}


def _read(path):
    try: st = os.stat(path)
    except OSError: return DEFAULT_PCT
    key = (str(path), st.st_mtime_ns, st.st_size)
    if _cache['key'] == key: return _cache['val']
    try:
        v = json.loads(Path(path).read_text(encoding='utf8')).get('max_pct', DEFAULT_PCT)
        v = int(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and float(v) == int(v) else DEFAULT_PCT
    except (OSError, ValueError, AttributeError, TypeError, OverflowError): v = DEFAULT_PCT
    v = max(DEFAULT_PCT, min(FIRMWARE_MAX, v))   # never below 50 (the json only widens; 50 = old behaviour)
    _cache['key'] = key; _cache['val'] = v
    return v


def max_pct(vehicle=None, path=None):
    """allowed |PWM| percent above which a PID / bench command is refused. vehicle = MCU status dict (optional)."""
    v = _read(path or CONFIG_FILE)
    if v <= DEFAULT_PCT: return v   # 50: the vehicle status is not consulted (identical to the old fixed 50 checks)
    try:
        fw = (vehicle or {}).get('throttle_limit_pct')
        if isinstance(fw, (int, float)) and not isinstance(fw, bool): v = max(DEFAULT_PCT, min(v, int(fw)))
    except Exception: pass
    return v
