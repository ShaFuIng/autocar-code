"""chatter-20261122: build candidate/downhill_speed.py (speed-meter-mcu-r1) from the snapshot-20261122c file.
python patch_speed_meter.py [--check]
Base: orin-web-snapshot-20261122c/downhill_speed.py sha256 9ec47317... Each anchor must match exactly once, else nothing is written.
live_pid.py is NOT touched (so this patch stacks with field-20261122/flags/candidate/live_pid.py and any other live_pid patch).

What it adds (SpeedMeter only; every other function / constant unchanged):
  web/speed_meter.json, read once per SpeedMeter() (= once per run start in LivePID.start, and on pause-resume):
    {"mode": "mcu"}  -> the wheel speed is the count difference over win_s of MCU status time (vehicle['ms']), one entry per new
                        status (5 Hz), instead of over Orin frame time with one entry per frame. Removes the one-frame +30 % spikes
                        (7.5 % of frames, 98 % of them on the frame a new status arrives).
    optional: win_s (default WIN_S 0.6), stale_s (default 0.45: no new status for longer than this on the Orin clock -> the old
              frame-time rate is returned, which decays to 0 as before = the old staleness behaviour).
    file absent / unreadable / mode not 'mcu' -> exactly the old code path (old rate on every call).
  Fallback to the old rate also when: vehicle['ms'] missing / not a number, fewer than 2 status entries, MCU clock went backwards or
  jumped > 2 s (entries cleared)."""
import hashlib, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXP = HERE.parents[2]
BASE = EXP / 'orin-web-snapshot-20261122c' / 'downhill_speed.py'
OUT = HERE.parent / 'candidate' / 'downhill_speed.py'
BASE_SHA = '9ec473172de734e1a95df812bdc67f3cc3e8115abc4237ee4af259cb7bd4fce9'

EDITS = [
    ("import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value\n",
     "import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value\n"
     "import json as _json   # speed-meter-mcu-r1 (field-20261122/chatter): web/speed_meter.json\n"
     "from pathlib import Path as _Path   # speed-meter-mcu-r1\n"
     "MCU_REVISION = 'speed-meter-mcu-r1'\n"
     "MCU_CONFIG = _Path(__file__).with_name('speed_meter.json')\n"),
    ("    def __init__(self):\n        self.hist = []; self.k_last = None\n",
     "    def __init__(self):\n        self.hist = []; self.k_last = None\n"
     "        self._mcu_init()   # speed-meter-mcu-r1: self.mcu None (file absent / mode not 'mcu') = the old rate on every call\n"
     "\n"
     "    def _mcu_init(self):\n"
     "        \"\"\"speed-meter-mcu-r1 (field-20261122/chatter): the frame-time rate below jumps +30 % for one frame on 37 % of the new 5 Hz\n"
     "        MCU statuses (the count steps by 0.2 s of travel while the Orin-clock window does not), 7.5 % of all frames; curve-2 / curve-4\n"
     "        bands turn each one into a one-frame 30-60 us command pulse and two play-comp flips. mode 'mcu': rate over MCU status time.\"\"\"\n"
     "        self.mcu = None; self.mh = []; self.m_last = None; self.m_seen = None\n"
     "        try:\n"
     "            c = _json.loads(MCU_CONFIG.read_text(encoding='utf8'))\n"
     "            if isinstance(c, dict) and c.get('mode') == 'mcu':\n"
     "                self.mcu = dict(win_s=float(c.get('win_s', WIN_S)), stale_s=float(c.get('stale_s', .45)))\n"
     "        except Exception: self.mcu = None\n"
     "\n"
     "    def _mcu_add(self, now, vehicle, b):\n"
     "        ms = (vehicle or {}).get('ms')\n"
     "        if isinstance(ms, bool) or not isinstance(ms, (int, float)): return\n"
     "        ms = float(ms)\n"
     "        if self.m_last is not None and ms == self.m_last: return\n"
     "        if self.m_last is not None and (ms < self.m_last or ms - self.m_last > 2000.): self.mh = []   # MCU restart / long gap\n"
     "        self.m_last = ms; self.m_seen = now; t = ms / 1000.\n"
     "        self.mh.append((t, float(b.get('forward', 0)) - float(b.get('reverse', 0))))\n"
     "        self.mh = [x for x in self.mh if t - x[0] <= 1.]\n"
     "\n"
     "    def _mcu_rate(self):\n"
     "        \"\"\"None = not usable (old rate is used).\"\"\"\n"
     "        if len(self.mh) < 2 or self.m_seen is None or not self.hist: return None\n"
     "        if self.hist[-1][0] - self.m_seen > self.mcu['stale_s']: return None\n"
     "        t1, c1 = self.mh[-1]; old = [x for x in self.mh if t1 - x[0] >= self.mcu['win_s']]\n"
     "        t0, c0 = old[-1] if old else self.mh[0]\n"
     "        return None if t1 - t0 < .05 else abs(c1 - c0) / (t1 - t0)\n"),
    ("        self.hist = [x for x in self.hist if now - x[0] <= 1.]\n        return self.rate()\n",
     "        self.hist = [x for x in self.hist if now - x[0] <= 1.]\n"
     "        if self.mcu is not None: self._mcu_add(now, vehicle, b)   # speed-meter-mcu-r1\n"
     "        return self.rate()\n"),
    ("    def rate(self):\n        if len(self.hist) < 2: return None\n",
     "    def rate(self):\n"
     "        if self.mcu is not None:   # speed-meter-mcu-r1\n"
     "            r = self._mcu_rate()\n"
     "            if r is not None: return r\n"
     "        if len(self.hist) < 2: return None\n"),
]


def build():
    raw = BASE.read_bytes()
    if hashlib.sha256(raw).hexdigest() != BASE_SHA: sys.exit(f'base sha mismatch: {hashlib.sha256(raw).hexdigest()}')
    s = raw.decode('utf8'); nl = chr(13) + chr(10) if chr(13) + chr(10) in s else chr(10)   # the car file uses CRLF; anchors are written with LF
    for a, b in EDITS:
        a = a.replace(chr(10), nl); b = b.replace(chr(10), nl)
        n = s.count(a)
        if n != 1: sys.exit(f'anchor matched {n} times (need 1): {a[:60]!r}')
        s = s.replace(a, b)
    return s.encode('utf8')


if __name__ == '__main__':
    data = build()
    if '--check' in sys.argv:
        cur = OUT.read_bytes() if OUT.exists() else b''
        print('OK' if cur == data else 'DIFFERENT', hashlib.sha256(data).hexdigest()); sys.exit(0 if cur == data else 1)
    OUT.parent.mkdir(parents=True, exist_ok=True); OUT.write_bytes(data)
    print('wrote', OUT, hashlib.sha256(data).hexdigest())
