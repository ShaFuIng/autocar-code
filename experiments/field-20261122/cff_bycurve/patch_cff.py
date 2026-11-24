"""curve-ff-bycurve-r1 (2026-11-22, user: tune the curves one at a time): per-curve values for the curve feed-forward floor.
curve_ff.json may carry  "by_curve": {"curve1": {"on_err": .., "full_err": .., "off_err": .., "max_frac": ..}, ...}  keyed by the
lap_pos segment name (curve1 / curve2 / curve3 / curve4 / left_bend ...). While lap_pos (not lost, this run) says the car is in such
a segment, those keys replace the global ones for that frame; everything else (gate, hysteresis state) is shared. No by_curve key =
the code path of today, frame for frame (the lap_pos lookup is not even made).
usage: python patch_cff.py <snapshot dir with curve_ff.py 5b.. and live_pid.py 800879c9> <out dir>"""
import hashlib, sys
from pathlib import Path
src, dst = Path(sys.argv[1]), Path(sys.argv[2]); dst.mkdir(parents=True, exist_ok=True)


def sha(s): return hashlib.sha256(s.encode('utf-8')).hexdigest()[:8]


def patch(name, base, pairs):
    s = (src / name).read_bytes().decode('utf-8'); assert sha(s) == base, (name, sha(s)); crlf = chr(13) + chr(10) in s
    for old, new in pairs:
        if crlf: old, new = old.replace(chr(10), chr(13) + chr(10)), new.replace(chr(10), chr(13) + chr(10))
        assert s.count(old) == 1, (name, s.count(old), old[:60]); s = s.replace(old, new)
    (dst / name).write_bytes(s.encode('utf-8')); print(name, base, '->', sha(s))


patch('curve_ff.py', sys.argv[3] if len(sys.argv) > 3 else 'x', [
    ("REVISION = 'curve-ff-r2'", "REVISION = 'curve-ff-r2'\nBY_CURVE_KEYS = ('on_err', 'full_err', 'off_err', 'max_frac')   # curve-ff-bycurve-r1"),
    ("    def step(self, steer_us, error, center_us, min_us, max_us, eligible, now=None, gz=None, path=None):\n"
     "        \"\"\"Returns (steer, info or None). r2: `now`, `gz` (deg/s, left positive) and `path` (image points) feed the curve gate.\"\"\"\n"
     "        c = self.c\n",
     "    def step(self, steer_us, error, center_us, min_us, max_us, eligible, now=None, gz=None, path=None, curve=None):\n"
     "        \"\"\"Returns (steer, info or None). r2: `now`, `gz` (deg/s, left positive) and `path` (image points) feed the curve gate.\n"
     "        curve-ff-bycurve-r1: `curve` = lap_pos segment name; by_curve[curve] replaces on_err / full_err / off_err / max_frac for this frame.\"\"\"\n"
     "        c = self.c; bc = None\n"
     "        if curve is not None and isinstance(c.get('by_curve'), dict) and isinstance(c['by_curve'].get(curve), dict):\n"
     "            bc = {k: v for k, v in c['by_curve'][curve].items() if k in BY_CURVE_KEYS and isinstance(v, (int, float)) and not isinstance(v, bool)}\n"
     "            if bc: c = dict(c, **bc)\n"
     "            else: bc = None\n"),
    ("        info = dict(revision=REVISION, side=self.side, e=round(e, 3), floor_us=round(floor, 1), pid_steer_us=round(s, 1), applied=out != s)\n",
     "        info = dict(revision=REVISION, side=self.side, e=round(e, 3), floor_us=round(floor, 1), pid_steer_us=round(s, 1), applied=out != s)\n"
     "        if bc is not None: info['by_curve'] = curve\n"),
])
patch('live_pid.py', '800879c9', [
    ("            stc,cfi=self.cff.step(proposal['steer_us'],proposal.get('image_error'),float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),elc,now=time.monotonic(),gz=gzc,path=self.latest.get('path'))   # curve-ff-r2: gate on path bend / yaw\n",
     "            cvc=None   # curve-ff-bycurve-r1: lap_pos segment for per-curve floor values; only looked up when curve_ff.json has by_curve\n"
     "            if self.cff.c.get('by_curve'):\n"
     "                try:\n"
     "                    tpc=getattr(self,'_lpos',None)\n"
     "                    if tpc is not None and getattr(self,'_lpos_run',None)==self.run['id'] and not tpc.lost:cvc=tpc.segment()\n"
     "                except Exception:cvc=None\n"
     "            stc,cfi=self.cff.step(proposal['steer_us'],proposal.get('image_error'),float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),elc,now=time.monotonic(),gz=gzc,path=self.latest.get('path'),curve=cvc)   # curve-ff-r2: gate on path bend / yaw\n"),
])
