"""x-band-r1 hook (c3c4-20261122): live_pid.py af2d6b68 -> the arc2 c3 layer's lap_pos dict gets the IMU yaw rate (one read) ONLY while the c3 block has
an enabled 'xband' (lap_ff.LapFF4.xb_on); otherwise the line does nothing (no read, same read order). One inserted line, nothing removed.
usage: python patch_live_xb.py <live_pid.py af2d6b68> <out live_pid.py>     python patch_live_xb.py --check <live_pid.py af2d6b68> <patched>"""
import hashlib, sys
from pathlib import Path

args = [a for a in sys.argv[1:] if a != '--check']; CHECK = '--check' in sys.argv
s = Path(args[0]).read_bytes().decode('utf-8')
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('af2d6b68'), 'unexpected base'
NL = chr(13) + chr(10) if chr(13) + chr(10) in s else chr(10)
old = "                lp3=dict(lp,pitch=ctx.pitch)\n".replace(chr(10), NL)
new = ("                lp3=dict(lp,pitch=ctx.pitch)\n"
       "                if getattr(A['c3'],'xb_on',False):self._lff4_gz(lp3)   # x-band-r1 (c3c4-20261122): yaw rate for the distance band, only while arc2.json c3.xband is enabled\n").replace(chr(10), NL)
assert s.count(old) == 1, s.count(old)
s = s.replace(old, new)
h = hashlib.sha256(s.encode('utf-8')).hexdigest()[:8]
if CHECK:
    t = Path(args[1]).read_bytes().decode('utf-8'); print('check', 'OK' if t == s else 'DIFFERENT', h); sys.exit(0 if t == s else 1)
Path(args[1]).write_bytes(s.encode('utf-8')); print('written', args[1], h)
