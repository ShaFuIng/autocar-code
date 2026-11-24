"""x-band-r2 hook (c3refit-20261122): live_pid.py 3b74ddc4 -> the arc2 c3 layer's lap_pos dict also carries arc_lat's lateral position of the PREVIOUS frame
(clock, y_cm or None) while an x-band is configured; arc_lat (a later layer of the same frame) stores it after its own step. Two changed lines, nothing removed;
no extra read. Without an enabled c3.xband the first line does nothing; the second only stores a tuple on the object (nothing reads it).
usage: python patch_live_r2.py <live_pid.py 3b74ddc4> <out live_pid.py>     python patch_live_r2.py --check <live_pid.py 3b74ddc4> <patched>"""
import hashlib, sys
from pathlib import Path

args = [a for a in sys.argv[1:] if a != '--check']; CHECK = '--check' in sys.argv
s = Path(args[0]).read_bytes().decode('utf-8')
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('3b74ddc4'), 'unexpected base'
NL = chr(13) + chr(10) if chr(13) + chr(10) in s else chr(10)


def rep(old, new):
    global s
    old, new = old.replace(chr(10), NL), new.replace(chr(10), NL)
    assert s.count(old) == 1, (s.count(old), old[:80]); s = s.replace(old, new)


rep("                if getattr(A['c3'],'xb_on',False):self._lff4_gz(lp3)   # x-band-r1 (c3c4-20261122): yaw rate for the distance band, only while arc2.json c3.xband is enabled\n",
    "                if getattr(A['c3'],'xb_on',False):self._lff4_gz(lp3);lp3['al']=getattr(self,'_al_prev',None)   # x-band-r1 (c3c4-20261122): yaw rate for the distance band, only while arc2.json c3.xband is enabled; x-band-r2 (c3refit): + arc_lat y of the previous frame\n")
rep("            if inf is not None:\n"
    "                proposal['arc_lat']=inf\n",
    "            if inf is not None:\n"
    "                self._al_prev=(F.now,inf.get('y_cm') if inf.get('ok') else None)   # x-band-r2 (c3refit): this frame's lateral position for the next frame's c3 x-band (stored only)\n"
    "                proposal['arc_lat']=inf\n")
h = hashlib.sha256(s.encode('utf-8')).hexdigest()[:8]
if CHECK:
    t = Path(args[1]).read_bytes().decode('utf-8'); print('check', 'OK' if t == s else 'DIFFERENT', h); sys.exit(0 if t == s else 1)
Path(args[1]).write_bytes(s.encode('utf-8')); print('written', args[1], h)
