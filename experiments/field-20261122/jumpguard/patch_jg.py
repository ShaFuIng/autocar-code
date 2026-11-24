"""jump-guard-minprev-r1 (2026-11-22): the error-jump guard holds only after the high error has LASTED.
Twice on 2026-11-22 a short spike of the image error (46c9 downhill mouth: one frame +1.00; 69d9 curve 3: two frames +0.80 / +0.98
while curve_ff raised the command to centre +370) was followed by the true error (-0.17 / -0.39). error-jump-guard-r1 read the return
to the true value as the 'jump' and held the spike's right command for JG_SHORT_S 0.3 s -> the car left the lane both times.
The guard was written (2026-11-05) for an error that SITS at the right edge and then jumps across the road for one frame.
New: web/jump_guard.json {"min_prev_s": 0.2} (read at each run start, when the guard state is built). The guard holds only when the
error has been >= JG_PREV continuously for at least min_prev_s before the drop; a shorter high episode is accepted at once
(jump_guard info: accepted true, short_high_s). File absent / min_prev_s not in (0, 2] = 0 = today's behaviour, frame for frame.
usage: python patch_jg.py <live_pid.py 0c158624> <out live_pid.py>"""
import hashlib, sys
from pathlib import Path

s = Path(sys.argv[1]).read_bytes().decode('utf-8')
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('0c158624'), 'unexpected base'
assert chr(13) not in s


def rep(old, new):
    global s
    assert s.count(old) == 1, (s.count(old), old[:70]); s = s.replace(old, new)


rep("            if jg is None:jg=self.jg=dict(prev=None,prev_t=None,since=None,sat_t=None,n=0)\n",
    "            if jg is None:jg=self.jg=dict(prev=None,prev_t=None,since=None,sat_t=None,n=0,mp=self._jg_min_prev(),hi_t=None)   # jump-guard-minprev-r1: mp 0 = as before\n")
rep("                    ls=(self.hold.last_sent(now) or {}).get('steer_us') if getattr(self,'hold',None) is not None else None\n"
    "                    if jt-jg['since']<=win and ls is not None and float(ls)>float(self.run['parameters']['center_us']):   # right commands only\n",
    "                    ls=(self.hold.last_sent(now) or {}).get('steer_us') if getattr(self,'hold',None) is not None else None\n"
    "                    jsh=None   # jump-guard-minprev-r1: seconds the error had been high when it dropped, when that is shorter than mp (None = long enough / mp 0)\n"
    "                    if jg['mp']>0.:\n"
    "                        jhd=0. if jg['hi_t'] is None else jg['prev_t']-jg['hi_t']\n"
    "                        if jhd<jg['mp']:jsh=jhd\n"
    "                    if jsh is None and jt-jg['since']<=win and ls is not None and float(ls)>float(self.run['parameters']['center_us']):   # right commands only\n")
rep("                        proposal['jump_guard']=dict(revision=JG_REVISION,accepted=True,prev_err=round(pe,3),err=round(je,3),frames=jg['n'])\n"
    "                        jg['prev']=je;jg['prev_t']=jt;jg['since']=None;jg['sat_t']=None\n",
    "                        proposal['jump_guard']=dict(revision=JG_REVISION,accepted=True,prev_err=round(pe,3),err=round(je,3),frames=jg['n'])\n"
    "                        if jsh is not None:proposal['jump_guard']['short_high_s']=round(jsh,3)\n"
    "                        jg['prev']=je;jg['prev_t']=jt;jg['since']=None;jg['sat_t']=None;jg['hi_t']=None\n")
rep("                    jg['since']=None;jg['prev']=je;jg['prev_t']=jt\n"
    "                    jg['sat_t']=(jg['sat_t'] if jg['sat_t'] is not None else jt) if je>=.95 else None\n",
    "                    jg['since']=None;jg['prev']=je;jg['prev_t']=jt\n"
    "                    jg['sat_t']=(jg['sat_t'] if jg['sat_t'] is not None else jt) if je>=.95 else None\n"
    "                    jg['hi_t']=(jg['hi_t'] if jg['hi_t'] is not None else jt) if je>=JG_PREV else None   # jump-guard-minprev-r1: start of the present high-error episode\n")
rep("    def _L_dh_last_err(self,F):\n",
    "    def _jg_min_prev(self):   # jump-guard-minprev-r1: web/jump_guard.json min_prev_s, read when the guard state is built (each run start); anything else = 0\n"
    "        try:\n"
    "            v=json.loads((Path(__file__).parent/'jump_guard.json').read_text(encoding='utf8')).get('min_prev_s')\n"
    "            return float(v) if isinstance(v,(int,float)) and not isinstance(v,bool) and 0.<v<=2. else 0.\n"
    "        except Exception:return 0.\n"
    "    def _L_dh_last_err(self,F):\n")
Path(sys.argv[2]).write_bytes(s.encode('utf-8')); print('written', sys.argv[2], hashlib.sha256(s.encode('utf-8')).hexdigest()[:8])
