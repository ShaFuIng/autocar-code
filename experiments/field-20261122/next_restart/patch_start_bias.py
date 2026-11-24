"""start-bias-r1 hooks for live_pid.py. Base = landing4/merged_car/live_pid.py e0a89bdb (car 128e9ed5 + land4-r1).
usage: python patch_start_bias.py <in live_pid.py> <out live_pid.py>
Three insertions, nothing else changes; every anchor must match exactly once."""
import hashlib, sys
from pathlib import Path

s = Path(sys.argv[1]).read_bytes().decode('utf-8')
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('e0a89bdb'), 'unexpected base'
NL = chr(13) + chr(10) if chr(13) + chr(10) in s else chr(10)


def ins_after_line(anchor, text):
    global s
    assert s.count(anchor) == 1, (s.count(anchor), anchor[:70]); i = s.index(anchor); j = s.index(chr(10), i)
    s = s[:j + 1] + text.replace(chr(10), NL) + s[j + 1:]


def ins_before(anchor, text):
    global s
    assert s.count(anchor) == 1, (s.count(anchor), anchor[:70]); i = s.index(anchor)
    s = s[:i] + text.replace(chr(10), NL) + s[i:]


ins_after_line("            self.land4=self._l4_new()",
               "            self.sbias=self._sb_new()   # start-bias-r1: start_bias.json once per run (absent / enabled not true -> None, layer not run)\n")
ins_before("    def _l4_new(self):",
           "    def _sb_new(self):   # start-bias-r1 (field-20261122/next_restart)\n"
           "        try:\n"
           "            import start_bias\n"
           "            return start_bias.load_config()\n"
           "        except Exception:return None\n"
           "    def _L_start_bias(self,F):   # last valid-frame layer: a short steering offset after the start hold (start_bias.py)\n"
           "        p=F.p\n"
           "        try:\n"
           "            import start_bias\n"
           "            if not getattr(self,'start_centred',True) or p.get('reason')!='PROPOSAL_ONLY':return\n"
           "            el=time.monotonic()-getattr(self,'start_t',time.monotonic())-float(getattr(self,'start_hold_s',0.) or 0.);rp=self.run['parameters']\n"
           "            st,inf,done=start_bias.apply(self.sbias,el,p['steer_us'],float(rp['min_us']),float(rp['max_us']))\n"
           "            if inf is not None:p['start_bias']=inf;p['steer_us']=st\n"
           "            if done:self.sbias=None\n"
           "        except Exception as e:p['start_bias']=dict(revision='start-bias-r1',reason='ERROR',error=type(e).__name__);self.sbias=None\n")
A = "    _SL('i_ramp',lambda s,F:getattr(s,'iramp',None) is not None and F.p.get('reason')!='WARMUP',LivePID._L_i_ramp))"
assert s.count(A) == 1, s.count(A)
s = s.replace(A, A[:-1] + "," + NL + "    _SL('start_bias',lambda s,F:getattr(s,'sbias',None) is not None,LivePID._L_start_bias))")
Path(sys.argv[2]).write_bytes(s.encode('utf-8')); print('written', sys.argv[2], hashlib.sha256(s.encode('utf-8')).hexdigest()[:8])
