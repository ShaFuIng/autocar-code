"""patch_land4.py <snapshot live_pid.py> <out live_pid.py>
land4-r1 hooks for live_pid.py (base: orin-web-snapshot-20261121c live_pid.py 800879c9...). Four insertions, nothing else changes:
  1. start(): self.land4 = self._l4_new()                      (land4.json absent / enabled false / no stage -> None)
  2. methods _l4_new / _l4_step / _L_land4 / _l4_lost          (before _ctrim_new)
  3. lost-frame path: land4 hook after the arc2 approach hook   (only when self.land4 is not None)
  4. VALID_LAYERS: 'land4' after bottom_guide, before upper_single_final (only when self.land4 is not None)
Every anchor must match exactly once, otherwise the script stops without writing."""
import sys, hashlib
from pathlib import Path

src = Path(sys.argv[1]).read_text(encoding='utf8'); out = Path(sys.argv[2])


def once(s, a):
    if s.count(a) != 1: raise SystemExit('anchor not unique (%d): %r' % (s.count(a), a[:80]))


A1 = "            self.ctrim=self._ctrim_new()"
once(src, A1)
i = src.index(A1); j = src.index('\n', i)
src = src[:j + 1] + "            self.land4=self._l4_new()   # land4-r1 (field-20261122/landing4): reads land4.json once per run (absent / enabled false / no stage -> None, nothing runs)\n" + src[j + 1:]

A2 = "    def _ctrim_new(self):"
once(src, A2)
M = '''    # ---- land4-r1 (field-20261122/landing4, offline candidate): slope + landing steered on the absolute heading of the upper straight (land4.py / land4.json).
    # self.land4 None (json absent / disabled) -> none of these is called: no IMU / MCU / clock read is added.
    def _l4_new(self):
        try:
            import land4
            return land4.Land4.from_file()
        except Exception:return None
    def _l4_step(self,F,p,valid):   # one frame (valid / WARMUP / lost): reads the IMU and the MCU status only on the upper straight, the slope and the landing
        L=self.land4;ctx=F.ctx;rp=self.run['parameters']
        up=bool(valid and ctx.hh_active(p));yaw=gz=enc=ms=None
        if L.wants_imu(up,ctx.down_tail):
            if getattr(self,'heading',None) is not None:
                tn=time.monotonic()
                try:yaw=float(self.heading._yaw(tn))
                except Exception:yaw=None
                try:gz=float(self.heading._gz(tn))
                except Exception:gz=None
            try:
                v_=(self.manual.state() or {}).get('vehicle') or {};b_=v_.get('enc_b') or {}
                if 'forward' in b_ and 'reverse' in b_:enc=int(b_['forward'])-int(b_['reverse']);ms=v_.get('ms')
            except Exception:enc=ms=None
        ls=None
        try:ls=(self.hold.last_sent(F.now) or {}).get('steer_us') if getattr(self,'hold',None) is not None else None
        except Exception:ls=None
        return L.step(F.now,bool(valid and p.get('reason')=='PROPOSAL_ONLY'),p.get('image_error'),yaw,gz,enc,ms,getattr(self,'_cm_speed',None),up,bool(ctx.down_tail),ctx.down_level,
                      p['steer_us'],float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),ls)
    def _L_land4(self,F):   # valid / WARMUP frames: last steering writer before the upper-single override (the chain value is its input)
        p=F.p
        try:
            st,inf=self._l4_step(F,p,True)
            if inf is not None:
                p['land4']=inf
                if st!=float(p['steer_us']):p['steer_us']=st
        except Exception as e:p['land4']=dict(revision='land4-r1',reason='ERROR',error=type(e).__name__)
    def _l4_lost(self,F,lp):   # lost frames: same law without the camera term (after every other lost-frame hook)
        try:
            st,inf=self._l4_step(F,lp,False)
            if inf is not None:
                b=lp['steer_us'];lp=dict(lp,land4=inf)
                if st!=float(b):lp['steer_us']=st;F.owner=segment_ctx.owner_after(F.owner,'land4_lost',b,lp)
        except Exception as e:lp=dict(lp,land4=dict(revision='land4-r1',reason='ERROR',error=type(e).__name__))
        return lp
'''
i = src.index(A2); src = src[:i] + M + src[i:]

A3 = "                            if getattr(self,'arc2',None) is not None and getattr(self.arc2.get('land'),'lost_on',False):lp=self._a2_lost(F,lp)"
once(src, A3)
i = src.index(A3); j = src.index('\n', i)
src = src[:j + 1] + "                            if getattr(self,'land4',None) is not None:lp=self._l4_lost(F,lp)   # land4-r1: slope / landing heading law on lost frames (land4.json absent -> not called)\n" + src[j + 1:]

A4 = "    _SL('bottom_guide',lambda s,F:getattr(s,'bguide',None) is not None and s.bguide.on,LivePID._L_bottom_guide),"
once(src, A4)
i = src.index(A4); j = src.index('\n', i)
src = src[:j + 1] + "    _SL('land4',lambda s,F:getattr(s,'land4',None) is not None,LivePID._L_land4),   # land4-r1: absent / disabled -> not run\n" + src[j + 1:]

out.write_text(src, encoding='utf8', newline='\n')
print('written', out, hashlib.sha256(out.read_bytes()).hexdigest()[:8])
