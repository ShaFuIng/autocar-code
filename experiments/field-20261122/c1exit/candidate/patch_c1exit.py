"""c1exit-r1 patch for lap_ff.py (curve 1 / 2 r1 band only: class Trigger and class LapFF; LapFF4 / c3 xband / the curve-4 band are NOT touched).

  python patch_c1exit.py <lap_ff.py in> <lap_ff.py out>        apply (every anchor must match exactly once, else nothing is written)
  python patch_c1exit.py --check <in> <expected out>            re-apply and compare with an existing output

Base used for the delivery: the car's lap_ff.py adcde919 (c3c4 candidate). The anchors are single lines of the r1 band code, so the patch can be
stacked with another patch of the same file as long as that patch leaves these lines alone (it then matches the same anchors).

New keys (every one absent = the code path of today, replay proof in proof/C1_key_absent.txt):
  curves.<cN>.band.own_lo_us   (number <= 0)  own mode only: the band target / top may go this far LEFT of the centre (today: clamped at the centre, so a
                               band that owns the output cannot steer left when the car over-rotated at the exit). Also used by the band's lost-frame hold.
  curves.<cN>.band.v_ref       (m/s)          the band profile and its gyro term use this fixed speed for the gain table instead of the wheel speed of the
                               frame (5 Hz encoder: the logged band speed jumps 0.89..1.39 m/s at a steady ~0.95 m/s = +-40 us of profile noise).
  curves.<cN>.start            (dict)         lap from the start (lap_pos lap 0, no landmark yet): the curve is armed on the odometer since the run start,
                               {"gap_m": metres from the start to the curve start, "win_m": [a, b], "pos_m": null, "ref_win": [a, b]} ; the band's straight
                               reference heading is the mean IMU heading over ref_win metres from the start. One trigger per run.
"""
import sys, hashlib
from pathlib import Path

EDITS = [
    # ---- start anchor (lap 0): Trigger.armed
    ("""            if not c.get('enabled') or lp.get('anchor') != c['anchor']: continue
            pid = (name, lp.get('anchor'), None if lp.get('anchor_d') is None else round(float(lp['anchor_d']), 3), lp.get('lap'))""",
     """            if lp.get('anchor') == 'START' and isinstance(c.get('start'), dict): c = dict(c, anchor='START', gap_m=c['start'].get('gap_m', c['gap_m']), win_m=c['start'].get('win_m', c['win_m']), pos_m=c['start'].get('pos_m'))   # c1exit-r1: lap from the start
            if not c.get('enabled') or lp.get('anchor') != c['anchor']: continue
            pid = (name, lp.get('anchor'), None if lp.get('anchor_d') is None else round(float(lp['anchor_d']), 3), lp.get('lap'))"""),
    # ---- start anchor: LapFF.step entry (lp replaced only when a curve has a 'start' dict)
    ("""        c = self.c; st = float(steer)
        bd_on = getattr(self, 'band_on', False)   # c2-band-r1
        if bd_on: self._band_acc(lp); self.lost_t0 = None""",
     """        c = self.c; st = float(steer)
        if getattr(self, 'a0', False): lp = self._lp0(lp)   # c1exit-r1 (no curve with a 'start' dict -> not called)
        bd_on = getattr(self, 'band_on', False)   # c2-band-r1
        if bd_on: self._band_acc(lp); self.lost_t0 = None"""),
    # ---- own_lo: target / top clamp
    ("""        else: tgt = max(float(centre), min(bi['cap_us'], float(hi), bi['tgt']))""",
     """        else: tgt = max(float(centre) + bi.get('own_lo', 0.), min(bi['cap_us'], float(hi), bi['tgt']))   # c1exit-r1: own_lo 0. unless band.own_lo_us in own mode"""),
    ("""            bi['upper'] = max(float(centre), bi['own_up'] if bi['upper'] is None else min(bi['upper'], bi['own_up']))""",
     """            bi['upper'] = max(float(centre) + bi.get('own_lo', 0.), bi['own_up'] if bi['upper'] is None else min(bi['upper'], bi['own_up']))   # c1exit-r1"""),
    # ---- band init: flag for the start anchor
    ("""        self.band_on = bool(self.band); self.pref = {}; self.guard_n = 0; self.lost_t0 = None""",
     """        self.band_on = bool(self.band); self.pref = {}; self.guard_n = 0; self.lost_t0 = None
        self.a0 = any(isinstance(cc, dict) and cc.get('enabled') and isinstance(cc.get('start'), dict) for cc in (self.c.get('curves') or {}).values())   # c1exit-r1

    def _lp0(self, lp):   # c1exit-r1: lap from the start (lap_pos lap 0, no landmark yet) -> pseudo anchor 'START' at odometer 0
        if not lp or lp.get('anchor') is not None or lp.get('lap') != 0 or lp.get('lost') or not _num(lp.get('odo_m')): return lp
        return dict(lp, anchor='START', anchor_d=0., since_anchor_m=float(lp['odo_m']))"""),
    # ---- start anchor: straight reference heading
    ("""            if lp.get('anchor') != self.c['curves'][k]['anchor']: continue
            key = (lp.get('lap'), None if lp.get('anchor_d') is None else round(float(lp['anchor_d']), 3))""",
     """            s0 = self.c['curves'][k].get('start') if lp.get('anchor') == 'START' else None   # c1exit-r1
            if lp.get('anchor') != ('START' if isinstance(s0, dict) else self.c['curves'][k]['anchor']): continue
            if isinstance(s0, dict) and s0.get('ref_win'): b = dict(b, ref_win=s0['ref_win'])
            key = (lp.get('lap'), None if lp.get('anchor_d') is None else round(float(lp['anchor_d']), 3))"""),
    # ---- v_ref + own_lo in the band target
    ("""        b = self.band[A['curve']]; kap, kmax, prof = self._band_prof(b, tb, v, g)""",
     """        b = self.band[A['curve']]
        if _num(b.get('v_ref')) and float(b['v_ref']) > 0.2: v = float(b['v_ref']); g = gain(self.c['gain_table'], v)   # c1exit-r1: fixed speed for the profile / gyro term
        kap, kmax, prof = self._band_prof(b, tb, v, g)"""),
    ("""        return dict(tgt=prof + et + gt, upper=up, cap_us=float(b['cap_us']), tb=tb, kap=kap, prof=prof, et=et, gt=gt, ec=ec, own_up=own)""",
     """        r_ = dict(tgt=prof + et + gt, upper=up, cap_us=float(b['cap_us']), tb=tb, kap=kap, prof=prof, et=et, gt=gt, ec=ec, own_up=own)
        if own is not None and _num(b.get('own_lo_us')) and float(b['own_lo_us']) < 0.: r_['own_lo'] = float(b['own_lo_us'])   # c1exit-r1
        return r_"""),
    # ---- lost-frame hold: same start anchor / v_ref / own_lo
    ("""        if lp is None or lp.get('lost') or now is None: return None
        b = self.band[A['curve']]
        if self.lost_t0 is None: self.lost_t0 = float(now)""",
     """        if lp is None or lp.get('lost') or now is None: return None
        b = self.band[A['curve']]
        if _num(b.get('v_ref')) and float(b['v_ref']) > 0.2: v = float(b['v_ref'])   # c1exit-r1
        if self.lost_t0 is None: self.lost_t0 = float(now)"""),
    ("""        tgt = max(float(centre), min(float(b['cap_us']), float(hi), prof + gt))
        a = 1.
        dl_, rm_ = self._dr(c['curves'][A['curve']])   # c1-band-r1""",
     """        tgt = max(float(centre) + (float(b['own_lo_us']) if (b.get('own_after_deg') is not None and _num(b.get('own_lo_us')) and float(b['own_lo_us']) < 0. and tb >= float(b['own_after_deg'])) else 0.), min(float(b['cap_us']), float(hi), prof + gt))   # c1exit-r1
        a = 1.
        dl_, rm_ = self._dr(c['curves'][A['curve']])   # c1-band-r1"""),
]


def apply(src):
    nl = '\r\n' if '\r\n' in src else '\n'   # the car file has CRLF line ends: keep whatever the input uses
    for k, (old, new) in enumerate(EDITS):
        old = old.replace('\n', nl); new = new.replace('\n', nl)
        n = src.count(old)
        if n != 1: raise SystemExit('edit %d: anchor matches %d times (need exactly 1) - nothing written' % (k, n))
        src = src.replace(old, new)
    return src


def sha(b): return hashlib.sha256(b).hexdigest()


if __name__ == '__main__':
    a = sys.argv[1:]
    if a and a[0] == '--check':
        src = Path(a[1]).read_bytes(); out = apply(src.decode('utf8')).encode('utf8'); exp = Path(a[2]).read_bytes()
        print('in %s  out %s  expected %s  %s' % (sha(src)[:8], sha(out)[:8], sha(exp)[:8], 'OK' if out == exp else 'DIFFERENT')); sys.exit(0 if out == exp else 1)
    src = Path(a[0]).read_bytes(); out = apply(src.decode('utf8')).encode('utf8'); Path(a[1]).write_bytes(out)
    print('in %s -> out %s (%d edits)' % (sha(src)[:8], sha(out)[:8], len(EDITS)))
