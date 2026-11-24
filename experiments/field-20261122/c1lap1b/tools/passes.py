"""c1lap1b: per curve-1 pass reconstruction. A pass = contiguous frames with lap_ff.curve == 'c1' (gaps <= 0.5 s allowed).
Heading hd = integrated proposal.inputs.gz (left +). plat_hd = hd + band.turned_deg (median over the pass; its spread checks the integration).
Also non-band passes (lap_pos seg curve1 with no lap_ff c1) are reconstructed with tb_hd = ref - hd, ref = mean hd over the same reference window."""
import statistics as st, math, json, sys
from lib import load, num


def med(a):
    a = [x for x in a if num(x)]; return st.median(a) if a else None


def mean(a):
    a = [x for x in a if num(x)]; return sum(a) / len(a) if a else None


def mn(a):
    a = [x for x in a if num(x)]; return min(a) if a else None


def mx(a):
    a = [x for x in a if num(x)]; return max(a) if a else None


def curve_passes(F):
    """curve-1 passes by lap_pos: each lap's frames from 'start_straight'/'main_straight' end to C1e+3 m. Returns list of dicts with
    index lists: pre (reference straight), cur (curve: band frames or seg curve1), ex (C1e 0..3 m)."""
    P = []
    n = len(F); i = 0
    # locate first frame of each curve1 segment occurrence
    starts = []
    prev = None
    for j, f in enumerate(F):
        s = f['seg']
        if s == 'curve1' and prev != 'curve1': starts.append(j)
        if s is not None: prev = s
    for j in starts:
        lapno = F[j]['lap']
        first = not any(F[q]['seg'] == 'curve1' for q in range(0, j))
        # band frames of this pass: lap_ff c1 frames within +-6 s of j
        tj = F[j]['t']
        band = [q for q in range(max(0, j - 200), min(n, j + 300)) if F[q]['lf_curve'] == 'c1' and num(F[q]['t']) and abs(F[q]['t'] - tj) < 6]
        ex = [q for q in range(j, min(n, j + 400)) if F[q]['anchor'] == 'C1e' and num(F[q]['sa']) and F[q]['sa'] <= 3.0]
        if first:
            ref = [q for q in range(0, j) if num(F[q]['odo']) and 0.4 <= F[q]['odo'] <= 1.8]
            pre = [q for q in range(0, j) if num(F[q]['odo'])]
        else:
            ref = [q for q in range(max(0, j - 500), j) if F[q]['anchor'] == 'C4e' and num(F[q]['sa']) and 3.0 <= F[q]['sa'] <= 8.0]
            pre = [q for q in range(max(0, j - 500), j) if F[q]['anchor'] == 'C4e' and num(F[q]['sa'])]
        cur = band if band else [q for q in range(j, min(n, j + 300)) if F[q]['seg'] == 'curve1']
        P.append(dict(j=j, lap=lapno, first=first, band=band, ex=ex, ref=ref, pre=pre, cur=cur))
    return P


def analyse(run, F=None):
    F = F or load(run); out = []
    for k, p in enumerate(curve_passes(F)):
        band = [F[q] for q in p['band'] if num(F[q]['tb'])]
        refhd = mean([F[q]['hd'] for q in p['ref']])
        if band:
            consts = [f['hd'] + f['tb'] for f in band]
            plat = med(consts); spread = max(consts) - min(consts)
        else:
            plat = refhd; spread = None
        r = dict(run=run, k=k, lap=F[p['j']]['lap'], first=p['first'], banded=bool(band), plat_hd=plat, ref_hd=refhd, plat_minus_ref=None if (plat is None or refhd is None) else plat - refhd,
                 plat_spread=spread)
        if band:
            ft = F[p['band'][0]]
            r.update(kind=ft['lf_kind'], odo_trig=ft['odo'], s_m_trig=ft['s_m'], anchor_trig=ft['anchor'], sa_trig=ft['sa'], tb_trig=band[0]['tb'])
        cur = [F[q] for q in p['cur']]
        for f in cur: f['_tb'] = f['tb'] if num(f['tb']) else (None if plat is None else plat - f['hd'])
        # entry speed: frames with _tb in [-5, 15]
        ent = [f for f in cur if num(f['_tb']) and -5 <= f['_tb'] <= 15]
        r['v_entry'] = med([f['v'] for f in ent]); r['y_entry'] = med([f['y'] for f in ent]); r['err_entry'] = med([f['err'] for f in ent])
        r['psi_entry'] = med([f['psi'] for f in ent])
        bins = {}
        for f in cur:
            if not num(f['_tb']): continue
            b = int(max(0, f['_tb']) // 20) * 20
            bins.setdefault(b, []).append(f)
        r['bins'] = {b: dict(n=len(v), y=med([f['y'] for f in v]), ci=mn([f['clr_in'] for f in v]), co=mn([f['clr_out'] for f in v]),
                             err=med([f['err'] for f in v]), ec=med([f['ec'] for f in v]), eus=med([f['eus'] for f in v]), prof=med([f['prof'] for f in v]),
                             gzus=med([f['gzus'] for f in v]), pre=med([f['pre'] for f in v]), sent=med([f['sent'] for f in v]), gz=med([f['gz'] for f in v]),
                             v=med([f['v'] for f in v]), psi=med([f['psi'] for f in v]))
                     for b, v in sorted(bins.items())}
        r['clr_in_min'] = mn([f['clr_in'] for f in cur]); r['clr_out_min'] = mn([f['clr_out'] for f in cur])
        ym = [(f['y'], f['_tb']) for f in cur if num(f['y']) and num(f['_tb'])]
        r['y_min'] = min(ym)[0] if ym else None; r['tb_at_ymin'] = min(ym)[1] if ym else None
        ex = [F[q] for q in p['ex']]
        if ex:
            r['exit_err_c1e'] = ex[0]['err']; r['exit_err_max'] = mx([f['err'] for f in ex])
            r['exit_y06'] = med([f['y'] for f in ex if f['sa'] <= 0.6])
            r['exit_head'] = None if plat is None else med([plat - f['hd'] for f in ex if 1.2 <= f['sa'] <= 2.2])
            r['exit_psi'] = med([f['psi'] for f in ex if 1.2 <= f['sa'] <= 2.2])
            r['exit_y12'] = med([f['y'] for f in ex if 1.2 <= f['sa'] <= 2.2])
            r['exit_y_max'] = mx([f['y'] for f in ex]); r['exit_y_min'] = mn([f['y'] for f in ex])
        ref = [F[q] for q in p['ref']]
        r['ref_psi'] = med([f['psi'] for f in ref]); r['ref_err'] = med([f['err'] for f in ref]); r['ref_y'] = med([f['y'] for f in ref]); r['ref_n'] = len(ref)
        pre = [F[q] for q in p['pre']]
        q0 = p['band'][0] if p['band'] else p['j']
        pre = [F[q] for q in p['pre'] if q < q0]
        if p['first']:
            r['hd_max_pre'] = mx([f['hd'] for f in pre]); r['hd_min_pre'] = mn([f['hd'] for f in pre])
            late = [f for f in pre if f['odo'] >= 1.8]
        else:
            late = [f for f in pre if f['sa'] >= 8.0]
        r['trig_hd_rel'] = None if plat is None else plat - F[q0]['hd']
        r['late_psi'] = med([f['psi'] for f in late]); r['late_hd_rel'] = None if plat is None else med([plat - f['hd'] for f in late])
        r['late_y'] = med([f['y'] for f in late]); r['late_err'] = med([f['err'] for f in late])
        out.append(r)
    return out, F


def fmt(r):
    return json.dumps({k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items() if k != 'bins'}, ensure_ascii=False)


if __name__ == '__main__':
    for run in sys.argv[1:]:
        R, _ = analyse(run)
        for r in R: print(fmt(r))
