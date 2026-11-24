"""c1lap1b: table of all curve-1 passes (band lap 1 / band lap 2+ / non-band) + per-20-deg bins + correlations. Writes proof/*.txt."""
import json, sys, math, statistics as st
from pathlib import Path
from passes import analyse, fmt
from lib import num
OUT = Path(__file__).resolve().parents[1] / 'proof'
LAP1_BAND = ['3f22', 'fd22', 'b23a', '2b72', 'b619', '1377', 'e69d', '54cb', '1cd7']
LATER_BAND = ['9af6', 'e69d', '54cb']
NONBAND = ['9af6', 'c6f3', '6f61', 'e0cd', '2728', '7658']


def corr(x, y):
    p = [(a, b) for a, b in zip(x, y) if num(a) and num(b)]
    if len(p) < 3: return None, None, len(p)
    mx = sum(a for a, _ in p) / len(p); my = sum(b for _, b in p) / len(p)
    sxx = sum((a - mx) ** 2 for a, _ in p); syy = sum((b - my) ** 2 for _, b in p); sxy = sum((a - mx) * (b - my) for a, b in p)
    if sxx <= 0 or syy <= 0: return None, None, len(p)
    return sxy / math.sqrt(sxx * syy), sxy / sxx, len(p)


def r2(v, n=2):
    return None if not num(v) else round(v, n)


def main():
    runs = sorted(set(LAP1_BAND + LATER_BAND + NONBAND))
    R = {}
    for run in runs:
        R[run], _ = analyse(run)
    groups = dict(lap1_band=[], later_band=[], lap1_nonband=[], later_nonband=[])
    for run in runs:
        for r in R[run]:
            if r['first']:
                groups['lap1_band' if r['banded'] else 'lap1_nonband'].append(r)
            else:
                groups['later_band' if r['banded'] else 'later_nonband'].append(r)
    cols = ['run', 'k', 'lap', 'kind', 'odo_trig', 's_m_trig', 'sa_trig', 'tb_trig', 'trig_hd_rel', 'late_hd_rel', 'hd_max_pre', 'hd_min_pre', 'plat_hd', 'plat_minus_ref', 'v_entry', 'late_y', 'y_entry',
            'late_err', 'err_entry', 'late_psi', 'psi_entry', 'clr_in_min', 'clr_out_min', 'y_min', 'tb_at_ymin', 'exit_head', 'exit_err_c1e', 'exit_err_max', 'exit_y06', 'exit_y12', 'exit_y_max', 'exit_y_min', 'exit_psi']
    L = []
    for gname, G in groups.items():
        L.append(f'== {gname}  n={len(G)}')
        L.append('\t'.join(cols))
        for r in G: L.append('\t'.join(str(r2(r.get(c)) if isinstance(r.get(c), float) else r.get(c)) for c in cols))
        for c in ['clr_in_min', 'clr_out_min', 'y_min', 'y_entry', 'trig_hd_rel', 'late_hd_rel', 'tb_trig', 'v_entry', 'exit_head', 'exit_err_max', 'exit_y06', 'exit_y12']:
            v = [r.get(c) for r in G if num(r.get(c))]
            if v: L.append(f'   {c}: median {st.median(v):.2f}  min {min(v):.2f}  max {max(v):.2f}  n {len(v)}')
    (OUT / 'A1_passes_table.txt').write_text('\n'.join(L), encoding='utf8')
    # bins
    B = []
    for gname, G in groups.items():
        B.append(f'== {gname}: per 20-deg bin of the band turned angle (non-band: IMU turn since the same reference window); medians of pass medians; ci/co = min over passes of the bin minimum')
        B.append('bin\tn\ty\tci\tco\terr\te_c\te_us\tprof\tgz_us\tpre\tsent\tgz\tv\tpsi')
        for b in range(0, 180, 20):
            rows = [r['bins'][b] for r in G if b in r['bins']]
            if not rows: continue
            def m(k):
                v = [x[k] for x in rows if num(x.get(k))]; return r2(st.median(v)) if v else None
            def mi(k):
                v = [x[k] for x in rows if num(x.get(k))]; return r2(min(v)) if v else None
            B.append('\t'.join(str(x) for x in [b, len(rows), m('y'), mi('ci'), mi('co'), m('err'), m('ec'), m('eus'), m('prof'), m('gzus'), m('pre'), m('sent'), m('gz'), m('v'), m('psi')]))
        # per pass y by bin
        B.append('  per pass y (cm) by bin 0..160:')
        for r in G:
            B.append('  %s/%d\t' % (r['run'], r['k']) + '\t'.join(str(r2(r['bins'].get(b, {}).get('y'), 1)) for b in range(0, 180, 20)) +
                     '\t| e_us ' + ' '.join(str(r2(r['bins'].get(b, {}).get('eus'), 0)) for b in range(0, 180, 20)) +
                     '\t| pre ' + ' '.join(str(r2(r['bins'].get(b, {}).get('pre'), 0)) for b in range(0, 180, 20)))
    (OUT / 'A2_bins.txt').write_text('\n'.join(B), encoding='utf8')
    # correlations within band passes
    C = []
    for gname in ('lap1_band', 'later_band', 'all_band'):
        G = groups['lap1_band'] + groups['later_band'] if gname == 'all_band' else groups[gname]
        C.append(f'== {gname} n={len(G)}: Pearson r and slope (target per unit of x)')
        for tgt in ('clr_in_min', 'y_min'):
            for x in ('y_entry', 'late_y', 'trig_hd_rel', 'late_hd_rel', 'tb_trig', 'v_entry', 'odo_trig', 'err_entry', 'psi_entry', 'late_psi', 'plat_minus_ref', 'exit_head'):
                rr, sl, n = corr([r.get(x) for r in G], [r.get(tgt) for r in G])
                C.append(f'  {tgt:11s} vs {x:14s} r={r2(rr)} slope={r2(sl)} n={n}')
        rr, sl, n = corr([1 if r['first'] else 0 for r in G], [r.get('clr_in_min') for r in G])
        if gname == 'all_band': C.append(f'  clr_in_min vs lap1-flag r={r2(rr)} slope(lap1 - later)={r2(sl)} n={n}')
    (OUT / 'A3_correlations.txt').write_text('\n'.join(C), encoding='utf8')
    print('\n'.join(L)); print('\n'.join(B)); print('\n'.join(C))
    json.dump({k: [{kk: vv for kk, vv in r.items()} for r in G] for k, G in groups.items()}, open(OUT / 'passes.json', 'w', encoding='utf8'), default=str)


if __name__ == '__main__':
    main()
