"""frame-by-frame trace around a curve exit.  python trace.py <run> <lap> [C1e|C2e] [t_from] [t_to]"""
import sys, numpy as np, lib
from lib import g
run, lap = sys.argv[1], int(sys.argv[2]); an = sys.argv[3] if len(sys.argv) > 3 else 'C1e'
a = float(sys.argv[4]) if len(sys.argv) > 4 else -1.0; b = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
Q = lib.load(run)
t = np.array([p['t'] for p in Q]); enc = np.array([g(p, 'inputs', 'enc_b') for p in Q], float); v = lib.speed(t, enc)
i0 = next(i for i, p in enumerate(Q) if p['_lap'] == lap and g(p, 'lap_pos', 'anchor', default=None) == an and g(p, 'lap_pos', 'lap', default=-1) is not None and (i == 0 or g(Q[i - 1], 'lap_pos', 'anchor', default=None) != an))
t0 = t[i0]; yaw = 0.; print('run', run, 'lap', lap, an, 't0', round(t0, 2))
print('   dt   x_m   e_near e_ctl    P     I     D   des  gk  cc   yin  cff  lff  ctr | pre   pc  sent | gz    yaw   pwm   v   | y_cm  psi  clrin | cmap           own')
yy = 0.
for i in range(len(Q)):
    dt = t[i] - t0
    if dt < a or dt > b: continue
    p = Q[i]; gz = g(p, 'inputs', 'gz')
    if i > 0 and np.isfinite(gz): yy += gz * (t[i] - t[i - 1])
    C = lib.CENTRE
    cc = g(p, 'cmd_comp', 'comp_us', default=None); ccs = '%4.0f' % cc if cc is not None else ' ' + str(g(p, 'cmd_comp', 'reason', default='-'))[:3]
    cff = g(p, 'curve_ff', 'floor_us', default=None); lff = g(p, 'lap_ff', 'band', 'tgt_us', default=None) or g(p, 'lap_ff', 'floor_us', default=None)
    ctr = g(p, 'curve_trim', 'trim_us', default=None)
    pc = p.get('play_comp') or {}
    cm = p.get('curve_map') or {}
    al = p.get('arc_lat') or {}
    print('%6.2f %5.2f %6.2f %5.2f %5.0f %5.0f %5.0f %5.0f %3.2f %s %5.0f %4s %4s %4s | %4.0f %4.0f %5.0f | %5.1f %6.1f %4.0f %4.2f | %5s %5s %5s | %s %s %s' % (
        dt, g(p, 'lap_pos', 'since_anchor_m'), g(p, 'image_error'), g(p, 'error'), g(p, 'p'), g(p, 'i'), g(p, 'd'), g(p, 'desired_us') - C, g(p, 'gain_schedule', 'k'), ccs,
        g(p, 'yaw_inner', 'term_us'), '-' if cff is None else '%.0f' % (cff - C), '-' if lff is None else '%.0f' % (lff - C), '-' if ctr is None else '%.0f' % ctr,
        g(p, 'steer_us') - C, pc.get('offset_us', np.nan), pc.get('steer_us', np.nan) - C, gz, yy, g(p, 'pwm'), v[i],
        al.get('y_cm'), al.get('psi_deg'), al.get('clr_in_cm'), cm.get('reason'), cm.get('turned_deg'), p.get('owner')))
