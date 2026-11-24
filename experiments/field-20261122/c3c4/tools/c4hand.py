"""curve 4: state at the land4 -> band hand-over and how often the band target is the final command."""
import numpy as np, lib
from c4pass import passes
P = passes()
print('per pass at lap_ff4 x 1.2 / 1.5 / 1.8 m: tb, psi, y, steer | tb when band w first 1')
T = {1.2: [], 1.5: [], 1.8: []}
for q in P:
    A = q['A']; s = '%s L%d' % (q['run'], q['lap'])
    for x0 in (1.2, 1.5, 1.8):
        m = (A['x4'] >= x0 - 0.1) & (A['x4'] < x0 + 0.1)
        r = (np.nanmedian(A['tb'][m]), np.nanmedian(A['psi'][m]) if np.isfinite(A['psi'][m]).any() else np.nan, np.nanmedian(A['y'][m]) if np.isfinite(A['y'][m]).any() else np.nan, np.nanmedian(A['steer'][m]), np.nanmedian(A['err'][m]))
        T[x0].append(r); s += ' | %5.1f %+5.1f %+5.1f %+4.0f e%+.2f' % r
    print(s)
for x0, v in T.items():
    v = np.array(v); print('x %.1f  tb med %.1f (min %.1f max %.1f sd %.1f) psi med %+.1f y med %+.1f steer %+.0f err %+.2f' % (x0, np.nanmedian(v[:, 0]), np.nanmin(v[:, 0]), np.nanmax(v[:, 0]), np.nanstd(v[:, 0]), np.nanmedian(v[:, 1]), np.nanmedian(v[:, 2]), np.nanmedian(v[:, 3]), np.nanmedian(v[:, 4])))
n = fl = tp = pd_ = 0
for q in P:
    A = q['A']; m = (A['x4'] >= 1.5) & (A['tb'] < 190) & np.isfinite(A['btgt'])
    d = A['steer'][m]; n += m.sum(); fl += (np.abs(d - A['btgt'][m]) < 1.).sum(); tp += ((np.abs(d - A['btop'][m]) < 1.) & (np.abs(d - A['btgt'][m]) >= 1.)).sum(); pd_ += ((np.abs(d - A['pid'][m]) < 1.) & (np.abs(d - A['btgt'][m]) >= 1.)).sum()
print('frames x4>=1.5, tb<190: %d | final = band target %.1f%% | = band top %.1f%% | = PID chain (between) %.1f%% | other %.1f%%' % (n, 100 * fl / n, 100 * tp / n, 100 * pd_ / n, 100 * (n - fl - tp - pd_) / n))
for a, b in ((40, 70), (70, 110), (110, 150), (150, 180), (180, 195)):
    n = fl = tp = pd_ = 0
    for q in P:
        A = q['A']; m = (A['x4'] >= 1.5) & (A['tb'] >= a) & (A['tb'] < b) & np.isfinite(A['btgt'])
        d = A['steer'][m]; n += m.sum(); fl += (np.abs(d - A['btgt'][m]) < 1.).sum(); tp += ((np.abs(d - A['btop'][m]) < 1.) & (np.abs(d - A['btgt'][m]) >= 1.)).sum(); pd_ += ((np.abs(d - A['pid'][m]) < 1.) & (np.abs(d - A['btgt'][m]) >= 1.)).sum()
    print('   tb %3d-%3d: n %4d target %.0f%% top %.0f%% pid %.0f%%' % (a, b, n, 100 * fl / n, 100 * tp / n, 100 * pd_ / n))
