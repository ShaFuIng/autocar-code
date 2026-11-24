"""mask-dash-r1 (upper-single-r2, consolidation-20261111/upper/side_lines): the dashed centre line found on the V4 mask in ground
coordinates, as a second dash source when the dash-route fit (JPEG) has none. No solid-edge logic.
 - mask (vision mask_png, 640x360) -> connected components touching rows >= row0; their pixels are projected with the ground
   homography (camera_ground_calibration.json homography_image_to_ground_cm: image px -> ground cm, x + right, y forward from the
   front tyre) and kept inside [y_min_cm, y_max_cm] (calibrated range 30-140 cm, flat ground only).
 - dash blob: length dash_l_min_cm..dash_l_max_cm, width <= dash_w_max_cm (Part 1: dashes 15-18 cm long, ~5 cm wide).
 - sanity gate (does not need an exact calibration): adjacent member centroids mask_space_min_cm..mask_space_max_cm apart (11-10
   CAL dashed rows: centre spacing p5 16, median 25, p95 48 cm; a +-10 % scale error from a ~1 deg pitch error stays inside).
 - dashed line: >= 2 blobs whose centroids lie within dash_col_cm of the line through a pair, the pair direction within
   dir_gate_deg of the expected line angle (None = no gate); the group with the most blobs wins.
 - the ground line is mapped back to the image with the inverse homography and returned as x = a*y + b (640x360), i.e. the same form
   as the dash-route fit, so the lateral error is computed by exactly the same formula and k_p_us keeps its meaning.
angle_deg(a, b): ground direction of an image line x = a*y + b (deg, + = points right of the car axis), from points of the calibrated range.
Part 1 agreement with the dash-route fit: median 0.2 cm, p10 -1.1, p90 8.1 (n 409). Experiment values; not a verified limit."""
import base64, json, math
from pathlib import Path
import numpy as np
import cv2

REVISION = 'mask-dash-r1'
DEFAULTS = dict(mask_row0=100, mask_y_min_cm=30., mask_y_max_cm=140., mask_dash_l_min_cm=8., mask_dash_l_max_cm=25.,
                mask_dash_w_max_cm=10., mask_dash_col_cm=6., mask_dir_gate_deg=15., mask_max_px=800,
                mask_space_min_cm=15., mask_space_max_cm=60.)


def load_h(path=None):
    """(H, H_inverse) from camera_ground_calibration.json next to this file (web/), or raises."""
    d = json.loads(Path(path or Path(__file__).with_name('camera_ground_calibration.json')).read_text(encoding='utf8'))
    H = np.asarray(d['homography_image_to_ground_cm'], float)
    if H.shape != (3, 3) or not np.isfinite(H).all(): raise ValueError('bad homography')
    return H, np.linalg.inv(H)


def _proj(H, pts):
    p = np.column_stack([np.asarray(pts, float), np.ones(len(pts))]) @ H.T
    if np.any(np.abs(p[:, 2]) < 1e-9): raise ValueError('horizon')
    return p[:, :2] / p[:, 2:3]


class MaskDash:
    def __init__(self, cfg=None, H=None):
        self.c = dict(DEFAULTS); self.c.update({k: v for k, v in (cfg or {}).items() if k in DEFAULTS})
        if H is None: H, Hi = load_h()
        else: H = np.asarray(H, float); Hi = np.linalg.inv(H)
        self.H, self.Hi = H, Hi

    def angle_deg(self, a, b):
        c = self.c; ys = np.arange(float(c['mask_row0']), 358., 4.); xs = float(a) * ys + float(b); k = (xs >= 0) & (xs < 640)
        if k.sum() < 3: return None
        g = _proj(self.H, np.column_stack([xs[k], ys[k]])); g = g[(g[:, 1] >= c['mask_y_min_cm']) & (g[:, 1] <= c['mask_y_max_cm'])]
        if len(g) < 2: return None
        d = g[-1] - g[0] if g[-1][1] > g[0][1] else g[0] - g[-1]
        return math.degrees(math.atan2(d[0], d[1]))

    def blobs(self, mask):
        c = self.c; n, lab, st, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8); out = []
        r0 = int(c['mask_row0'])
        for k in range(1, n):
            x, y, w, h, a = st[k]
            if a < 30 or y + h <= r0: continue
            ys, xs = np.nonzero(lab[y:y + h, x:x + w] == k); ys = ys + y; xs = xs + x
            idx = np.flatnonzero(ys >= r0)
            if len(idx) < 20: continue
            if len(idx) > c['mask_max_px']: idx = idx[np.linspace(0, len(idx) - 1, int(c['mask_max_px'])).astype(int)]
            try: g = _proj(self.H, np.column_stack([xs[idx], ys[idx]]))
            except ValueError: continue
            g = g[(g[:, 1] >= c['mask_y_min_cm']) & (g[:, 1] <= c['mask_y_max_cm']) & (np.abs(g[:, 0]) <= 150)]
            if len(g) < 15: continue
            m = g.mean(0); _, _, vt = np.linalg.svd(g - m, full_matrices=False); d = vt[0]
            if d[1] < 0: d = -d
            pr = (g - m) @ d; pw = (g - m) @ np.array([-d[1], d[0]])
            L = float(np.percentile(pr, 98) - np.percentile(pr, 2)); W = float(np.percentile(pw, 98) - np.percentile(pw, 2))
            if c['mask_dash_l_min_cm'] <= L <= c['mask_dash_l_max_cm'] and W <= c['mask_dash_w_max_cm']: out.append((float(m[0]), float(m[1])))
        return out

    def find(self, mask_png, expect_deg=None):
        """mask_png: base64 PNG (str/bytes) or a decoded array. Returns (dict(a, b, ang, n, b_cm) or None, info)."""
        c = self.c
        if isinstance(mask_png, (str, bytes)):
            im = cv2.imdecode(np.frombuffer(base64.b64decode(mask_png), np.uint8), cv2.IMREAD_UNCHANGED)
            if im is None: return None, dict(reason='DECODE')
            if im.ndim == 3: im = im[..., 0]
            mask = im > 0
        else: mask = np.asarray(mask_png) > 0
        if mask.shape[:2] != (360, 640): return None, dict(reason='SIZE')
        B = self.blobs(mask)
        if len(B) < 2: return None, dict(reason='FEW_BLOBS', blobs=len(B))
        best = None
        for i in range(len(B)):
            for j in range(i + 1, len(B)):
                p, q = (B[i], B[j]) if B[i][1] <= B[j][1] else (B[j], B[i])
                if q[1] - p[1] < 5: continue
                ang = math.degrees(math.atan2(q[0] - p[0], q[1] - p[1]))
                if expect_deg is not None and abs(ang - float(expect_deg)) > c['mask_dir_gate_deg']: continue
                t = math.radians(ang); o0 = p[0] * math.cos(t) - p[1] * math.sin(t)
                mem = [b for b in B if abs(b[0] * math.cos(t) - b[1] * math.sin(t) - o0) <= c['mask_dash_col_cm']]
                if best is None or len(mem) > len(best[1]): best = (ang, mem)
        if best is None or len(best[1]) < 2: return None, dict(reason='NO_ROW', blobs=len(B))
        mem = sorted(best[1], key=lambda b: b[1]); sp = [math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(mem, mem[1:])]
        if any(not c['mask_space_min_cm'] <= x <= c['mask_space_max_cm'] for x in sp):   # sanity gate, tolerant to calibration error
            return None, dict(reason='SPACING', blobs=len(B), spacing_cm=[round(x, 1) for x in sp])
        M = np.array(best[1])
        if len(M) >= 3 and np.ptp(M[:, 1]) > 5: s_, i_ = np.polyfit(M[:, 1], M[:, 0], 1)
        else: s_ = math.tan(math.radians(best[0])); i_ = float(np.mean(M[:, 0] - s_ * M[:, 1]))
        ang = math.degrees(math.atan(s_))
        gy = np.array([c['mask_y_min_cm'] + 10., c['mask_y_max_cm'] - 30.]); gp = np.column_stack([s_ * gy + i_, gy])
        ip = _proj(self.Hi, gp)
        if abs(ip[1, 1] - ip[0, 1]) < 1e-6: return None, dict(reason='DEGENERATE')
        a = (ip[1, 0] - ip[0, 0]) / (ip[1, 1] - ip[0, 1]); b = ip[0, 0] - a * ip[0, 1]
        return dict(a=float(a), b=float(b), ang=float(ang), n=len(M), b_cm=float(i_ * math.cos(math.radians(ang)))), dict(reason='OK', blobs=len(B))
