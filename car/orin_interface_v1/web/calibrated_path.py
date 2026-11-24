"""V7 inner-edge geometry -> image path for the existing visual PID.

Centimetres retain the calibration lane/front-tyre reference, not rear-axle
coordinates. No pure-pursuit controller, hardware, network, or old-camera reuse.
"""
import hashlib
import json
from pathlib import Path
import sys
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'laptop_workbench'))
from temporal_lane import thin, graph_path, row_curve
from ground_calibration import project

CALIBRATION = Path(__file__).with_name('calibration_v7.json')


def load_calibration(path=CALIBRATION):
    raw = Path(path).read_bytes()
    data = json.loads(raw)
    H = np.asarray(data['matrix'], float)
    if data['source_revision'] != 7 or data['source_snapshot']['width'] != 640 or data['source_snapshot']['height'] != 360:
        raise ValueError('V7 calibration identity/size mismatch')
    if H.shape != (3, 3) or not np.isfinite(H).all() or abs(np.linalg.det(H)) < 1e-12:
        raise ValueError('Invalid V7 matrix')
    if not 10 < data['lane_width_cm'] < 200:
        raise ValueError('Invalid V7 width')
    return data, H, hashlib.sha256(raw).hexdigest()


def validate_row(row):
    _, H, _ = load_calibration()
    z = project([[319.5, row*719/2]], H)[0, 1]
    if not 50 <= z <= 120:
        raise ValueError('v7前視列超出50～120cm量測範圍；可先使用近列0.60、遠列0.50')
    return float(z)


def offset(points, distance, side):
    tangent = np.gradient(points, axis=0)
    norm = np.linalg.norm(tangent, axis=1)
    if np.any(norm < 1e-8):
        raise ValueError('Degenerate boundary')
    normal = np.column_stack([tangent[:, 1], -tangent[:, 0]])/norm[:, None]
    result = points+normal*distance*(1 if side == 'left' else -1)
    if np.any(np.diff(result[:, 1]) <= 0) or np.any(
            np.sum(np.diff(result, axis=0)*np.diff(points, axis=0), axis=1) <= 0):
        raise ValueError('Offset foldback')
    return result


class CalibratedPath:
    def __init__(self, path=CALIBRATION):
        self.calibration, self.H, self.sha = load_calibration(path)
        self.inverse = np.linalg.inv(self.H)
        self.mode = self.pending = self.time = None
        self.count = 0
        self.streaks = dict.fromkeys(('double', 'left', 'right'), 0)
        self.previous = {}

    def step(self, image, mask, t):
        if image.shape[1]*9 != image.shape[0]*16 or mask.shape != image.shape[:2]:
            raise ValueError('V7 requires uncropped 16:9 image and matching mask')
        if not np.isfinite(t):
            raise ValueError('Invalid capture time')
        if self.time is None or not 0 < t-self.time <= .5:
            self.mode = self.pending = None
            self.count = 0
            self.streaks = dict.fromkeys(self.streaks, 0)
            self.previous = {}
        self.time = t
        mask = cv2.resize(mask.astype(np.uint8), (640, 360),
                          interpolation=cv2.INTER_NEAREST).astype(bool)
        sk = thin(mask)
        sk[:100] = 0
        edges = {}
        fits = {}
        for side in ('left', 'right'):
            points = []
            rows = row_curve(graph_path(sk, side, np.empty((0, 2))))
            for y, x in rows.items():
                y, x = int(y), int(round(x))
                if not mask[y, x]:
                    continue
                step = 1 if side == 'left' else -1
                while 0 <= x+step < 640 and mask[y, x+step]:
                    x += step
                p = project([[x, y]], self.H)[0]
                if 50 <= p[1] <= 120:
                    points.append(p)
            if len(points) < 8:
                fits[side] = dict(points=len(points), residual_cm=None, reason='INSUFFICIENT_POINTS')
                continue
            a = np.asarray(points)
            a = a[np.argsort(a[:, 1])]
            if np.any(np.diff(a[:, 1]) <= 0):
                fits[side] = dict(points=len(points), residual_cm=None, reason='NON_MONOTONIC')
                continue
            z = np.linspace(a[0, 1], a[-1, 1], 60)
            coefficients = np.polyfit(a[:, 1], a[:, 0], 2)
            residual = np.polyval(coefficients, a[:, 1])-a[:, 0]
            # Local smooth fit only inside observed support; reject complex
            # shapes instead of forcing a quadratic through a sharp turn.
            rms = float(np.sqrt(np.mean(residual**2)))
            fits[side] = dict(points=len(points), residual_cm=rms,
                              reason='FIT_RESIDUAL' if rms > 2. else 'OK')
            if rms > 2.:
                continue
            x = np.polyval(coefficients, z)
            edges[side] = np.column_stack([x, z])
        candidates = {}
        widths = None
        if len(edges) == 2:
            lo = max(p[0, 1] for p in edges.values())
            hi = min(p[-1, 1] for p in edges.values())
            if hi > lo:
                z = np.linspace(lo, hi, 60)
                left, right = [np.interp(z, edges[s][:, 1], edges[s][:, 0]) for s in ('left', 'right')]
                widths = [float((right-left).min()), float((right-left).max())]
                if np.all(right > left):
                    candidates['double'] = np.column_stack([(left+right)/2, z])
        # Prepare both singles even while double is selected; no blended weights.
        for side in edges:
            try:
                p = offset(edges[side], self.calibration['lane_width_cm']/2, side)
                p = p[(p[:, 1] >= 50) & (p[:, 1] <= 120)]
                if len(p) >= 8:
                    candidates[side] = p
            except ValueError:
                pass
        diagnostics = {}
        # A contradictory pair is not evidence for arbitrarily choosing a side.
        conflict = len(edges) == 2 and 'double' not in candidates
        for name in self.streaks:
            candidate = candidates.get(name, np.empty((0, 2)))
            pixels = project(candidate, self.inverse)*2 if len(candidate) else np.empty((0, 2))
            usable = not conflict and len(pixels) >= 8 and bool(
                np.isfinite(pixels).all() and np.all((pixels >= 0) & (pixels < [1280, 720])))
            delta = None
            old = self.previous.get(name)
            if old is not None and len(candidate):
                lo, hi = max(old[0, 1], candidate[0, 1]), min(old[-1, 1], candidate[-1, 1])
                if hi > lo:
                    zs = np.linspace(lo, hi, 30)
                    delta = float(np.mean(np.abs(np.interp(zs, old[:, 1], old[:, 0]) -
                                                 np.interp(zs, candidate[:, 1], candidate[:, 0]))))
            self.streaks[name] = self.streaks[name]+1 if usable else 0
            diagnostics[name] = dict(usable=usable, consecutive_valid=self.streaks[name],
                ready=self.streaks[name] >= 3, points=len(candidate),
                support_cm=[float(candidate[0, 1]), float(candidate[-1, 1])] if len(candidate) else None,
                change_cm=delta, boundary_fits={s: fits[s] for s in
                    (('left', 'right') if name == 'double' else (name,)) if s in fits},
                reason='PAIR_CONFLICT' if conflict else 'OK' if usable else 'NO_USABLE_CURRENT_PATH')
        self.previous = {k: v.copy() for k, v in candidates.items() if diagnostics[k]['usable']}
        before = self.mode
        ready = [k for k in ('double', 'left', 'right') if diagnostics[k]['ready']]
        if 'double' in ready:
            self.mode = 'double'
        elif self.mode not in ready:
            self.mode = next(iter(ready), None)
        desired = next((k for k in diagnostics if diagnostics[k]['usable']), None)
        transition = ('HOLD' if before == self.mode else 'BACKGROUND_FALLBACK'
                      if before == 'double' and self.mode in ('left', 'right')
                      else 'CONFIRMED_DOUBLE' if self.mode == 'double'
                      else 'CONFIRMED_SINGLE' if self.mode else 'NO_READY_CURRENT_PATH')
        path = candidates.get(self.mode, np.empty((0, 2)))
        pixels = project(path, self.inverse)*2 if len(path) else np.empty((0, 2))
        valid = len(pixels) >= 8 and bool(np.all((pixels >= 0) & (pixels < [1280, 720])))
        return dict(path=pixels.tolist() if valid else [], metric_path_cm=path.tolist() if valid else [],
                    valid=valid, quality=1. if valid else 0., calibration_sha256=self.sha,
                    mode=self.mode if valid else 'confirming' if desired else 'no_evidence',
                    projected_width_cm=widths, hardware_output=False,
                    candidates=diagnostics, transition=transition, switch_version=1,
                    reason='V7_CURRENT_INNER_EDGES' if valid else 'V7_NO_CONFIRMED_PATH')
