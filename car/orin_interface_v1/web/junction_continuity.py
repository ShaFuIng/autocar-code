"""Committed junction recovery; baseline SegmentCandidate remains unchanged.

Memory is a selected exit identity, not permission to replay stale commands.
Only current mask support can bridge a missing reference. Units: 640 x 360.
"""
import copy
import cv2
import numpy as np
from junction_segment_candidate import SegmentCandidate, ExitGate


class CommittedExitGate(ExitGate):
    def step(self, d, reference, compatible, continuous):
        reset = self.last_t is not None and (d['capture_s'] <= self.last_t or d['capture_s']-self.last_t > 1)
        previous = 'A' if reset else self.state
        r = super().step(d, reference, compatible, continuous)
        if previous == 'CROSS' and not reference:
            # Preserve intent only; output still needs current A or a supported K.
            self.state = 'CROSS'
            self.cooldown = 0
            self.clear = 0
            r.update(state='CROSS', reason='CROSS_REFERENCE_MISSING_WAIT_EVIDENCE')
        return r


def mask_support(mask, fits):
    """Match the selected exit against narrow, continuous current white runs.

    Same 14 px / 10 degree association bounds as the visible reference bridge;
    no entrance-at-y270 requirement while already committed to a junction.
    """
    m = cv2.resize(mask.astype('uint8'), (640, 360), interpolation=cv2.INTER_NEAREST) > 0
    sides = []
    for side, fit in enumerate(fits):
        points = []
        for y in range(170, 310, 4):
            expected = float(np.polyval(fit, y))
            if not 0 <= expected < 640:
                continue
            ds = np.diff(np.r_[False, m[y], False].astype('int8'))
            runs = [(l, r) for l, r in zip(np.flatnonzero(ds == 1), np.flatnonzero(ds == -1)) if 2 <= r-l <= 45]
            if not runs:
                continue
            # Fits in the K tracker describe white-line centers, not inner edges.
            x = min(((l+r-1)/2 for l, r in runs), key=lambda x: abs(x-expected))
            if abs(x-expected) < 14:
                points.append((x, y, x-expected))
        # Require a contiguous span; scattered nearby paint is not a boundary.
        groups = []
        for p in points:
            if not groups or p[1]-groups[-1][-1][1] > 8:
                groups.append([])
            groups[-1].append(p)
        p = max(groups, key=len, default=[])
        if len(p) < 6 or p[-1][1]-p[0][1] < 20:
            continue
        p = np.asarray(p)
        observed = np.polyfit(p[:, 1], p[:, 0], 1)
        angle = abs(np.degrees(np.arctan(observed[0])-np.arctan(fit[0])))
        if angle >= 10 or np.percentile(abs(p[:, 0]-np.polyval(observed, p[:, 1])), 90) >= 3:
            continue
        sides.append(dict(side=side, rows=len(p), span=float(p[-1, 1]-p[0, 1]),
                          delta=float(np.median(p[:, 2])), angle=float(angle)))
    if not sides or (len(sides) == 2 and abs(sides[0]['delta']-sides[1]['delta']) > 8):
        return None, sides
    delta = float(np.clip(np.median([s['delta'] for s in sides]), -5, 5))
    aligned = np.asarray(fits, float).copy()
    aligned[:, 1] += delta
    return aligned.tolist(), sides


class CommittedSegmentCandidate(SegmentCandidate):
    def __init__(self, row=.6):
        super().__init__(row)
        self.gate = CommittedExitGate()
        self.exit_anchor = None
        self.continuity_t = None

    def combine(self, a, v, mask, d=None, *, augmented=False):
        t = float(d['capture_s'] if d is not None else a['capture_s'])
        if self.continuity_t is not None and (t <= self.continuity_t or t-self.continuity_t > 1):
            self.exit_anchor = None
        self.continuity_t = t
        anchor = self.exit_anchor
        committed = self.gate.state == 'CROSS' and anchor is not None
        continuity = dict(committed=committed, source='CURRENT_K', supported_sides=[], age_s=None)
        v = copy.deepcopy(v)
        if committed:
            age = t-anchor['t']
            continuity['age_s'] = age
            fits = v['fused']
            # Identity check applies to a newly found reference as well as a gap.
            same = fits is not None and max(abs(np.polyval(f, y)-np.polyval(old, y))
                    for f, old in zip(fits, anchor['fits']) for y in (190, 230, 270)) < 30
            if not same:
                aligned, support = mask_support(mask, anchor['fits']) if 0 < age <= .4 else (None, [])
                v['fused'] = aligned
                continuity.update(source='CURRENT_MASK_BRIDGE' if aligned is not None else 'NO_SUPPORTED_REFERENCE',
                                  supported_sides=support)
                v['decision'] = continuity['source']
        r = super().combine(a, v, mask, d, augmented=augmented)
        if r['k']['state'] == 'CROSS' and v['fused'] is None:
            # Retain the existing guarded A fallback, not stale K. Existing
            # continuous-path / sample-row validation decides if it can control.
            continuity['fallback'] = 'CURRENT_A' if r['valid'] else 'NO_PATH'
        elif r['k']['state'] == 'CROSS' and continuity['source'] == 'CURRENT_K' and (
                self.exit_anchor is None or v.get('decision') in (
                'CURRENT_ONLY_NO_WARP', 'VISIBLE_FUSE', 'CONFIRMED_VISIBLE_CORRECTION', 'GEOMETRY_BRIDGE_FUSE')):
            self.exit_anchor = dict(t=t, fits=copy.deepcopy(v['fused']))
        if r['k']['state'] not in ('CROSS', 'RETURN_A'):
            self.exit_anchor = None
        if continuity['source'] == 'CURRENT_MASK_BRIDGE':
            for segment in r.get('road_segments', []):
                segment.update(source='junction_inferred', inference='current_mask_bridge', age_s=continuity['age_s'])
            for p in r.get('path_samples', []):
                p.update(source='junction_inferred', inference='current_mask_bridge', age_s=continuity['age_s'])
        r['k']['continuity'] = continuity
        r['candidate_revision'] = 'committed-junction-r1'
        return r
