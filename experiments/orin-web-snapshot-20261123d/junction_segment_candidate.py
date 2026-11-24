"""Offline candidate: selected-chain guard, exit hysteresis, supported K extent.

Not registered in the live algorithm menu. All coordinates below are 640x360.
"""
import copy
import numpy as np
from junction_k_lane import KLanePath, lane_evidence
from junction_k import evidence
from junction_branch_candidate import TemporalBranchGate, augment, guard_single
from junction_geometry_guard import geometry_guard


def selected_guard(a):
    """Only a join belonging to the selected chain may cut that chain."""
    selected = {(round(p[0] / 2, 4), round(p[1] / 2, 4)) for p in a['path']}
    scoped = copy.deepcopy(a)
    scoped['path_samples'] = [p for p in a.get('path_samples', [])
                              if (round(p['x'], 4), round(p['y'], 4)) in selected]
    guarded = geometry_guard(scoped)
    return guarded


class ExitGate(TemporalBranchGate):
    def __init__(self):
        super().__init__()
        self.exit_quality = False

    def step(self, d, reference, compatible, continuous):
        # Do not change entrance votes, entrance matching, or CROSS-loss handling.
        adjusted = copy.deepcopy(d)
        if self.state in ('CROSS', 'RETURN_A') and self.exit_quality and continuous:
            compatible = True
            adjusted['a_supported'] = True
        result = super().step(adjusted, reference, compatible, continuous)
        result['exit_supported_quality'] = self.exit_quality
        return result


class SegmentCandidate(KLanePath):
    def __init__(self, row=.6):
        super().__init__(row)
        self.gate = ExitGate()
        self.extent = None

    def combine(self, a, v, mask, d=None, *, augmented=False):
        if not augmented:
            d = augment(lane_evidence(mask, a, v) if d is None else d, mask, a)
        clean = selected_guard(a)
        ev = evidence(clean, v['fused'])
        # Permit a small number of missing rows, only with strong geometry.
        # Original 5-frame quorum and continuous-A check remain in force.
        self.gate.exit_quality = bool(
            ev['double_rows'] >= 16 and ev['span'] >= 60
            and ev['center_error'] is not None and ev['center_error'] < 12
            and ev['width_error'] is not None and ev['width_error'] < .15
            and ev['angle_error'] is not None and ev['angle_error'] < 8
            and d['reference_compatible'] and not d['opening'])
        # Entrance decision uses the original evidence, not a newly shortened path.
        r = super().combine(guard_single(a, self.gate), v, mask, d)
        if clean.get('geometry_guard') and r['k']['source'] in ('A', 'blend'):
            cutoff = clean['geometry_guard']['cutoff_y'] * 2
            r['path'] = [p for p in r['path'] if p[1] >= cutoff]
            r['geometry_guard'] = clean['geometry_guard']
        if r['k']['source'] == 'K':
            f = np.mean(v['fused'], axis=0)
            # Extend only over a continuous currently selected chain agreeing
            # with the K line. Do not extend to unrelated diagnostic candidates.
            support = []
            for p in clean.get('path_samples', []):
                if p['source'] in ('double', 'single') and abs(p['x'] - np.polyval(f, p['y'])) < 12:
                    support.append(p['y'])
            target = [160., 334.]
            if len(support) >= 4:
                target = [min(160., min(support)), max(334., max(support))]
            target = [max(100., target[0]), min(354., target[1])]
            if self.extent is None:
                # Keep the previous actual output extent on takeover if it is
                # supported now; otherwise retain the original K core.
                self.extent = [160., 334.]
            self.extent = [float(old + np.clip(new-old, -4, 4)) for old, new in zip(self.extent, target)]
            ys = np.arange(self.extent[0], self.extent[1] + .1, 2)
            r['path'] = [[float(np.polyval(f, y)*2), float(y*2)] for y in ys]
            r['k']['extent'] = dict(range=self.extent[:], target=target, observed_rows=len(support))
        else:
            self.extent = None
        r['valid'] = len(r['path']) >= 2
        # Keep all displayed/consumed segments consistent with the chosen path.
        if r.get('geometry_guard'):
            allowed = {p[1] for p in r['path']}
            for field in ('road_segments', 'history_segments'):
                for segment in r.get(field, []):
                    segment['points'] = [p for p in segment['points'] if p[1] in allowed]
            r['history_path'] = [p for p in r.get('history_path', []) if p[1] in allowed]
            r['path_samples'] = [p for p in r.get('path_samples', []) if p['y']*2 in allowed]
        if r['k']['source'] == 'K':
            r['road_segments'] = [dict(source='junction_inferred', points=r['path'], confidence=.35, age_s=0)]
            r['path_samples'] = [dict(x=x/2,y=y/2,source='junction_inferred',confidence=.35,age_s=0) for x,y in r['path']]
        r['candidate_revision'] = 'selected-chain-exit-extent-r1'
        return r
