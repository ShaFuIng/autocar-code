"""Experimental straight-branch preference; current-mask evidence only.

An observed zigzag can be bypassed using its straight approach segment. This
is bounded image-space extrapolation, not a metric route planner or a promise
that an unseen exit exists. Missing current support returns the ordinary path.
"""
import numpy as np
from standalone_line_path import LineEstimate, StandaloneLine, tracks, ROWS, prior_width

REVISION = 'straight-approach-r1'


class StraightLineEstimate(LineEstimate):
    def __init__(self, continue_steep=True):
        super().__init__(True)
        self.branch_t = None
        self.branch_width = None
        self.branch_side = None
        self.straight_started = False
        self.straight_finished = False

    def reset_route(self):
        self.branch_t = self.branch_width = self.branch_side = None
        self.straight_started = self.straight_finished = False

    def step(self, mask, t):
        previous_t = self.t
        started, finished = self.straight_started, self.straight_finished
        previous_width = (self.ratio if self.ratio is not None and self.ratio_t is not None
                          and 0 <= t-self.ratio_t <= 1. else 1.)
        base = super().step(mask, t)
        self.straight_started, self.straight_finished = started, finished
        if previous_t is None or not 0 < t-previous_t <= .4:
            self.branch_t = self.branch_width = self.branch_side = None
            if started:
                self.straight_finished = True
        if self.branch_t is not None and t-self.branch_t > 1. and started:
            self.straight_finished = True
        candidates = []
        raw_tracks = self.get_tracks(mask)
        for tr in raw_tracks:
            side = tr['side']
            if side not in ('L', 'R') or not self.last[side]:
                continue
            ys = sorted(tr['pts'], reverse=True)
            ref = self.last[side]
            common = [y for y in ys if y in ref]
            if len(common) < 4 or np.median([abs(tr['pts'][y]-ref[y]) for y in common]) > 10:
                continue
            near = [y for y in ys if y >= ys[0]-64]
            if len(near) < 10 or near[0]-near[-1] < 40:
                continue
            a, b = np.polyfit(near, [tr['pts'][y] for y in near], 1)
            if np.sqrt(np.mean([(tr['pts'][y]-a*y-b)**2 for y in near])) > 2.5:
                continue
            # A sharp reversal is a branch clue; a merely steep bend is not.
            xs = [tr['pts'][y] for y in ys]
            reversals = []
            for i in range(6, len(ys)-6):
                s0 = (xs[i]-xs[i-6])/(ys[i]-ys[i-6])
                s1 = (xs[i+6]-xs[i])/(ys[i+6]-ys[i])
                if s0*s1 < 0 and abs(s0-s1) > 2 and min(abs(s0),abs(s1)) > .5:
                    reversals.append(ys[i])
            # Grow the currently observed straight seed until it actually ends.
            ymin = near[-1]
            for y in ys[len(near):]:
                if ymin-y > 12 or abs(tr['pts'][y]-a*y-b) > 6:
                    break
                ymin = y
            opposite = [other for other in raw_tracks if other['slope']*tr['slope'] < 0]
            # A bend seen between two near boundaries is still ordinary lane
            # following. Require an opening on the opposite side as well as
            # the reversal, rather than relabeling every distant bend a fork.
            opening = (bool(opposite) and max(other['ymax'] for other in opposite) < 240
                       and any(other['side']=='X' and other['ymax'] < 216 for other in opposite))
            candidates.append(dict(side=side, a=float(a), b=float(b), ymin=ymin,
                                   ymax=ys[0], reversals=reversals, opening=opening))
        evidence = [c for c in candidates if c['reversals'] and c['opening']]
        if evidence and not self.straight_finished:
            seed = max(evidence, key=lambda c:c['ymax']-c['ymin'])
            if self.branch_t is None or t-self.branch_t > 1. or seed['side'] != self.branch_side:
                self.branch_width = float(np.clip(previous_width, .7, 1.4))
            self.branch_t, self.branch_side = t, seed['side']
        active = not self.straight_finished and self.branch_t is not None and 0 <= t-self.branch_t <= 1.
        seed = next((c for c in candidates if c['side']==self.branch_side), None) if active else None
        detail = dict(revision=REVISION, route='next_junction_straight_once', active=False,
                      reason='STRAIGHT_USED' if self.straight_finished else 'NO_CURRENT_BRANCH', side=None, width_ratio=None,
                      max_extension_rows=80, branch_memory_s=1.)
        if seed is not None:
            points = []
            for y in ROWS:
                if not seed['ymin']-80 <= y <= seed['ymax']+80:
                    continue
                x = seed['a']*y+seed['b']
                x += (1 if seed['side']=='L' else -1)*self.branch_width*self.width(y)/2
                if np.isfinite(x) and (self.allow_offscreen or 0 <= x < 640):
                    points.append([float(x*2), y*2])
            if len(points) >= 3:
                self.straight_started = True
                base['path'] = points
                base['width_ratio'] = self.branch_width
                base['width_source'] = 'straight_entry_width'
                base['sources'] = {'straight_approach': len(points)}
                base['line_fit'] = [seed['side']]
                # Do not learn the widening of a branch as the road width.
                self.ratio, self.ratio_t = self.branch_width, t
                detail.update(active=True, reason='CURRENT_STRAIGHT_APPROACH',
                              side=seed['side'], width_ratio=self.branch_width,
                              observed_rows=[seed['ymin'],seed['ymax']],
                              reversals=seed['reversals'])
        elif active:
            detail['reason'] = 'NO_CURRENT_STRAIGHT_SUPPORT'
        detail['route_state'] = 'finished' if self.straight_finished else 'active' if self.straight_started else 'waiting'
        base['straight_selection'] = detail
        return base


class StraightPreviewLine(StandaloneLine):
    def set_row(self, row):
        super().set_row(row)
        self.estimate = StraightLineEstimate()

    def step(self, image, mask, captured):
        result = super().step(image, mask, captured)
        result['path_diagnostic'].update(path_revision='standalone-line-v3',
            estimator_revision=REVISION, controller_revision='preview-turn-r3',
            junction_policy='Next observed opening straight once per run; current straight seed, bounded extrapolation')
        return result
