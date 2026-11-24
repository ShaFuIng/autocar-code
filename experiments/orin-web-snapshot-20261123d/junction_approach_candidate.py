"""Offline-only gate candidate; original A, K geometry and PID stay unchanged."""
from collections import deque
from junction_k_lane import LaneGate

class ApproachGate(LaneGate):
    def __init__(self):
        super().__init__();self.approach=deque();self.approach_until=None;self.last_t=None

    def can_publish_early(self):
        # A new approach vote can enable handoff on this frame. Do not early-publish
        # A using the old near-vote shortcut while an approach is being confirmed.
        if self.approach or self.approach_until is not None:return False
        return self.state in ('A','SUSPECT') and (sum(list(self.openings)[-11:])+1<5 or sum(list(self.near)[-7:])+1<3)

    def step(self,d,reference,compatible,continuous):
        t=d['capture_s']
        if self.last_t is not None and (t<=self.last_t or t-self.last_t>1):
            self.__init__()
        self.last_t=t
        while self.approach and t-self.approach[0][0]>1.5:self.approach.popleft()
        if self.approach_until is not None and t>self.approach_until:self.approach_until=None
        middle=next(z['rows'] for z in d['zones'] if z['name']=='middle')
        # A single line / disconnected neighbouring lane cannot establish the latch.
        supported=d['a_supported'] and d['owned_components']>=2 and continuous
        if self.state in ('A','SUSPECT') and supported and len(middle)>=3:
            self.approach.append((t,float(max(middle))))
        progress=self.approach[-1][1]-self.approach[0][1] if len(self.approach)>1 else 0.
        approaching=len(self.approach)>=5 and progress>=12
        if approaching and supported and self.state in ('A','SUSPECT'):
            # Expire from latest evidence, never refresh solely from retained state.
            self.approach_until=self.approach[-1][0]+1.
        retained=self.approach_until is not None and t<=self.approach_until
        old_state=self.state
        result=super().step(d,reference,compatible,continuous)
        # Original successful near-entry route is retained. Alternative approach
        # needs the exact existing same-frame K/entrance compatibility and quorum.
        ready=reference and d['reference_compatible'] and sum(self.refs)>=3
        if old_state in ('A','SUSPECT') and self.state in ('A','SUSPECT') and retained:
            self.state='SUSPECT'
            result.update(state=self.state,reason='CONFIRMED_APPROACH_WAIT_ENTRANCE')
            if ready and not self.cooldown:
                self.state='CROSS';self.good.clear();self.alpha=0.;self.clear=0;self.passed=False
                result.update(state='CROSS',reason='APPROACH_SEQUENCE_AND_CURRENT_ENTRANCE',alpha=0.,reached_opening=True)
        if old_state in ('CROSS','RETURN_A') and self.state in ('A','SUSPECT'):
            self.approach.clear();self.approach_until=None
        result.update(approach_votes=len(self.approach),approach_progress_px=progress,
                      approach_retained=retained,approach_expires_in=max(0,self.approach_until-t) if self.approach_until else 0,
                      entry_check_passed=bool(ready))
        return result
