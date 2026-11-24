"""CPU overlap candidate. Same-frame evidence only; no hardware interfaces.

One speculative history alignment can overlap the existing A/K processes.
No queued backlog, no stale-result reuse, no relaxed geometry or PID thresholds.
"""
import time
from concurrent.futures import ThreadPoolExecutor
import cv2
from k_integrated_live import IntegratedK
from history_gap_candidate import HistoryGap
from temporal_lane import _estimate_motion
from k_adaptive_pid import continuous_paths


class ParallelHistory(HistoryGap):
    def __init__(self, max_age=.3):
        super().__init__(max_age)
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='history-motion')
        self.future=None;self.key=None;self.stats={}

    def prefetch(self,image,t):
        a=self.anchor
        self.stats=dict(submitted=False,reused=False,wait_ms=0.,busy=False)
        if self.future is not None and not self.future.done():
            self.stats['busy']=True
            return
        self.key=None
        if a is None or not 0<t-a['t']<=self.max_age:return
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        self.key=(id(a),t)
        # Bypass temporal_lane's global per-frame cache: it belongs to A's thread.
        self.future=self.pool.submit(_estimate_motion,a['gray'],gray)
        self.stats['submitted']=True

    def motion(self,anchor,gray,t):
        start=time.perf_counter()
        if self.key==(id(anchor),t):
            result=self.future.result();self.stats['reused']=True
        else:
            result=_estimate_motion(anchor['gray'],gray)
        self.stats['wait_ms']=(time.perf_counter()-start)*1000
        return result

    def step(self,image,mask,t,result):
        out=super().step(image,mask,t,result,motion_provider=self.motion)
        out['gap_repair']['cpu_overlap']=dict(self.stats)
        return out

    def close(self):self.pool.shutdown(wait=True,cancel_futures=True)


class OptimizedIntegratedK(IntegratedK):
    revision='k-integrated-cpu-overlap-r1'
    def __init__(self,row=.6):
        super().__init__(row);self.history=ParallelHistory()

    def set_row(self,row):
        self.history.close();super().set_row(row);self.history=ParallelHistory()

    def preprocess(self,image,captured):
        if self.t is not None and (captured<=self.t or captured-self.t>1):self.history.close()
        result=super().preprocess(image,captured)
        if not isinstance(self.history,ParallelHistory):self.history=ParallelHistory()
        # Starts before V4 in live flow; offline replay omits model delay entirely.
        self.history.prefetch(self.prepared[1],captured)
        return result

    def close(self):
        self.history.close();super().close()


class ControlPriorityHistory(HistoryGap):
    def __init__(self,row=.6):
        super().__init__();self.row=row

    def step(self,image,mask,t,result):
        # Same feasible-row bounds as KAdaptivePID, without advancing PID state.
        supported=bool(result.get('valid')) and any(
            max(self.row,c[0][1]/719+.001)<=min(.95,c[-1][1]/719-.001)
            for c in continuous_paths(result.get('path',[])))
        out=super().step(image,mask,t,result,repair_enabled=not supported)
        out['gap_repair']['control_priority']=dict(current_supported=supported,repair_skipped=supported)
        return out


class ControlPriorityIntegratedK(IntegratedK):
    revision='k-integrated-junction-continuity-r4'
    def __init__(self,row=.6):
        super().__init__(row);self.history=ControlPriorityHistory(row)

    def set_row(self,row):
        super().set_row(row);self.history=ControlPriorityHistory(row)

    def preprocess(self,image,captured):
        result=super().preprocess(image,captured)
        if not isinstance(self.history,ControlPriorityHistory):self.history=ControlPriorityHistory(self.row)
        return result
