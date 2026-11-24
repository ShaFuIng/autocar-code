"""Bounded temporal color correction; pure image preprocessing, offline candidate."""
import cv2
import numpy as np

# model-stage-r1 (model-stage-20261117): web/model_stage.json "warm_fast": true (read at import = worker start) switches step()
# to an equivalent that (a) converts to HSV only the rows 190:335 that the ROI uses (per-pixel conversion -> same values),
# (b) takes the per-channel median from a 256-bin histogram (same value as np.median, incl. the even-count mean), and
# (c) applies the gains through a per-channel lookup table built with the same float64 expression (same bytes).
# Absent / false = the original code below, unchanged.
try:
    import json as _json; from pathlib import Path as _Path
    _FAST = bool(_json.loads(_Path(__file__).with_name('model_stage.json').read_text(encoding='utf8')).get('warm_fast', False))
except Exception:
    _FAST = False
ROI = (190, 335)


def _hist_median(v):
    """np.median(v) for a 1-D uint8 array (float64), via counts."""
    n = v.size
    c = np.cumsum(np.bincount(v, minlength=256))
    a = int(np.searchsorted(c, (n - 1) // 2, side='right')); b = int(np.searchsorted(c, n // 2, side='right'))
    return (a + b) / 2.0 if a != b else float(a)


class WarmColor:
    def __init__(self):
        self.gains=np.ones(3);self.target=np.ones(3);self.last_warm=None;self.t=None
    def step(self,image,t):
        if _FAST: return self._step_fast(image,t)
        if self.t is not None and (t<=self.t or t-self.t>1):self.__init__()
        dt=min(.2,max(0,t-self.t)) if self.t is not None else 0.
        self.t=t
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        roi=np.zeros(hsv.shape[:2],bool);roi[190:335]=True
        bright=roi&(hsv[:,:,2]>160)&(hsv[:,:,1]<150)
        detected=False
        if bright.sum()>150:
            med=np.median(image[bright],axis=0).astype(float)
            detected=med[2]/max(med[0],1)>1.15
            if detected:
                self.target=np.clip(np.mean(med)/np.maximum(med,1),.8,1.55)
                self.last_warm=t
        held=not detected and self.last_warm is not None and t-self.last_warm<=.3
        if not detected and not held:self.target=np.ones(3)
        self.gains+=np.clip(self.target-self.gains,-.65*dt,.65*dt)
        active=bool(np.max(abs(self.gains-1))>1e-6)
        result=np.clip(image.astype(float)*self.gains,0,255).astype('uint8') if active else image
        return result,dict(active=active,detected=bool(detected),held=held,gains=self.gains.tolist(),target=self.target.tolist())
    def _step_fast(self,image,t):
        if self.t is not None and (t<=self.t or t-self.t>1):self.__init__()
        dt=min(.2,max(0,t-self.t)) if self.t is not None else 0.
        self.t=t
        sub=image[ROI[0]:ROI[1]]
        hsv=cv2.cvtColor(sub,cv2.COLOR_BGR2HSV)
        bright=(hsv[:,:,2]>160)&(hsv[:,:,1]<150)
        detected=False
        if np.count_nonzero(bright)>150:
            px=sub[bright]
            med=np.array([_hist_median(np.ascontiguousarray(px[:,c])) for c in range(3)],dtype=float)
            detected=med[2]/max(med[0],1)>1.15
            if detected:
                self.target=np.clip(np.mean(med)/np.maximum(med,1),.8,1.55)
                self.last_warm=t
        held=not detected and self.last_warm is not None and t-self.last_warm<=.3
        if not detected and not held:self.target=np.ones(3)
        self.gains+=np.clip(self.target-self.gains,-.65*dt,.65*dt)
        active=bool(np.max(abs(self.gains-1))>1e-6)
        if active and image.ndim==3 and image.shape[2]==3 and image.dtype==np.uint8:
            lut=np.clip(np.arange(256,dtype=float)[:,None]*self.gains,0,255).astype('uint8').reshape(256,1,3)
            result=cv2.LUT(image,lut)
        elif active:
            result=np.clip(image.astype(float)*self.gains,0,255).astype('uint8')
        else:result=image
        return result,dict(active=active,detected=bool(detected),held=held,gains=self.gains.tolist(),target=self.target.tolist())
