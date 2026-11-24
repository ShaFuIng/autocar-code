"""model-stage-r1 (model-stage-20261117): faster V4 mask stage with the SAME output, behind web/model_stage.json.

Read once at worker start (live_model_worker.V4.__init__). File absent / all keys false = this module is not used at all
(the worker runs its original V4.predict line by line).

Keys (all default false):
  "direct":     call the ultralytics predictor's own preprocess / inference / postprocess functions directly instead of
                model.predict() (skips setup_source, the generator, profilers, callbacks per frame). Same functions -> same
                tensors. Version independent.
  "cuda_graph": capture ONE CUDA graph for the fixed input shape (360x640x3 uint8, the shape every camera-pair frame has
                after standalone.preprocess): letterbox pad + BGR->RGB + /255 (a replica of the predictor's preprocess),
                the predictor's own inference() call (the network), and a replica of the semantic postprocess
                (bilinear upsample, letterbox crop, argmax, ==1). Per frame: one host->device copy, one graph replay, one
                device->host copy. Removes the per-kernel CPU launch cost of eager PyTorch (the Orin diagnosis of
                2026-11-17 found the model stage CPU-bound: main thread 0.93 CPU for the whole 27 ms).
                Other input shapes fall back to "direct" (or the original call if direct is off).
  "self_check": default true when any switch is on. At setup (during the worker's warm-up, before 'ready') the fast path
                is run on synthetic frames and compared with the original model.predict() result (exact equality of
                the preprocessed tensor and of the final boolean mask). Any difference -> that level is disabled for
                the life of the worker (falls back to the next level / the original). Result is written to stderr
                (pid-model.log) as one JSON line {"model_stage": ...}.
  "cudnn_benchmark": with cuda_graph only: torch.backends.cudnn.benchmark = True before the capture (cuDNN times its kernel
                choices for this shape; can change float rounding -> the self check decides; laptop: 300/300 frames identical).
  "verify_every": N > 0 = every N-th real frame also runs the original call and compares; on the first mismatch the fast
                path is switched off for the rest of the worker's life (logged). Default 0 (no extra cost).
"""
import json, sys, time
from pathlib import Path
import numpy as np

REVISION = 'model-stage-r1'
SHAPE = (360, 640, 3)


def load_cfg(here):
    try:
        cfg = json.loads(Path(here).with_name('model_stage.json').read_text(encoding='utf8'))
        return cfg if isinstance(cfg, dict) else {}
    except Exception:
        return {}


def _log(obj):
    try:
        sys.stderr.write(json.dumps({'model_stage': obj}, default=repr) + '\n'); sys.stderr.flush()
    except Exception:
        pass


def _synthetic(seed):
    """test frames for the self check: road-like white stripes on grey plus noise (gives both classes)."""
    rng = np.random.default_rng(seed)
    img = rng.integers(40, 120, SHAPE, dtype=np.uint8)
    for x0, slope in ((150, 0.9), (470, -0.9), (300, 0.1)):
        for y in range(120, 360):
            x = int(x0 + slope * (y - 120))
            if 0 <= x < 630: img[y, max(0, x - 4):x + 5] = 235
    img[rng.random(SHAPE[:2]) < 0.01] = 255
    return img


class FastV4:
    def __init__(self, yolo, ref_predict, cfg):
        """yolo = ultralytics YOLO object already used once (predictor exists); ref_predict(image) = the original
        V4.predict body (returns the bool mask)."""
        import torch
        self.torch = torch
        self.yolo = yolo; self.ref = ref_predict; self.cfg = cfg
        self.p = yolo.predictor
        self.want_direct = bool(cfg.get('direct')) or bool(cfg.get('cuda_graph'))
        self.want_graph = bool(cfg.get('cuda_graph'))
        self.self_check = bool(cfg.get('self_check', True))
        self.verify_every = int(cfg.get('verify_every') or 0)
        self.direct_ok = False; self.graph_ok = False; self.n = 0; self.mismatch = None
        self.status = dict(revision=REVISION, cfg=cfg)
        t0 = time.monotonic()
        try:
            self._setup()
        except Exception as e:
            self.direct_ok = self.graph_ok = False
            self.status['setup_error'] = repr(e)
        self.status.update(direct_ok=self.direct_ok, graph_ok=self.graph_ok, setup_s=round(time.monotonic() - t0, 3))
        _log(self.status)

    # ---------------------------------------------------------------- level 1: predictor functions called directly
    def _direct_mask_t(self, image):
        p = self.p
        p.batch = (['image0.jpg'], [image], [''])   # what the numpy loader yields for one array (only the path is read)
        im = p.preprocess([image])
        preds = p.inference(im)
        res = p.postprocess(preds, im, [image])
        return res[0].semantic_mask.data

    def _direct(self, image):
        with self.torch.inference_mode():
            m = self._direct_mask_t(image).cpu().numpy()
        return np.squeeze(m) == 1

    # ---------------------------------------------------------------- level 2: one CUDA graph for the fixed shape
    def _letterbox_geom(self):
        """where the predictor's own letterbox puts a 360x640 image: shape of the padded image and the content offset."""
        probe = np.full(SHAPE, 255, np.uint8)
        lb = self.p.pre_transform([probe])[0]
        H, W = lb.shape[:2]
        m = (lb == 255).all(axis=2)
        rows = np.flatnonzero(m.any(axis=1)); cols = np.flatnonzero(m.any(axis=0))
        top, left = int(rows[0]), int(cols[0])
        assert rows[-1] - top + 1 == SHAPE[0] and cols[-1] - left + 1 == SHAPE[1], 'letterbox resizes this shape'
        assert m[top:top + SHAPE[0], left:left + SHAPE[1]].all() and m.sum() == SHAPE[0] * SHAPE[1], 'letterbox content not 1:1'
        padv = lb[0, 0].copy() if (top > 0 or left > 0) else None
        return H, W, top, left, padv

    def _pre_gpu(self, x_u8):
        """replica of the predictor's preprocess for a 360x640 uint8 BHWC on the GPU (checked against it exactly)."""
        torch = self.torch
        H, W, top, left, padv = self.geom
        b = torch.empty((1, H, W, 3), dtype=torch.uint8, device=x_u8.device)
        if padv is not None: b[:] = self.padv_t
        b[:, top:top + SHAPE[0], left:left + SHAPE[1]] = x_u8
        im = b.permute(0, 3, 1, 2).flip(1).contiguous()
        return (im.half() if self.p.model.fp16 else im.float()).div_(255)

    def _post_gpu(self, preds, im):
        """replica of SemanticSegmentationPredictor.postprocess (logits branch) + V4's ==1, for orig shape 360x640."""
        torch = self.torch; F = torch.nn.functional
        if isinstance(preds, (tuple, list)): preds = preds[0]
        pred = preds[0][None].float()
        if pred.shape[2:] != im.shape[2:]:
            pred = F.interpolate(pred, im.shape[2:], mode='bilinear', align_corners=False)
        H, W, top, left, _ = self.geom
        if (H, W) != SHAPE[:2]:
            pred = F.interpolate(pred[..., top:top + SHAPE[0], left:left + SHAPE[1]].float(), SHAPE[:2], mode='bilinear')
        pred = pred[0]
        if pred.shape[0] > 1: cm = pred.argmax(0).to(torch.uint8)
        else: cm = pred.gt(0).squeeze(0).to(torch.uint8)
        return cm == 1

    def _graph_body(self):
        im = self._pre_gpu(self.s_in.unsqueeze(0))
        preds = self.p.inference(im)
        return self._post_gpu(preds, im)

    def _setup(self):
        torch = self.torch
        frames = [_synthetic(s) for s in (1, 2, 3)] + [np.zeros(SHAPE, np.uint8)]
        refs = [self.ref(f) for f in frames]
        self.status['check_frames'] = len(frames)
        self.status['check_boundary_px'] = [int(r.sum()) for r in refs]
        if self.want_direct:
            ok = all(np.array_equal(self._direct(f), r) for f, r in zip(frames, refs)) if self.self_check else True
            self.direct_ok = ok
            if not ok: self.status['direct_fail'] = 'mask differs from model.predict'
        if self.want_graph and self.direct_ok and self.p.device.type == 'cuda':
            dev = self.p.device
            if self.cfg.get('cudnn_benchmark'):   # optional: cuDNN picks the fastest kernels for the captured shape (self check still applies)
                torch.backends.cudnn.benchmark = True
            self.geom = self._letterbox_geom()
            self.padv_t = None if self.geom[4] is None else torch.as_tensor(self.geom[4], device=dev)
            if self.self_check:   # preprocess replica must be bit-identical to the predictor's own preprocess
                with torch.inference_mode():
                    for f in frames:
                        a = self.p.preprocess([f]); b = self._pre_gpu(torch.from_numpy(f).to(dev).unsqueeze(0))
                        if a.shape != b.shape or a.dtype != b.dtype or not torch.equal(a, b):
                            self.status['graph_fail'] = 'preprocess replica differs'; return
            with torch.inference_mode():
                self.s_in = torch.zeros(SHAPE, dtype=torch.uint8, device=dev)
                self.h_in = torch.empty(SHAPE, dtype=torch.uint8).pin_memory()
                self.s_in.copy_(torch.from_numpy(frames[0]))
                st = torch.cuda.Stream(device=dev); st.wait_stream(torch.cuda.current_stream(dev))
                with torch.cuda.stream(st):
                    for _ in range(3): self._graph_body()
                torch.cuda.current_stream(dev).wait_stream(st)
                self.g = torch.cuda.CUDAGraph()
                with torch.cuda.graph(self.g):
                    self.s_out = self._graph_body()
            with torch.inference_mode():
                self.h_out = torch.empty(tuple(self.s_out.shape), dtype=torch.bool).pin_memory()
            ok = all(np.array_equal(self._graph(f), r) for f, r in zip(frames, refs)) if self.self_check else True
            self.graph_ok = ok
            if not ok: self.status['graph_fail'] = 'mask differs from model.predict'

    def _graph(self, image):
        torch = self.torch
        np.copyto(self.h_in.numpy(), image)            # host copy into pinned buffer
        with torch.inference_mode():
            self.s_in.copy_(self.h_in, non_blocking=True)
            self.g.replay()
            self.h_out.copy_(self.s_out, non_blocking=True)
            torch.cuda.current_stream(self.p.device).synchronize()
        return self.h_out.numpy().copy()                # the buffer is reused next frame

    # ---------------------------------------------------------------- entry
    def predict(self, image):
        self.n += 1
        if self.graph_ok and image.shape == SHAPE and image.dtype == np.uint8:
            out = self._graph(np.ascontiguousarray(image))
        elif self.direct_ok:
            out = self._direct(image)
        else:
            return self.ref(image)
        if self.verify_every and self.n % self.verify_every == 0:
            r = self.ref(image)
            if not np.array_equal(out, r):
                self.graph_ok = self.direct_ok = False
                self.mismatch = dict(frame=self.n, px=int((out != r).sum()))
                _log(dict(revision=REVISION, verify_mismatch=self.mismatch, now='original path for the rest of this worker'))
                return r
        return out
