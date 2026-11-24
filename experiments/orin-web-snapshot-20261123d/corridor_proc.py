"""pathfast2-r1 (corridor-parallel-r1) child process: runs junction_corridor.CorridorTracker.step for the worker.
Started by corridor_parallel.py (mode "process"); talks only over its stdin / stdout pipes; imports numpy, OpenCV and
junction_corridor from this folder (no torch, no CUDA, no network, no vehicle device). Exits on EOF (worker gone).
Request : <II len(meta) len(mask)> + pickle(('step', t, shape, state or None)) + np.packbits(mask)
Reply   : <I len> + pickle((status 'ok'|'exc', result | (exception, text), tracker state after the step, step ms))
"""
import os
import pickle
import struct
import sys
import time
import traceback

out_fd = sys.stdout.fileno(); sys.stdout = sys.stderr   # nothing else may write to the reply pipe
in_fd = sys.stdin.fileno()
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cv2
import junction_corridor as JC   # NOT corridor_parallel (it would start another child)
_SKIP = ('_grid_tables',)         # same as corridor_parallel.get_state / set_state
try: cv2.setNumThreads(2)   # same as live_model_worker (the corridor's OpenCV calls give the same values for any count)
except Exception: pass


def rd(n):
    buf = bytearray()
    while len(buf) < n:
        b = os.read(in_fd, n - len(buf))
        if not b: raise EOFError
        buf += b
    return bytes(buf)


def wr(b):
    v = memoryview(b)
    while v:
        v = v[os.write(out_fd, v):]


class Helpers:
    """JC.COUNTS_REMOTE for this process: the coarse-pass vx range [hs, n0) is split over the helper processes
    (corridor_helper.py). A helper that fails (exception, EOF, no reply in TIMEOUT_S) is dropped and its part is counted here
    with the same function, so the counts never depend on the helpers."""
    TIMEOUT_S = 2.0

    def __init__(self, n):
        import subprocess
        self.h = []
        for _ in range(n):
            try:
                p = subprocess.Popen([sys.executable, '-u', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'corridor_helper.py')],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None, bufsize=0)
                self.h.append([p, p.stdin.fileno(), p.stdout.fileno(), True])
            except Exception as e:
                print('corridor-proc: helper start failed %s' % type(e).__name__, file=sys.stderr, flush=True)
        for x in self.h:
            try:
                if self._rd(x, 4) != b'RDY1': self._drop(x, 'ready')
            except Exception as e: self._drop(x, 'ready %s' % type(e).__name__)

    def live(self): return [x for x in self.h if x[3]]

    def _drop(self, x, why):
        if x[3]: print('corridor-proc: helper dropped (%s); its counts are done here' % why, file=sys.stderr, flush=True)
        x[3] = False
        try: x[0].kill()
        except Exception: pass

    def _rd(self, x, n, timeout=None):
        import time as _t
        dl = None if timeout is None else _t.monotonic() + timeout; buf = bytearray()
        while len(buf) < n:
            if dl is not None and os.name == 'posix':
                import select
                left = dl - _t.monotonic()
                if left <= 0 or not select.select([x[2]], [], [], left)[0]: raise TimeoutError
            b = os.read(x[2], n - len(buf))
            if not b: raise EOFError
            buf += b
        return bytes(buf)

    def _wr(self, x, b):
        v = memoryview(b)
        while v: v = v[os.write(x[1], v):]

    def frame(self, shape, packed):
        hd = pickle.dumps(('frame', shape), protocol=4)
        for x in self.live():
            try: self._wr(x, struct.pack('<II', len(hd), len(packed)) + hd + packed)
            except Exception as e: self._drop(x, 'send %s' % type(e).__name__)

    def split(self, n0):
        return -(-n0 // (len(self.live()) + 1))

    def send(self, tr, g0r, g1, g2, g3, rows, ys):
        hs = None; lv = self.live(); tok = []; n = len(g0r); P = max(1, len(lv))
        cut = [round(i * n / P) for i in range(P + 1)]
        for i in range(P):
            a, b = cut[i], cut[i + 1]
            if a >= b: continue
            x = lv[i] if i < len(lv) else None
            if x is not None:
                try:
                    hd = pickle.dumps(('counts', g0r[a:b], g1, g2, g3, rows, ys, tr.entrance['horizon_y'], tr.entrance['width_slope']), protocol=4)
                    self._wr(x, struct.pack('<II', len(hd), 0) + hd)
                except Exception as e:
                    self._drop(x, 'send %s' % type(e).__name__); x = None
            tok.append((x, a, b))
        return tok

    def recv(self, tok, local, hs):
        res = []
        for x, a, b in tok:
            r = None
            if x is not None:
                try:
                    n = struct.unpack('<I', self._rd(x, 4, self.TIMEOUT_S))[0]; r = pickle.loads(self._rd(x, n, self.TIMEOUT_S))
                    if r[0] != 'ok': self._drop(x, 'exception'); r = None
                except Exception as e:
                    self._drop(x, 'reply %s' % type(e).__name__); r = None
            if r is not None: res.append(r[1])
            else: res += local(hs + a, hs + b)
        return res


def main():
    tr = object.__new__(JC.CorridorTracker)
    nh = 0
    try:
        import json
        nh = int(json.loads(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pathfast.json'), encoding='utf8').read()).get('corridor_helpers', 0) or 0)
    except Exception:
        nh = 0
    helpers = Helpers(nh) if nh > 0 else None
    if helpers is not None and helpers.live(): JC.COUNTS_REMOTE = helpers
    wr(b'RDY1')
    while True:
        try: a, b = struct.unpack('<II', rd(8))
        except EOFError: return
        kind, t, shape, state = pickle.loads(rd(a)); packed = np.frombuffer(rd(b), np.uint8)
        if kind != 'step': continue
        if JC.COUNTS_REMOTE is not None:
            if helpers.live(): helpers.frame(shape, packed.tobytes())
            else: JC.COUNTS_REMOTE = None
        if state is not None: tr.__dict__.clear(); tr.__dict__.update(state)
        mask = np.unpackbits(packed, count=shape[0] * shape[1]).reshape(shape).view(bool)
        t0 = time.perf_counter()
        try:
            res = ('ok', tr.step(mask, t))
        except BaseException as e:
            text = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
            print('corridor-proc step exception:\n' + text, file=sys.stderr, flush=True)
            try: pickle.dumps(e); ex = e
            except Exception: ex = None
            res = ('exc', (ex, text))
        ms = 1e3 * (time.perf_counter() - t0)
        try: body = pickle.dumps(res + ({k: v for k, v in tr.__dict__.items() if k not in _SKIP}, ms), protocol=4)
        except Exception as e:   # unpicklable result: report as an exception (never seen; keeps the pipe in step)
            body = pickle.dumps(('exc', (None, 'reply pickle %r' % (e,)), {k: v for k, v in tr.__dict__.items() if k not in _SKIP}, ms), protocol=4)
        wr(struct.pack('<I', len(body)) + body)


if __name__ == '__main__':
    main()
