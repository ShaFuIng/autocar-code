"""pathfast2-r1 (corridor-parallel-r1): run the junction corridor (CorridorTracker.step) at the same time as the baseline
line estimate inside JunctionLine.step. The two do not depend on each other within a frame (the corridor reads only the
640x360 mask and its own past state; the baseline reads the mask and its own state); the switch combines both afterwards.
Output is unchanged: the same CorridorTracker code runs on the same mask with the same state, only on another core.

Modes (pathfast.json next to this file, key "corridor_parallel"; read at import = at each model load):
  absent / null / "off" : JunctionLine uses the plain CorridorTracker (code path identical to pathfast-r1)
  "thread"  : the corridor step runs in one helper thread of the worker process (numpy / OpenCV release the GIL only inside
              their loops, so the Python parts of both stages still take turns)
  "process" : the corridor runs in a separate Python process (corridor_proc.py, started at import, numpy + OpenCV only,
              no torch / CUDA). Per frame the packed mask (np.packbits, 28.8 kB) goes down a pipe, the corridor result
              and the tracker state (a few kB, pickle) come back.
State and failures (process mode): the parent keeps an exact mirror of the tracker state (every reply carries the state
after the step). Before the child is ready, after a child failure (crash, broken pipe, no reply within corridor_timeout_s)
or for the first frame after a reset, the step runs in-process on the mirror (exactly the same computation) and the full
state is sent with the next request. An exception raised by CorridorTracker.step in the child is raised again in the parent
(same type when it can be pickled) with the tracker state the child had after it, as in-process. If the baseline estimate
raises, the corridor step of that frame is undone (state restored to the frame start) before the exception propagates, so
the corridor state matches the sequential code where the corridor is never reached.
No vehicle I/O, no change to any algorithm, threshold or output field. Diagnostics (timings, mode) stay in attributes and are
never written into the result.
"""
import json
import os
import pickle
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

REVISION = 'corridor-parallel-r1'
HERE = Path(__file__).resolve().parent
MODE = None
TIMEOUT_S = 2.0
try:
    _cfg = json.loads(HERE.joinpath('pathfast.json').read_text(encoding='utf8'))
    MODE = _cfg.get('corridor_parallel')
    TIMEOUT_S = float(_cfg.get('corridor_timeout_s', TIMEOUT_S))
except (OSError, ValueError, TypeError, AttributeError):
    pass
if MODE in ('off', '', False): MODE = None
if MODE not in (None, 'thread', 'process'): MODE = None
_SKIP = ('_grid_tables',)   # per-frame cache keyed by array identity; rebuilt on the next frame


def get_state(tr):
    return {k: v for k, v in tr.__dict__.items() if k not in _SKIP}


def set_state(tr, st):
    tr.__dict__.clear(); tr.__dict__.update(st)


def snapshot(tr):
    """deep enough copy of the tracker state for an undo: ndarrays are updated in place by the tracker (velocity *= .5),
    everything else is replaced, never mutated."""
    return {k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in tr.__dict__.items() if k not in _SKIP}


def _log(msg):
    try: print('corridor-parallel: ' + msg, file=sys.stderr, flush=True)
    except Exception: pass


# ------------------------------------------------------------------------------------------------ child process handle
class _Child:
    def __init__(self):
        self.p = None; self.ready = threading.Event(); self.dead = False; self.owner = None
        try:
            env = dict(os.environ)
            self.p = subprocess.Popen([sys.executable, '-u', str(HERE / 'corridor_proc.py')], stdin=subprocess.PIPE,
                                      stdout=subprocess.PIPE, stderr=None, cwd=str(HERE), env=env, bufsize=0)
            self.fi = self.p.stdin.fileno(); self.fo = self.p.stdout.fileno()
            threading.Thread(target=self._wait_ready, daemon=True, name='corridor-proc-ready').start()
        except Exception as e:
            self.dead = True; _log('start failed %s' % type(e).__name__)

    def _wait_ready(self):
        try:
            b = self._read(4, None)
            if b == b'RDY1': self.ready.set()
            else: self.kill('bad ready %r' % (b,))
        except Exception as e:
            self.kill('ready %s' % type(e).__name__)

    def _read(self, n, deadline):
        buf = bytearray()
        while len(buf) < n:
            if deadline is not None and os.name == 'posix':
                import select
                left = deadline - time.monotonic()
                if left <= 0 or not select.select([self.fo], [], [], left)[0]: raise TimeoutError('corridor process reply')
            b = os.read(self.fo, n - len(buf))
            if not b: raise EOFError('corridor process closed')
            buf += b
        return bytes(buf)

    def send(self, meta, packed):
        mb = pickle.dumps(meta, protocol=4)
        data = struct.pack('<II', len(mb), len(packed)) + mb + packed
        view = memoryview(data)
        while view:
            n = os.write(self.fi, view); view = view[n:]

    def recv(self, timeout):
        dl = None if timeout is None else time.monotonic() + timeout
        n = struct.unpack('<I', self._read(4, dl))[0]
        return pickle.loads(self._read(n, dl))

    def usable(self):
        return not self.dead and self.p is not None and self.ready.is_set() and self.p.poll() is None

    def kill(self, why):
        if not self.dead: _log('child stopped (%s); corridor continues in-process' % why)
        self.dead = True
        try:
            if self.p is not None: self.p.kill()
        except Exception:
            pass


_CHILD = [None]
_TPOOL = []


def _child():
    """one child per worker process, started once; after a failure it is not restarted (in-process until the next model load)."""
    if _CHILD[0] is None: _CHILD[0] = _Child()
    return _CHILD[0]


if MODE == 'process':   # start at import (model load): numpy / cv2 import in the child overlaps the model warm-up
    _child()


# ------------------------------------------------------------------------------------------------ per-frame jobs
class _Done:
    """in-process step done at submit time (child not ready / lost); abandon() undoes it like the other jobs."""
    def __init__(self, pc, m, t):
        self.pc = pc; self.snap = snapshot(pc.local); self.value = self.exc = None
        try: self.value = pc.local.step(m, t)
        except BaseException as e: self.exc = e

    def result(self):
        if self.exc is not None: raise self.exc
        return self.value

    def abandon(self): set_state(self.pc.local, self.snap)


class _ThreadJob:
    def __init__(self, pc, m, t):
        self.pc = pc; self.snap = snapshot(pc.local); self.ev = threading.Event(); self.value = self.exc = None
        self.t0 = time.perf_counter()
        pc._pool().submit(self._run, m, t)

    def _run(self, m, t):
        try: self.value = self.pc.local.step(m, t)
        except BaseException as e: self.exc = e
        self.pc.last_ms = 1e3 * (time.perf_counter() - self.t0); self.ev.set()

    def result(self):
        self.ev.wait()
        if self.exc is not None: raise self.exc
        return self.value

    def abandon(self):
        self.ev.wait(); set_state(self.pc.local, self.snap)


class _ProcJob:
    def __init__(self, pc, m, t, ch):
        self.pc = pc; self.m = m; self.t = t; self.ch = ch; self.pre = snapshot(pc.local); self.sent = False
        meta = ('step', float(t), m.shape, None if not pc.sync and ch.owner == id(pc) else get_state(pc.local))
        try:
            ch.send(meta, np.packbits(m).tobytes()); self.sent = True
            ch.owner = id(pc); pc.sync = False
        except Exception as e:
            ch.kill('send %s' % type(e).__name__)

    def _reply(self):
        if not self.sent: raise EOFError('not sent')
        return self.ch.recv(TIMEOUT_S)

    def result(self):
        pc = self.pc
        try:
            r = self._reply()
        except Exception as e:   # child lost: this frame in-process from the frame-start mirror (exact), later frames too
            self.ch.kill('reply %s' % type(e).__name__); pc.sync = True
            set_state(pc.local, self.pre)
            return pc.local.step(self.m, self.t)
        status, payload, state, ms = r
        set_state(pc.local, state); pc.last_ms = ms
        if status == 'ok': return payload
        pc.sync = True   # child state after an exception is mirrored; resend it (owner check alone is not enough)
        exc, text = payload
        raise exc if isinstance(exc, BaseException) else RuntimeError('corridor process: ' + text)

    def abandon(self):
        try: self._reply()
        except Exception as e: self.ch.kill('reply %s' % type(e).__name__)
        set_state(self.pc.local, self.pre); self.pc.sync = True


class ParallelCorridor:
    """wraps one CorridorTracker; attribute reads fall through to the (mirrored) tracker, so JunctionLine's diagnostics
    (cal_rejects ...) read the same values as before."""
    parallel = True

    def __init__(self, tracker, mode):
        self.__dict__.update(local=tracker, mode=mode, sync=True, last_ms=None, inproc=0)

    def __getattr__(self, name):
        return getattr(self.__dict__['local'], name)

    def _pool(self):   # one helper thread per worker process (trackers are re-created at each route reset)
        if not _TPOOL:
            from concurrent.futures import ThreadPoolExecutor
            _TPOOL.append(ThreadPoolExecutor(1, thread_name_prefix='corridor-step'))
        return _TPOOL[0]

    def step(self, m, t):   # sequential use (same as the plain tracker)
        return self.submit(m, t).result()

    def submit(self, m, t):
        if self.mode == 'thread': return _ThreadJob(self, m, t)
        ch = _child()
        if ch.usable(): return _ProcJob(self, m, t, ch)
        self.__dict__['inproc'] += 1; self.__dict__['sync'] = True; self.__dict__['last_ms'] = None
        return _Done(self, m, t)

    def __setattr__(self, k, v):
        if k in ('sync', 'last_ms'): self.__dict__[k] = v
        else: setattr(self.local, k, v); self.__dict__['sync'] = True


def wrap(tracker):
    return tracker if MODE is None else ParallelCorridor(tracker, MODE)
