"""pathfast2-r1 (corridor-parallel-r1) count helper: started by corridor_proc.py when pathfast.json "corridor_helpers" >= 1.
Holds the distance map and running sums of the current frame (computed from the same mask with the same formulas as
CorridorTracker._step) and counts the coarse-search candidates of the vx range it is given with
CorridorTracker._counts_grid (the same function the corridor process runs). Talks only over stdin / stdout pipes.
Request 'frame': <II len(meta) len(mask)> + pickle(('frame', shape)) + np.packbits(mask)
Request 'counts': <II len(meta) 0> + pickle(('counts', g0, g1, g2, g3, rows, ys, horizon_y, width_slope))
Reply (counts only): <I len> + pickle(('ok', (nl, nr, ni)) | ('exc', text))
"""
import os
import pickle
import struct
import sys
import traceback

out_fd = sys.stdout.fileno(); sys.stdout = sys.stderr
in_fd = sys.stdin.fileno()
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cv2
import junction_corridor as JC
JC.COUNTS_REMOTE = None
try: cv2.setNumThreads(2)
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


def main():
    tr = object.__new__(JC.CorridorTracker); dist = cums = None
    wr(b'RDY1')
    while True:
        try: a, b = struct.unpack('<II', rd(8))
        except EOFError: return
        meta = pickle.loads(rd(a)); payload = rd(b) if b else b''
        if meta[0] == 'frame':
            shape = meta[1]
            mask = np.unpackbits(np.frombuffer(payload, np.uint8), count=shape[0] * shape[1]).reshape(shape).view(bool)
            # full-image versions of the two tables (CorridorTracker._step); the rows the search reads hold the same values
            # as the row-limited / cropped variants (pathfast.json cums_rows / dt_crop)
            dist = cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, 3)
            cums = np.concatenate([np.zeros((shape[0], 1), np.int32), np.cumsum(mask, 1, dtype=np.int32)], 1)
            tr.__dict__.clear()
            continue
        try:
            _, g0, g1, g2, g3, rows, ys, hy, ws = meta
            tr.entrance = dict(horizon_y=hy, width_slope=ws)
            out = ('ok', tr._counts_grid(g0, g1, g2, g3, dist, cums, rows, ys, f32=True))
        except BaseException as e:
            out = ('exc', ''.join(traceback.format_exception(type(e), e, e.__traceback__)))
        body = pickle.dumps(out, protocol=4)
        wr(struct.pack('<I', len(body)) + body)


if __name__ == '__main__':
    main()
