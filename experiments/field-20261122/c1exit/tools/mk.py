import sys, lib
for r in (sys.argv[1:] or lib.MAIN + lib.MORE):
    Q = lib.load(r); print(r, len(Q), 'laps', max(p['_lap'] for p in Q), flush=True)
