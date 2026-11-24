import lib
for r in lib.RUNS:
    Q = lib.load(r); print(r, len(Q), 'laps', max(p['_lap'] for p in Q))
