"""cache builder: reads the five pid.jsonl once (read-only) and writes a small pickle per run (masks / path diagnostics dropped) into the cache
folder (env L4_CACHE, default <system temp>/l4). Every other tool here reads these pickles. Run first:  python mk.py"""
import json, pickle, os, tempfile
EXP = os.environ.get('L4_EXP', r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments')
CACHE = os.environ.get('L4_CACHE', os.path.join(tempfile.gettempdir(), 'l4'))
RUNS = ['run-e1b4-20261121', 'run-65ea-20261122', 'run-3197-20261122', 'run-8350-20261122', 'run-46c9-20261122']
os.makedirs(CACHE, exist_ok=True)
for r in RUNS:
    Q = []; start = None
    for l in open(os.path.join(EXP, r, 'pid.jsonl'), encoding='utf8'):
        d = json.loads(l)
        if d.get('kind') == 'start': start = d; continue
        if d.get('kind') != 'sample': continue
        p = d['proposal']; v = d.get('vision') or {}
        pdg = v.get('path_diagnostic') or {}
        p['_v'] = {k: v.get(k) for k in ('valid', 'selected_source', 'dash_route', 'capture_at', 'consumed_at', 'frame_seq', 'error', 'control_age_s', 'downhill_release', 'flat_dip_guard')}
        p['_v']['le'] = pdg.get('line_estimate'); p['_v']['jstate'] = (pdg.get('junction') or {}).get('state')
        p['_flags'] = [k for k in d if k not in ('kind', 'wall', 'vision', 'proposal', 'pid_input_seq', 'command_dispatched', 'physical_response_verified', 'feedback_observation_version')]
        p['_wall'] = d['wall']
        Q.append(p)
    pickle.dump((start, Q), open(os.path.join(CACHE, r[4:8] + '.pkl'), 'wb'))
    print(r, len(Q))
