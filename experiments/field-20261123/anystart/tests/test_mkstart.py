"""python tests/test_mkstart.py <base folder>: D=3.44 byte-identical; D=6 changes only the documented keys."""
import sys, subprocess, tempfile, hashlib
from pathlib import Path
T = Path(__file__).resolve().parents[1] / 'tools' / 'mkstart.py'; B = Path(sys.argv[1])
h = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
with tempfile.TemporaryDirectory() as d:
    subprocess.run([sys.executable, str(T), '--dist', '3.44', '--base', str(B), '--out', d], check=True, capture_output=True)
    for n in ('lap_pos.json', 'brake_zones.json', 'lap_ff.json', 'start_bias.json'): assert h(B / n) == h(Path(d) / n), n
    out = subprocess.run([sys.executable, str(T), '--dist', '6', '--base', str(B), '--out', d], check=True, capture_output=True, text=True, encoding='utf8').stdout
    keys = [l.split()[1].rstrip(':') for l in out.splitlines() if l.strip().startswith(('~', '+', '-'))]
    ok = all(k == 'lap_pos.json.start_s' or k.endswith(('.from_m', '.to_m')) or k == 'lap_ff.json.curves.c1.start.gap_m' for k in keys)
    assert ok, keys
print('OK')
