"""Temporary system sleep inhibition while this experiment is progressing."""
import ctypes,json,time
from pathlib import Path
p=Path(__file__).resolve().parent/'data/experiments/night-20261018/report.json'
start=time.time()
try:
    while time.time()-start<4*3600:
        try:
            r=json.loads(p.read_text(encoding='utf-8'))
            if r['state']!='running' or time.time()-p.stat().st_mtime>1800:break
        except (OSError,ValueError):break
        # System only: display may turn off normally. No persistent power changes.
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)
        time.sleep(30)
finally:
    ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)
