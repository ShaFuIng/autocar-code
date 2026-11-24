"""Windows login supervisor for the laptop website only; never starts vehicle tasks."""
import ctypes
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent

def listening():
    try:
        with socket.create_connection(('127.0.0.1', 8093), timeout=2):
            return True
    except OSError:
        return False

def ensure_running():
    if listening():
        return 'already listening'
    python = Path(sys.executable).with_name('python.exe')
    env = dict(os.environ, PYTHONUTF8='1')
    (ROOT / 'data').mkdir(exist_ok=True)
    with (ROOT / 'data' / 'autostart.log').open('ab') as log:
        subprocess.Popen([str(python), str(ROOT / 'server.py'), '--port', '8093'],
                         cwd=ROOT, env=env, stdout=log, stderr=log,
                         creationflags=subprocess.CREATE_NO_WINDOW)
    return 'started laptop website'

if __name__ == '__main__':
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.CreateMutexW.restype = ctypes.c_void_p
    kernel.CreateMutexW.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_wchar_p]
    handle = kernel.CreateMutexW(None, False, 'Local\\OrinLaptopWorkbenchSupervisor8093')
    if not handle or ctypes.get_last_error() == 183:
        sys.exit(0)
    while True:
        result = ensure_running()
        if '--once' in sys.argv:
            print(result)
            break
        time.sleep(15)
