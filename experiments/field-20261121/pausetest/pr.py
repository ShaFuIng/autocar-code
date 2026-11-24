"""Pause / resume test helper (runs ON the Orin, talks to the local dashboard on port 8000).

usage: python3 pr.py status | pause | resume | watch [seconds] | auto <pause at active_s> [hold_s] [wait_s]
- status : one line of /api/pid/status (run, pause object, timing) + vehicle state
- pause  : POST /api/pid/pause  {"run_id": <current run>}   (throttle 0, steer to centre)
- resume : POST /api/pid/resume {"run_id": <current run>}   (the car DRIVES again)
- watch  : print the pause object every 0.5 s for N seconds (default 6)
- auto   : arm BEFORE a run: when a new run reaches active_s, pause once; after can_resume has held for hold_s, resume once (the car DRIVES again)
The login token is read from ~/.config/orin_lab/token on the Orin and is never printed.
No start / prepare / servo / motor endpoint is reachable from this tool.
"""
import base64, json, subprocess, sys, time, urllib.request, urllib.error
from pathlib import Path

ALLOWED_POST = ('/api/pid/pause', '/api/pid/resume')
ALLOWED_GET = ('/api/pid/status', '/api/status')


def call(path, body=None):
    if path not in (ALLOWED_POST if body is not None else ALLOWED_GET): raise ValueError('not allowed: ' + path)
    cred = base64.b64encode(('jetson:' + (Path.home() / '.config/orin_lab/token').read_text().strip()).encode()).decode()
    last = None
    for host in ['127.0.0.1'] + subprocess.check_output(['hostname', '-I'], text=True).split():
        if ':' in host: continue
        base = 'http://' + host + ':8000'; h = {'Authorization': 'Basic ' + cred}
        data = None
        if body is not None:
            data = json.dumps(body).encode(); h.update({'Origin': base, 'X-Orin-Control': '1', 'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(urllib.request.Request(base + path, data=data, headers=h), timeout=4) as r: return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            raw = e.read()
            try: return e.code, json.loads(raw)
            except Exception: return e.code, {'raw': raw[:120].decode('utf8', 'replace')}
        except OSError as e: last = e
    raise RuntimeError('dashboard unreachable: ' + type(last).__name__)


def brief():
    _, s = call('/api/pid/status'); _, v = call('/api/status'); v = v.get('vehicle') or {}
    run = s.get('run') or {}
    return dict(model=s.get('state'), run_id=(run.get('id') or '')[:8], run_state=run.get('state'), run_reason=run.get('reason'), seconds=run.get('seconds'),
                pause=s.get('pause'), timing=s.get('timing'), vehicle=v.get('state'), vreason=v.get('reason'),
                pwm=[(v.get('motor_a') or {}).get('pwm_command'), (v.get('motor_b') or {}).get('pwm_command')], steer=v.get('steer_us_command')), run.get('id')


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'status'
    if cmd == 'status':
        print(json.dumps(brief()[0], ensure_ascii=False))
    elif cmd in ('pause', 'resume'):
        _, rid = brief()
        t = time.monotonic(); code, r = call('/api/pid/' + cmd, {'run_id': rid}); dt = (time.monotonic() - t) * 1000
        print('%s http %s in %.0f ms: %s' % (cmd, code, dt, json.dumps(r, ensure_ascii=False)))
        time.sleep(0.6); print('after 0.6 s:', json.dumps(brief()[0], ensure_ascii=False))
    elif cmd == 'watch':
        n = float(sys.argv[2]) if len(sys.argv) > 2 else 6.
        t0 = time.monotonic()
        while time.monotonic() - t0 < n:
            b = brief()[0]; p = b.get('pause') or {}
            print('%.1f run=%s veh=%s pwm=%s steer=%s pause=%s can_resume=%s reason=%s coast_m=%s moving=%s' % (
                time.monotonic() - t0, b['run_state'], b['vehicle'], b['pwm'], b['steer'], p.get('state'), p.get('can_resume'), p.get('reason'), p.get('coast_m'), p.get('moving')))
            time.sleep(0.5)
    elif cmd == 'auto':
        # auto <pause at active_s> [hold seconds after can_resume, default 5] [wait-for-run timeout, default 180]
        t_pause = float(sys.argv[2]); hold = float(sys.argv[3]) if len(sys.argv) > 3 else 5.; wait = float(sys.argv[4]) if len(sys.argv) > 4 else 180.
        b0, rid0 = brief()
        if b0['run_state'] == 'running': print('a run is already running; start auto BEFORE the run'); return
        print('armed: waiting up to %.0f s for a NEW run; pause at active_s >= %.1f, resume %.1f s after can_resume' % (wait, t_pause, hold)); sys.stdout.flush()
        t0 = time.monotonic(); rid = None
        while time.monotonic() - t0 < wait:
            b, r = brief()
            if b['run_state'] == 'running' and r and r != rid0: rid = r; break
            time.sleep(0.1)
        if rid is None: print('no new run started; nothing sent'); return
        print('run %s started' % rid[:8]); sys.stdout.flush()
        while True:
            b, r = brief()
            if r != rid or b['run_state'] != 'running': print('run ended before the pause point (%s); nothing sent' % b['run_reason']); return
            if ((b.get('timing') or {}).get('active_s') or 0.) >= t_pause: break
            time.sleep(0.05)
        code, r = call('/api/pid/pause', {'run_id': rid}); print('pause http %s at active_s %s: %s' % (code, (b.get('timing') or {}).get('active_s'), json.dumps(r, ensure_ascii=False)[:300])); sys.stdout.flush()
        if code != 200: return
        ok_at = None; t1 = time.monotonic()
        while time.monotonic() - t1 < 30.:
            b, r2 = brief(); p = b.get('pause') or {}
            if r2 != rid or b['run_state'] != 'running': print('run ended while paused (%s); no resume sent' % b['run_reason']); return
            print('  +%.1f can_resume=%s reason=%s coast_m=%s coast_yaw=%s moving=%s' % (time.monotonic() - t1, p.get('can_resume'), p.get('reason'), p.get('coast_m'), p.get('coast_yaw_deg'), p.get('moving'))); sys.stdout.flush()
            if p.get('can_resume'):
                if ok_at is None: ok_at = time.monotonic()
                if time.monotonic() - ok_at >= hold: break
            else: ok_at = None
            time.sleep(0.5)
        else:
            print('never became resumable for %.0f s in a row within 30 s; NO resume sent (run stays paused; stop it by hand)' % hold); return
        code, r = call('/api/pid/resume', {'run_id': rid}); print('resume http %s: %s' % (code, json.dumps(r, ensure_ascii=False)[:300])); sys.stdout.flush()
        t2 = time.monotonic()
        while time.monotonic() - t2 < 4.:
            b, _ = brief(); p = b.get('pause') or {}
            print('  +%.1f run=%s reason=%s veh=%s pwm=%s steer=%s pause=%s' % (time.monotonic() - t2, b['run_state'], b['run_reason'], b['vehicle'], b['pwm'], b['steer'], p.get('state'))); sys.stdout.flush()
            time.sleep(0.4)
    else:
        print(__doc__)


if __name__ == '__main__':
    main()
