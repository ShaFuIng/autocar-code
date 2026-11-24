"""centred-pose log (2026-11-22): read-only. The user places the car by hand (lane centre, body along the curve) and holds it still;
this logs the preview's image error (workbench proxy, GET only) and the IMU (ssh cat of the logger's latest.json) for <minutes>.
usage: python logpose.py <out.jsonl> <minutes>"""
import json, subprocess, sys, threading, time, urllib.request

OUT, MIN = sys.argv[1], float(sys.argv[2])
SSH = ['ssh', '-i', 'C:/Users/User/.ssh/codex_orin_ed25519', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'StrictHostKeyChecking=yes',
       '-o', 'HostKeyAlias=orinnano-super.local', 'jetson@192.168.0.244',
       'while true; do cat ~/orin_interface_v1/logs/imu/latest.json; echo; sleep 0.2; done']
imu = {}


def imu_reader():
    p = subprocess.Popen(SSH, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    for line in p.stdout:
        try: d = json.loads(line)
        except ValueError: continue
        imu.update(yaw=d.get('yaw_deg'), pitch=d.get('pitch_deg'), roll=d.get('roll_deg'), gyro=d.get('gyro_dps'), it=d.get('t'))


threading.Thread(target=imu_reader, daemon=True).start()
end = time.time() + MIN * 60; last = None
with open(OUT, 'a', encoding='utf8') as f:
    while time.time() < end:
        try:
            r = urllib.request.Request('http://127.0.0.1:8093/vehicle/api/pid/status', headers={'X-Workbench': '1'})
            d = json.load(urllib.request.urlopen(r, timeout=4)); l = d.get('latest') or {}
            if l.get('seq') != last:
                last = l.get('seq'); tp = l.get('turn_preview') or {}; pdg = l.get('path_diagnostic') or {}
                f.write(json.dumps(dict(t=round(time.time(), 2), seq=last, state=d.get('state'), run=(d.get('run') or {}).get('state'), valid=l.get('valid'), err=l.get('error'),
                                        near=tp.get('near_error'), far=tp.get('far_projected_error'), ctl=tp.get('control_error'), mode=pdg.get('mode'),
                                        src=(pdg.get('line_estimate') or {}).get('sources'), **imu)) + '\n'); f.flush()
        except Exception as e:
            f.write(json.dumps(dict(t=round(time.time(), 2), fail=type(e).__name__)) + '\n'); f.flush()
        time.sleep(0.2)
