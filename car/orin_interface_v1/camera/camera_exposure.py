#!/usr/bin/env python3
"""Keeps the USB camera on the exposure settings in camera_exposure.json (2026-11-17, user request).
Why: in auto exposure the camera stretches each frame to 50-60 ms in this hall's light (17-20 fps instead of 30).
Manual 30 ms + gamma 300 gives 30 fps with masks checked at 4 track spots and one lap (AGENTS.md 11-17).
The camera forgets the settings when it loses power / is re-plugged, so this re-checks every few seconds.
Only V4L2 control ioctls on the camera; no streaming, no UART, no vehicle control. enabled false = do nothing."""
import fcntl, json, os, struct, time
HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, 'camera_exposure.json')
DEV = '/dev/v4l/by-id/usb-BC-250219-ZW_USB_Camera_01.00.00-video-index0'
IDS = {'exposure_auto': 0x9a0901, 'exposure_absolute': 0x9a0902, 'gamma': 0x980910, 'brightness': 0x980900,
       'contrast': 0x980901, 'sharpness': 0x98091b, 'backlight_compensation': 0x98091c}
ORDER = ['exposure_auto', 'exposure_absolute', 'gamma', 'brightness', 'contrast', 'sharpness', 'backlight_compensation']
G_CTRL, S_CTRL = 0xC008561B, 0xC008561C

def get(fd, cid):
    b = bytearray(8); struct.pack_into('I', b, 0, cid); fcntl.ioctl(fd, G_CTRL, b); return struct.unpack_from('i', b, 4)[0]

def put(fd, cid, v):
    b = bytearray(8); struct.pack_into('Ii', b, 0, cid, int(v)); fcntl.ioctl(fd, S_CTRL, b)

def check(force=False):
    cfg = json.load(open(CFG, encoding='utf8'))
    if not cfg.get('enabled'): return None
    want = {k: int(v) for k, v in (cfg.get('controls') or {}).items() if k in IDS}
    if not os.path.exists(DEV): return None
    fd = os.open(DEV, os.O_RDWR | os.O_NONBLOCK)
    try:
        # exposure_absolute is not writable while auto is on, so compare/set auto first
        changed = {}
        keys = [k for k in ORDER if k in want]
        # The camera keeps the auto loop's real exposure time when only exposure_auto is switched back to manual
        # (controls then read 1/300 but frames still take 60 ms, seen 2026-11-17). So if anything differs, or on
        # the first pass after start, write every control again, exposure_auto first.
        if force or any(get(fd, IDS[k]) != want[k] for k in keys):
            for k in keys:
                cur = get(fd, IDS[k]); put(fd, IDS[k], want[k]); changed[k] = [cur, get(fd, IDS[k])]
        return changed
    finally:
        os.close(fd)

def main():
    last_err = None; first = True
    while True:
        try:
            ch = check(force=first)
            if ch is not None: first = False
            if ch: print(time.strftime('%Y-%m-%d %H:%M:%S'), 'applied', json.dumps(ch), flush=True)
            last_err = None
        except Exception as e:
            if str(e) != last_err: print(time.strftime('%Y-%m-%d %H:%M:%S'), 'error', repr(e), flush=True)
            last_err = str(e)
        try: period = float(json.load(open(CFG, encoding='utf8')).get('period_s', 3))
        except Exception: period = 3
        time.sleep(max(1.0, period))

if __name__ == '__main__': main()
