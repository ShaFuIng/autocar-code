"""Authenticated dashboard with physically authorized servo calibration only."""
import json
import base64
import secrets
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import threading
import queue
import binascii
import time
import tempfile
import sys
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from motor_tests import MotorTestManager
from manual_drive import ManualDriveManager,SessionRecorder
from record_only import RecordOnly
from control_ws import serve_control
BENCH_FIRMWARE=("orin-v1.3.0-bench","orin-v1.4.1-manual")
manual_drive=None
recorder=None
lock = threading.Lock()
data = {"uart_error": "connecting", "camera_error": "connecting", "vehicle": None,
        "uart_at": 0, "camera_at": 0, "bad_startup": 0, "bad_steady": 0}
jpeg = None
stop = threading.Event()
camera_changed = threading.Condition(lock)
stream_slots = threading.BoundedSemaphore(3)


PROFILE_PATH = Path.home()/'.config/orin_car/servo_profile.json'
profile_lock = threading.RLock()
control_operation_lock=threading.RLock()
saved_profile = {'revision':1,'min_us':1010,'center_us':1460,'max_us':1810}
profile_loaded = False

def validate_profile(profile):
    if not isinstance(profile,dict): raise ValueError('Expected profile object')
    for key in ('min_us','center_us','max_us','revision'):
        if type(profile.get(key)) is not int: raise ValueError('Profile values must be integers')
    if not 500 <= profile['min_us'] < profile['center_us'] < profile['max_us'] <= 2500:
        raise ValueError('Require 500 <= left minimum < center < right maximum <= 2500 us')
    if not 0 < profile['revision'] <= 0xffffffff: raise ValueError('Invalid profile revision')
    return {key:profile[key] for key in ('revision','min_us','center_us','max_us')}

def write_profile(profile,path=None):
    path=Path(path or PROFILE_PATH)
    profile=validate_profile(profile)
    path.parent.mkdir(parents=True,exist_ok=True)
    temp_name=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=path.parent,prefix='.servo-',delete=False) as f:
            temp_name=f.name
            json.dump(profile,f,ensure_ascii=False,indent=2)
            f.write('\n');f.flush();os.fsync(f.fileno())
        os.replace(temp_name,path)
    finally:
        if temp_name and os.path.exists(temp_name): os.unlink(temp_name)

def load_profile(path=None):
    global saved_profile,profile_loaded
    path=Path(path or PROFILE_PATH)
    with profile_lock:
        if path.exists(): profile=validate_profile(json.loads(path.read_text(encoding='utf-8')))
        else:
            profile=dict(saved_profile)
            write_profile(profile,path)
        saved_profile=profile
        profile_loaded=True
    return dict(profile)

def profile_matches(vehicle,profile=None):
    if profile is None:
        with profile_lock:
            if not profile_loaded: return False
            profile=dict(saved_profile)
    actual=vehicle.get('servo_profile') or {}
    return bool(actual.get('synced') and all(actual.get(k)==profile[k] for k in ('revision','min_us','center_us','max_us')))

def save_profile(payload):
    with control_operation_lock:
        return _save_profile(payload)

def _save_profile(payload):
    if motor_tests and motor_tests.busy(): raise ValueError('Stop motor test before changing settings')
    if manual_drive and manual_drive.busy(): raise ValueError('Use manual driving trim settings while driving')
    global saved_profile,profile_loaded
    with profile_lock:
        if type(payload.get('expected_revision')) is not int or payload.get('expected_revision') != saved_profile['revision']:
            raise ValueError('Settings changed; reload settings before saving')
        profile=validate_profile(dict(payload,revision=saved_profile['revision']+1))
        with lock:
            v=dict(data['vehicle'] or {})
            if v.get('state')=='MOTOR_TEST': raise ValueError('Stop motor test before changing settings')
            if v.get('servo_returning'): raise ValueError('Wait for center return or stop first')
            if v.get('steer_output_enabled') and not profile['min_us'] <= v.get('steer_us_command',0) <= profile['max_us']:
                raise ValueError('New limits exclude current command; adjust position first')
        write_profile(profile)
        saved_profile=profile
        profile_loaded=True
    with lock: data['profile_sync_error']=None
    return {'ok':True,'saved':True,'profile':dict(profile),'synced':False,'message':'Saved without movement; awaiting BMduino synchronization'}

commands = queue.Queue(maxsize=8)

def frame(seq, command):
    body = f"{seq},{command}".encode('ascii')
    crc = binascii.crc_hqx(body, 0xffff)
    return b'$' + body + f'*{crc:04X}\n'.encode('ascii')

def validate_control(payload):
    with lock:
        v = dict(data['vehicle'] or {})
        age = time.monotonic() - data['uart_at']
        error = data['uart_error']
    if age > 1 or error:
        raise ValueError('UART status unavailable')
    if payload.get('action') == 'stop':
        return 'STOP'
    if motor_tests and motor_tests.busy(): raise ValueError('Motor test is active; only stop is allowed')
    if manual_drive and manual_drive.busy(): raise ValueError('Manual driving is active; only stop is allowed')
    if not v.get('servo_calibration') or v.get('fw') not in BENCH_FIRMWARE:
        raise ValueError('Flash orin-v1.3.0-bench first')
    if payload.get('action') == 'enable':
        if v.get('web_calibration_start') is not True or v.get('state')!='LOCKED' or v.get('pins',{}).get('D11')!=1:
            raise ValueError('Web calibration unavailable: check ESTOP/state/firmware')
        if not profile_matches(v):raise ValueError('Saved profile is not synchronized')
        gen=payload.get('generation')
        if type(gen) is not int or gen!=v.get('generation'):raise ValueError('Authorization changed')
        return f'CAL_ENABLE,{gen}'
    if v.get('state') != 'READY' or v.get('pins', {}).get('D11') != 1:
        raise ValueError('Enable calibration on the web page; check ESTOP and state')
    if not profile_matches(v): raise ValueError('Saved profile is not synchronized to BMduino')
    if payload.get('action') not in (None,'step','center'):
        raise ValueError('Unknown calibration action')
    gen, pulse = payload.get('generation'), payload.get('us')
    if payload.get('action') == 'center':
        if type(gen) is not int or gen != v.get('generation'): raise ValueError('Authorization changed')
        if not v.get('steer_output_enabled'): raise ValueError('Send an explicit first test pulse first')
        return f'CENTER,{gen}'
    if v.get('servo_returning'): raise ValueError('Returning to center; wait or stop first')
    if payload.get('action') == 'step':
        delta=payload.get('delta')
        if type(delta) is not int or delta == 0 or abs(delta) > 100:
            raise ValueError('Step must be a nonzero integer up to 100 us')
        if not v.get('steer_output_enabled'):
            raise ValueError('Send an explicit first test pulse first')
        pulse=v.get('steer_us_command',0)+delta
    if type(gen) is not int or gen != v.get('generation'):
        raise ValueError('Authorization changed; refresh status')
    if type(pulse) is not int or not 0 < pulse <= 65535:
        raise ValueError('Pulse must be a positive 16-bit integer')
    current = v.get('steer_us_command', 0)
    profile=v['servo_profile']
    if not profile['min_us'] <= pulse <= profile['max_us']: raise ValueError('Requested position exceeds saved limits')
    if v.get('steer_output_enabled') and abs(pulse-current) > 100:
        raise ValueError('Maximum change per request is 100 us')
    return f'SERVO,{gen},{pulse}'

motor_tests=None

def motor_state():
    with lock:
        result=json.loads(json.dumps(data))
    result['uart_age_s']=time.monotonic()-result.pop('uart_at')
    result['camera_age_s']=time.monotonic()-result.pop('camera_at',0)
    result['servo_settings_synced']=profile_matches(result.get('vehicle') or {})
    return result

def validate_motor_control(payload):
    if payload.get('action')=='stop':return 'STOP'
    d=motor_state();v=d.get('vehicle') or {}
    if d.get('uart_error') or d['uart_age_s']>1:raise ValueError('UART unavailable')
    if v.get('fw') not in BENCH_FIRMWARE or not d['servo_settings_synced']:raise ValueError('Bench firmware/profile required')
    if v.get('pins',{}).get('D11')!=1:raise ValueError('ESTOP_ACTIVE')
    gen=payload.get('generation')
    if type(gen) is not int or gen!=v.get('generation'):raise ValueError('STALE_GENERATION')
    action=payload.get('motor_action')
    if action=='arm':
        duration=payload.get('duration_ms')
        if (v.get('state')!='READY' and not (v.get('web_timed_test_start') is True and v.get('state')=='LOCKED')) or v.get('servo_returning'):raise ValueError('START_AUTHORIZATION_OR_STEERING_BUSY')
        if type(duration) is not int or not 500<=duration<=30000:raise ValueError('BAD_DURATION')
        return f'TEST_ARM,{gen},{duration}'
    if v.get('state')!='MOTOR_TEST':raise ValueError('TEST_NOT_ARMED')
    if action=='end':return f'TEST_END,{gen}'
    if action=='move':
        right,left=payload.get('right'),payload.get('left')
        if any(type(x) is not int or not -25<=x<=25 for x in (right,left)):raise ValueError('PWM_LIMIT_25')
        return f'TEST,{gen},{right},{left}'
    raise ValueError('Unknown motor command')

def manual_profile():
    with profile_lock:return dict(saved_profile)

def save_manual_center(center):
    global saved_profile
    with profile_lock:
        desired=validate_profile(dict(saved_profile,center_us=center,revision=saved_profile['revision']+1))
        if center!=saved_profile['center_us']:
            write_profile(desired);saved_profile=desired
    return manual_profile()

def validate_manual_control(payload):
    if payload.get('action')=='stop':return 'STOP'
    d=motor_state();v=d.get('vehicle') or {}
    if d.get('uart_error') or d['uart_age_s']>1:raise ValueError('UART unavailable')
    if v.get('fw')!='orin-v1.4.1-manual':raise ValueError('Manual firmware required')
    gen=payload.get('generation')
    if type(gen) is not int or gen!=v.get('generation'):raise ValueError('STALE_GENERATION')
    if v.get('pins',{}).get('D11')!=1:raise ValueError('ESTOP_ACTIVE')
    if payload.get('manual_action')=='arm':
        if (v.get('state')!='READY' and not (v.get('web_manual_start') is True and v.get('state')=='LOCKED')) or not profile_matches(v):raise ValueError('Start authorization and synchronized profile required')
        return f'MANUAL_ARM,{gen}'
    if v.get('state')!='MANUAL':raise ValueError('MANUAL_NOT_ARMED')
    if payload.get('manual_action')!='move':raise ValueError('Unknown manual action')
    right,left,pulse=payload.get('right'),payload.get('left'),payload.get('us')
    if any(type(x) is not int or not -25<=x<=25 for x in (right,left)):raise ValueError('PWM_LIMIT_25')
    actual=v.get('servo_profile') or {}
    if not actual.get('synced') or type(pulse) is not int or not actual['min_us']<=pulse<=actual['max_us']:raise ValueError('SERVO_LIMIT')
    return f'MANUAL,{gen},{right},{left},{pulse}'

def submit_control(payload,motor=False,manual=False):
    command = validate_manual_control(payload) if manual else validate_motor_control(payload) if motor else validate_control(payload)
    job = {'command':command, 'event':threading.Event(), 'deadline':time.monotonic()+2,
           'payload':payload, 'result':None, 'cancelled':False,'motor':motor,'manual':manual}
    commands.put_nowait(job)
    if not job['event'].wait(2.5):
        job['cancelled'] = True
        raise ValueError('No firmware acknowledgement; movement not confirmed')
    result = job['result']
    if not result or not result.get('ok'):
        raise ValueError((result or {}).get('reason','UART disconnected'))
    return result

def uart_worker():
    import serial
    logger = logging.getLogger('uart')
    logger.setLevel(logging.INFO)
    logdir = Path.home() / 'orin_interface_v1' / 'logs'
    logdir.mkdir(exist_ok=True)
    logger.addHandler(RotatingFileHandler(logdir/'web_uart.jsonl',maxBytes=2000000,backupCount=3))
    seq = 0
    while not stop.is_set():
        pending = None
        try:
            with serial.Serial('/dev/ttyTHS1',115200,timeout=.05,write_timeout=.2,exclusive=True) as port:
                opened = time.monotonic()
                buf = bytearray()
                synced = False
                sync_sent = False
                previous_ms = None
                received_at = opened
                profile_retry_at = 0
                while not stop.is_set():
                    now = time.monotonic()
                    if pending and now > pending['until']:
                        # Best-effort STOP, never replay an unacknowledged servo setting.
                        seq += 1
                        try: port.write(frame(seq,'STOP'))
                        except (OSError,serial.SerialException): pass
                        raise RuntimeError('UART command ACK timeout')
                    if now-opened > 3 and now-received_at > 2:
                        raise RuntimeError('UART status timeout')
                    if not pending and now-opened >= 2:
                        if not sync_sent:
                            # Never synchronize an actively driving firmware.
                            with lock:
                                v = data['vehicle'] or {}
                                can_sync = now-data['uart_at'] < 1 and v.get('servo_calibration') and v.get('state') in ('LOCKED','READY')
                            if can_sync:
                                seq += 1
                                port.write(frame(seq,'SYNC'))
                                pending = {'seq':seq,'job':None,'kind':'sync','until':now+1.5}
                                sync_sent = True
                        elif synced:
                            with profile_lock:
                                desired=dict(saved_profile)
                                loaded=profile_loaded
                            with lock:
                                v=dict(data['vehicle'] or {})
                                fresh=now-data['uart_at'] < 1
                            if loaded and fresh and v.get('fw') in BENCH_FIRMWARE and v.get('state') in ('LOCKED','READY') and not profile_matches(v,desired) and now >= profile_retry_at:
                                seq+=1
                                command=f"PROFILE,{v['generation']},{desired['revision']},{desired['min_us']},{desired['center_us']},{desired['max_us']}"
                                port.write(frame(seq,command))
                                pending={'seq':seq,'job':None,'kind':'profile','profile':desired,'until':now+1.5}
                                profile_retry_at=now+2
                                logger.info(json.dumps({'time':time.time(),'tx':command,'seq':seq}))
                                continue
                            try: job = commands.get_nowait()
                            except queue.Empty: job = None
                            if job:
                                if job['cancelled'] or now > job['deadline']:
                                    job['result']={'ok':False,'reason':'Expired request'}
                                    job['event'].set()
                                else:
                                    try:
                                        command=validate_manual_control(job['payload']) if job.get('manual') else validate_motor_control(job['payload']) if job.get('motor') else validate_control(job['payload'])
                                        seq += 1
                                        job['command']=command
                                        port.write(frame(seq,command))
                                        pending={'seq':seq,'job':job,'until':now+1.5}
                                        logger.info(json.dumps({'time':time.time(),'tx':command,'seq':seq}))
                                    except (ValueError,OSError) as exc:
                                        job['result']={'ok':False,'reason':str(exc)}
                                        job['event'].set()
                    buf.extend(port.read(min(max(port.in_waiting,1),8192)))
                    while b'\n' in buf:
                        raw, _, tail = buf.partition(b'\n')
                        buf = bytearray(tail)
                        try:
                            msg=json.loads(raw)
                            if isinstance(msg,dict) and msg.get('type')=='status':
                                received_at=time.monotonic()
                                uptime=msg.get('ms')
                                if isinstance(uptime,int):
                                    if previous_ms is not None and uptime < previous_ms:
                                        synced=False
                                        sync_sent=False
                                        if pending and pending['job']:
                                            pending['job']['result']={'ok':False,'reason':'Firmware restarted'}
                                            pending['job']['event'].set()
                                        pending=None
                                    previous_ms=uptime
                                with lock:
                                    data.update(vehicle=msg,uart_at=time.monotonic(),uart_error=None)
                            elif isinstance(msg,dict) and msg.get('type')=='ack' and pending and msg.get('seq')==pending['seq']:
                                job=pending['job']
                                logger.info(json.dumps({'time':time.time(),'ack':msg}))
                                if job:
                                    if msg.get('ok') and job.get('manual') and job['command'].startswith('MANUAL_ARM,'):
                                        with lock:
                                            v=data['vehicle'] or {}
                                            v['state']='MANUAL';v['reason']='MANUAL_ARMED_NO_MOTION'
                                    if msg.get('ok') and job.get('motor'):
                                        with lock:
                                            v=data['vehicle'] or {}
                                            if job['command'].startswith('TEST_ARM,'):
                                                v['state']='MOTOR_TEST';v['reason']='MOTOR_TEST_ARMED'
                                                v['motor_test']={'active':True,'started':False,'start_ms':0,'duration_ms':job['payload']['duration_ms'],'right_pct':0,'left_pct':0}
                                            elif job['command'].startswith('TEST_END,'):
                                                v['state']='READY';v['reason']='MOTOR_TEST_COMPLETE';v['generation']=msg['generation']
                                    if msg.get('ok') and job['command'].startswith('SERVO,'):
                                        _,generation,pulse=job['command'].split(',')
                                        msg['servo_us']=int(pulse)
                                        with lock:
                                            v=data['vehicle'] or {}
                                            msg['status_ms']=v.get('ms',0)
                                            if v.get('generation')==int(generation) and v.get('state')=='READY':
                                                # ACK confirms command acceptance, not measured angle.
                                                v['steer_us_command']=int(pulse)
                                                v['steer_output_enabled']=True
                                    job['result']=msg
                                    job['event'].set()
                                else:
                                    if pending.get('kind')=='sync':
                                        synced=bool(msg.get('ok'))
                                        if synced:
                                            with lock:
                                                if data['vehicle']: data['vehicle']['generation']=msg['generation']
                                    elif pending.get('kind')=='profile':
                                        with lock:
                                            data['profile_sync_error']=None if msg.get('ok') else msg.get('reason','Profile rejected')
                                            if msg.get('ok') and data['vehicle']:
                                                data['vehicle']['servo_profile']=dict(pending['profile'],synced=True)
                                pending=None
                        except (ValueError,UnicodeDecodeError):
                            startup=time.monotonic()-opened<2
                            with lock: data['bad_startup' if startup else 'bad_steady']+=1
                            logger.info(json.dumps({'time':time.time(),'elapsed_s':time.monotonic()-opened,
                                                    'startup':startup,'hex':raw.hex()}))
                    if len(buf)>4096: raise RuntimeError('UART line overflow')
        except Exception as exc:
            if pending and pending['job']:
                pending['job']['result']={'ok':False,'reason':str(exc)}
                pending['job']['event'].set()
            logger.error(json.dumps({'time':time.time(),'error':str(exc),
                                     'pending_seq':pending['seq'] if pending else None}))
            with lock:
                data['uart_error']=str(exc)
                if time.monotonic()-data['uart_at'] > 1: data['vehicle']=None
            # No retry of a motion command after a lost acknowledgement.
            stop.wait(2)


def camera_worker():
    import cv2
    global jpeg
    device = '/dev/v4l/by-id/usb-BC-250219-ZW_USB_Camera_01.00.00-video-index0'
    while not stop.is_set():
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            last_encode = last_record = 0
            preview_times=[];capture_times=[]
            while not stop.is_set():
                read_started=time.monotonic()
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError('Camera read failed')
                now = time.monotonic()
                read_ms=(now-read_started)*1000
                capture_times=[t for t in capture_times if now-t<2];capture_times.append(now)
                if now-last_record>=.1:
                    if recorder: recorder.write(frame)
                    record_only.write(frame)
                    last_record=now
                if now-last_encode < .12:
                    continue
                encode_started=time.monotonic()
                ok, encoded = cv2.imencode('.jpg', cv2.resize(frame, (640, 360)), [cv2.IMWRITE_JPEG_QUALITY,60])
                encode_ms=(time.monotonic()-encode_started)*1000
                if ok:
                    with camera_changed:
                        jpeg = encoded.tobytes()
                        preview_times=[t for t in preview_times if now-t<2];preview_times.append(now)
                        fps=(len(preview_times)-1)/(now-preview_times[0]) if len(preview_times)>1 else 0
                        data.update(camera_at=now, camera_error=None, camera_size=[frame.shape[1],frame.shape[0]],
                                    camera_seq=data.get('camera_seq',0)+1,preview_fps=round(fps,1),
                                    capture_fps=round((len(capture_times)-1)/(now-capture_times[0]),1) if len(capture_times)>1 else 0,
                                    camera_read_ms=round(read_ms,1),preview_encode_ms=round(encode_ms,1))
                        camera_changed.notify_all()
                    last_encode = now
        except Exception as exc:
            with lock:
                data['camera_error'] = str(exc)
        finally:
            cap.release()
        stop.wait(2)


browser_diagnostic_lock=threading.Lock()
def browser_diagnostic(payload):
    # Independent of UART/arming: this endpoint only validates and records data.
    import re
    if not isinstance(payload.get('owner'),str) or not re.fullmatch('[0-9a-f]{32}',payload['owner']):raise ValueError('Invalid diagnostic owner')
    if payload.get('event') not in ('sample','finished','input_error','preview_error','page_error','visibility'):raise ValueError('Invalid diagnostic event')
    detail=payload.get('detail','')
    if not isinstance(detail,str) or len(detail)>400:raise ValueError('Invalid diagnostic detail')
    record={'time':time.time(),'owner':payload['owner'],'event':payload['event'],'detail':detail}
    for key in ('seq','rtt_ms','failures','max_rtt_ms','preview_age_ms','loop_gap_ms','timer_lag_ms'):
        value=payload.get(key)
        if value is not None:
            if type(value) not in (int,float) or not __import__('math').isfinite(value) or not 0<=value<=10000000:raise ValueError('Invalid diagnostic number')
            record[key]=value
    with browser_diagnostic_lock:
        folder=Path.home()/'orin_interface_v1/logs';folder.mkdir(parents=True,exist_ok=True)
        path=folder/'browser_link.jsonl'
        if path.exists() and path.stat().st_size>1000000:os.replace(path,path.with_suffix('.jsonl.1'))
        with path.open('a',encoding='utf-8') as log:log.write(json.dumps(record,ensure_ascii=False)+'\n')
    return {'ok':True,'seq':record.get('seq')}


class DashboardHTTPServer(ThreadingHTTPServer):
    # Streaming clients must not crowd out new control connections.
    request_queue_size = 64


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    timeout = 3

    def camera_stream(self):
        slots=stream_slots
        if not slots.acquire(blocking=False):self.send_error(429,'Too many camera streams');return
        try:
            self.connection.settimeout(1)
            self.connection.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,65536)
            self.send_response(200)
            self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control','no-store')
            self.send_header('Connection','close')
            self.end_headers()
            previous=-1
            while not stop.is_set():
                with camera_changed:
                    camera_changed.wait_for(lambda:stop.is_set() or (data.get('camera_seq',0)!=previous and jpeg is not None),timeout=.5)
                    if stop.is_set():break
                    sequence=data.get('camera_seq',0)
                    if time.monotonic()-data['camera_at']>1:break
                    if sequence==previous or jpeg is None:continue
                    payload=jpeg
                # Keep only the latest image: no per-client queue of old frames.
                previous=sequence
                header=f'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(payload)}\r\nX-Frame-Seq: {sequence}\r\n\r\n'.encode()
                self.wfile.write(header+payload+b'\r\n');self.wfile.flush()
        except (OSError,ConnectionResetError) as exc:
            logging.getLogger('uart').info(json.dumps({'time':time.time(),'camera_stream_error':type(exc).__name__,'detail':str(exc)}))
        finally:
            slots.release();self.close_connection=True

    def authenticated(self):
        try:
            supplied = base64.b64decode(self.headers.get('Authorization','').removeprefix('Basic '), validate=True)
            expected = ('jetson:' + (Path.home()/'.config/orin_lab/token').read_text().strip()).encode()
            authenticated = secrets.compare_digest(supplied, expected)
        except (ValueError, OSError):
            authenticated = False
        if not authenticated:
            self.send_response(401)
            self.close_connection=True
            self.send_header('Connection','close')
            self.send_header('WWW-Authenticate', 'Basic realm="Orin dashboard"')
            self.send_header('Content-Length','0')
            self.end_headers()
            return False
        return True

    def do_GET(self):
        if not self.authenticated(): return
        path = self.path.split('?')[0]
        if path=='/api/manual/ws':
            if self.headers.get('Origin') != 'http://' + self.headers.get('Host',''):
                self.send_error(403);return
            serve_control(self, self.manual_action, lambda p: manual_drive.stop_for_page(p,'CONTROL_SOCKET_LOST'))
            return
        if path=='/camera.mjpg':self.camera_stream();return
        if path in ('/','/manual'):
            payload = (ROOT/('manual.html' if path=='/manual' else 'index.html')).read_bytes()
            kind, code = 'text/html; charset=utf-8', 200
        elif path == '/api/status':
            with lock:
                result = dict(data)
            with profile_lock:
                profile=dict(saved_profile)
                loaded=profile_loaded
            result.update(servo_settings=profile,servo_settings_synced=loaded and profile_matches(result.get('vehicle') or {},profile),motor_busy=bool(motor_tests and motor_tests.busy()))
            now = time.monotonic()
            result.update(uart_age_s=round(now-result.pop('uart_at'),2),
                          camera_age_s=round(now-result.pop('camera_at'),2))
            payload = json.dumps(result).encode()
            kind, code = 'application/json', 200
        elif path == '/api/record/status':
            payload=json.dumps(record_only.snapshot()).encode();kind,code='application/json',200
        elif path == '/api/manual/status':
            payload=json.dumps(dict(manual_drive.snapshot(include_history='compact=1' not in self.path.split('?',1)[-1].split('&')),vehicle=motor_state())).encode();kind,code='application/json',200
        elif path.startswith('/api/manual/files/'):
            run_id=path.removeprefix('/api/manual/files/')
            if not __import__('re').fullmatch('[0-9a-f]{32}',run_id):self.send_error(404);return
            folder=manual_drive.root/run_id
            files=[{'name':f.name,'size':f.stat().st_size} for f in folder.iterdir() if f.is_file()] if folder.is_dir() else []
            payload=json.dumps(files).encode();kind,code='application/json',200
        elif path.startswith('/api/manual/file/'):
            parts=path.removeprefix('/api/manual/file/').split('/')
            if len(parts)!=2 or not __import__('re').fullmatch('[0-9a-f]{32}',parts[0]) or not __import__('re').fullmatch(r'controls\.jsonl|summary\.json|video_[0-9]{3}\.avi|frames_[0-9]{3}\.jsonl',parts[1]):self.send_error(404);return
            f=manual_drive.root/parts[0]/parts[1]
            if not f.is_file():self.send_error(404);return
            if manual_drive.busy() and manual_drive.current['id']==parts[0]:self.send_error(409,'Stop driving before downloading session files');return
            self.send_response(200);self.send_header('Content-Type','application/octet-stream');self.send_header('Content-Length',str(f.stat().st_size));self.send_header('Content-Disposition','attachment; filename='+f.name);self.end_headers()
            try:
                with f.open('rb') as source:__import__('shutil').copyfileobj(source,self.wfile,65536)
            except (BrokenPipeError,ConnectionResetError):pass
            return
        elif path == '/api/motor/status':
            payload=json.dumps(motor_tests.snapshot()).encode()
            kind,code='application/json',200
        elif path.startswith('/api/motor/result/'):
            try:
                name=path.removeprefix('/api/motor/result/')
                if name.endswith('.csv') or name.endswith('.jsonl'):
                    run_id,extension=name.rsplit('.',1)
                    payload=motor_tests.download(run_id,extension)
                    kind='text/csv; charset=utf-8' if extension=='csv' else 'application/x-ndjson'
                else:
                    payload=json.dumps(motor_tests.result(name)).encode();kind='application/json'
                code=200
            except (ValueError,OSError):
                payload,kind,code=b'Not found','text/plain',404
        elif path == '/camera.jpg':
            with lock:
                payload = jpeg if time.monotonic()-data['camera_at'] < 2 else None
            kind, code = ('image/jpeg',200) if payload else ('text/plain',503)
            payload = payload or b'Camera unavailable'
        else:
            payload, kind, code = b'Not found', 'text/plain', 404
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff')
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def manual_action(self,action,payload):
        if action=='diagnostic':result=browser_diagnostic(payload)
        elif action=='start':
            with control_operation_lock:
                if motor_tests.busy():raise ValueError('Stop motor test first')
                result=manual_drive.start(payload)
        elif action=='input':result=manual_drive.input(payload)
        elif action=='settings':
            with control_operation_lock:
                if motor_tests.busy():raise ValueError('Stop motor test first')
                result=manual_drive.settings(payload)
        elif action=='record':result=manual_drive.recording(payload)
        else:
            reason=payload.get('reason','MANUAL_STOP')
            if reason not in ('MANUAL_STOP','BUTTON_STOP','ESC_STOP','WINDOW_BLUR','PAGE_HIDDEN','PAGE_EXIT','PREVIEW_LOST','STATUS_CONNECTION_LOST','CAMERA_OR_UART_STALE','CONTROL_SOCKET_LOST'):raise ValueError('Invalid stop reason')
            if reason in ('MANUAL_STOP','BUTTON_STOP','ESC_STOP'):result=manual_drive.request_stop(reason)
            else:result=manual_drive.stop_for_page(payload,reason)
        return result

    def do_POST(self):
        # Reuse only requests whose body was consumed; rejected requests cannot
        # leave unread bytes to be interpreted as another HTTP request.
        self.close_connection=True
        if not self.authenticated(): return
        try:
            if self.path not in ('/api/record/start','/api/record/stop','/api/servo','/api/servo/settings','/api/motor/start','/api/motor/heartbeat','/api/motor/stop','/api/manual/start','/api/manual/input','/api/manual/settings','/api/manual/stop','/api/manual/record','/api/manual/diagnostic'):
                self.send_error(404); return
            if self.headers.get('Origin') != 'http://' + self.headers.get('Host','') or self.headers.get('X-Orin-Control') != '1':
                self.send_error(403); return
            if self.headers.get('Content-Type') != 'application/json':
                self.send_error(415); return
            length=int(self.headers.get('Content-Length','0'))
            if not 0 < length <= 2048: raise ValueError('Invalid request size')
            payload=json.loads(self.rfile.read(length))
            self.close_connection = self.headers.get('Connection','').lower() == 'close'
            if not isinstance(payload,dict): raise ValueError('Expected object')
            if self.path=='/api/record/start':
                with lock:
                    camera_ok=not data['camera_error'] and time.monotonic()-data['camera_at']<1
                result=record_only.start(camera_ok)
            elif self.path=='/api/record/stop':result=record_only.finish()
            elif self.path.startswith('/api/manual/'):
                action=self.path.rsplit('/',1)[1]
                result=self.manual_action(action,payload)
            elif self.path=='/api/servo/settings':result=save_profile(payload)
            elif self.path=='/api/servo':result=submit_control(payload)
            elif self.path=='/api/motor/start':
                with control_operation_lock:
                    if manual_drive.busy():raise ValueError('Stop manual driving first')
                    result=motor_tests.start(payload)
            elif self.path=='/api/motor/heartbeat':result=motor_tests.heartbeat(payload)
            else:result=motor_tests.request_stop()
            code=200
        except (ValueError,OSError,queue.Full) as exc:
            logging.getLogger('uart').warning(json.dumps({'time':time.time(),'http_control_rejected':self.path,'reason':str(exc)}))
            result={'ok':False,'reason':str(exc) or 'Control queue full'}
            code=409
        output=json.dumps(result).encode()
        self.send_response(code)
        self.send_header('Content-Type','application/json')
        self.send_header('Content-Length',str(len(output)))
        self.send_header('Cache-Control','no-store')
        self.end_headers()
        try: self.wfile.write(output)
        except (BrokenPipeError,ConnectionResetError): pass

    def log_message(self, *args):
        pass


motor_tests=MotorTestManager(Path.home()/'orin_interface_v1/logs/motor_tests',motor_state,lambda p:submit_control(p,motor=True))
manual_root=Path.home()/'orin_interface_v1/logs/manual_driving'
recorder=SessionRecorder(manual_root)
record_only=None

def recording_watchdog():
    while not stop.wait(1):
        try: record_only.check()
        except OSError as exc: logging.getLogger('uart').error('Recording storage error: %s',exc)


def serve_address(address):
    # Each specific interface may appear later during boot or USB replug.
    while not stop.is_set():
        try:
            server = DashboardHTTPServer((address,8000),Handler)
        except OSError as exc:
            print('Waiting for dashboard address',address,str(exc),flush=True)
            stop.wait(5)
            continue
        try:
            server.timeout = 1
            while not stop.is_set():
                server.handle_request()
        finally:
            server.server_close()

def interface_ipv4_addresses(names):
    # Bind concrete assigned addresses, never a wildcard interface.
    import subprocess,ipaddress,re
    addresses=[]
    for name in names:
        if not re.fullmatch('[A-Za-z0-9_.:-]{1,32}',name):raise ValueError('Invalid bind interface')
        try:
            result=subprocess.run(['ip','-j','-4','address','show','dev',name],capture_output=True,text=True,timeout=3)
            if result.returncode:continue
            for interface in json.loads(result.stdout):
                for item in interface.get('addr_info',[]):
                    if item.get('family')=='inet' and item.get('scope')=='global':
                        value=str(ipaddress.IPv4Address(item['local']))
                        if value!='0.0.0.0':addresses.append(value)
        except (OSError,ValueError,subprocess.TimeoutExpired):continue
    return list(dict.fromkeys(addresses))


def watch_interface_listeners(names,started):
    while not stop.is_set():
        for address in interface_ipv4_addresses(names):
            if address not in started:
                started.add(address)
                threading.Thread(target=serve_address,args=(address,),daemon=True).start()
                print('Dashboard interface address:',address,flush=True)
        stop.wait(5)


if __name__ == '__main__':
    import signal
    signal.signal(signal.SIGTERM,lambda *_: stop.set())
    record_only=RecordOnly(Path.home()/'orin_interface_v1/logs/record_only')
    try: load_profile()
    except (ValueError,OSError) as exc: data['profile_sync_error']='Settings file error: '+str(exc)
    manual_drive=ManualDriveManager(manual_root,motor_state,lambda p:submit_control(p,manual=True),manual_profile,save_manual_center,Path.home()/'.config/orin_car/manual_defaults.json',recorder)
    motor_tests.recover()
    threading.Thread(target=uart_worker, daemon=True).start()
    threading.Thread(target=camera_worker, daemon=True).start()
    threading.Thread(target=recording_watchdog, daemon=True).start()
    addresses = list(dict.fromkeys(os.environ.get('ORIN_BIND','127.0.0.1').split(',')))
    if any(not address.strip() or address.strip() in ('0.0.0.0','::') for address in addresses):
        raise ValueError('Use explicit dashboard interface addresses')
    for address in addresses:
        threading.Thread(target=serve_address,args=(address.strip(),),daemon=True).start()
    interfaces=[name.strip() for name in os.environ.get('ORIN_BIND_INTERFACES','').split(',') if name.strip()]
    if interfaces:threading.Thread(target=watch_interface_listeners,args=(interfaces,set(addresses)),daemon=True).start()
    try:
        stop.wait()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        record_only.finish('SERVICE_STOPPED')
