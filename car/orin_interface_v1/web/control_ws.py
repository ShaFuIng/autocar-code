"""Dedicated authenticated WebSocket; diagnostics never dispatch vehicle actions."""
import json
import logging
import logging.handlers
from pathlib import Path
import queue
import secrets
import socket
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).parent / 'ws_vendor'))
from wsproto import WSConnection, ConnectionType
from wsproto.events import AcceptConnection, Request, TextMessage, BytesMessage, Ping, CloseConnection

slots = threading.BoundedSemaphore(8)
log_lock = threading.Lock()
log_queue = queue.Queue(maxsize=4096)
log_started = False


def record(value):
    # Disk latency must not stall control acknowledgements.
    global log_started
    with log_lock:
        if not log_started:
            folder = Path.home() / 'orin_interface_v1/logs'
            folder.mkdir(parents=True, exist_ok=True)
            handler = logging.handlers.RotatingFileHandler(folder / 'control_ws.jsonl', maxBytes=4000000, backupCount=3)
            handler.setFormatter(logging.Formatter('%(message)s'))
            logging.handlers.QueueListener(log_queue, handler).start()
            log_started = True
    try:
        log_queue.put_nowait(logging.LogRecord('control_ws', logging.INFO, '', 0, json.dumps(value), (), None))
    except queue.Full:
        logging.getLogger('uart').warning('control_ws diagnostic queue full; sample dropped')


class Channel:
    def __init__(self, dispatch, clock=time.monotonic):
        self.connection_id = secrets.token_hex(8)
        self.dispatch = dispatch
        self.clock = clock
        self.tokens = {}
        self.owner_run = None
        self.last_received = None

    def stamp(self):
        now = self.clock()
        self.tokens = {k: v for k, v in self.tokens.items() if now-v <= .8}
        token = secrets.token_hex(16)
        self.tokens[token] = now
        return {'freshness': token, 'server_mono_ms': now*1000}

    def handle(self, p):
        received = self.clock()
        if not isinstance(p, dict) or type(p.get('id')) is not int:
            raise ValueError('Invalid message')
        action = p.get('action')
        payload = p.get('payload', {})
        if not isinstance(payload, dict): raise ValueError('Invalid payload')
        if action not in ('probe','report','input','start','settings','record','stop'):
            raise ValueError('Unsupported action')
        gap = None if self.last_received is None else (received-self.last_received)*1000
        self.last_received = received
        token_at = self.tokens.get(p.get('freshness'), -1e9)
        result = {'ok': True}
        try:
            # A server-issued token bounds message age without trusting PC clock sync.
            if action not in ('probe','report','stop') and received-token_at > .8:
                raise ValueError('Expired control freshness; reconnect before enabling')
            if action == 'report':
                pass  # Telemetry only: never dispatch or renew the driving lease.
            elif action == 'probe':
                pass  # Same transport/ACK path, zero vehicle calls.
            else:
                result = self.dispatch(action, payload)
                if action == 'start' and result.get('ok'):
                    self.owner_run = {'owner': payload.get('owner'), 'run_id': result['run_id']}
                elif action == 'input' and result.get('ok'):
                    self.owner_run = {'owner': payload.get('owner'), 'run_id': payload.get('run_id')}
        except (ValueError, RuntimeError) as exc:
            result = {'ok': False, 'reason': str(exc)}
        finished = self.clock()
        timing = {'received_mono_ms': received*1000, 'server_processing_ms': (finished-received)*1000,
                  'receive_gap_ms': gap, 'freshness_age_ms': (received-token_at)*1000 if token_at > 0 else None}
        record({'time': time.time(), 'connection_id': self.connection_id, 'id': p['id'], 'action': action, 'ok': result.get('ok'),
                'reason': result.get('reason'), 'seq': payload.get('seq'), 'run_id': payload.get('run_id'),
                'client': {k: p.get(k) for k in ('client_send_ms','timer_lag_ms','buffered_bytes','last_ack')}, **timing})
        return {'id': p['id'], **result, **timing, **self.stamp()}


def serve_control(handler, dispatch, disconnected):
    if not slots.acquire(blocking=False): handler.send_error(429); return
    channel = Channel(dispatch)
    ws = WSConnection(ConnectionType.SERVER)
    handler.close_connection = True
    started = False
    try:
        ws.initiate_upgrade_connection([(k.lower().encode(), v.encode()) for k,v in handler.headers.items()], handler.path)
        if not any(isinstance(e, Request) for e in ws.events()): raise ValueError('Invalid upgrade')
        handler.connection.settimeout(1)
        handler.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        handler.wfile.write(ws.send(AcceptConnection()))
        handler.wfile.write(ws.send(TextMessage(data=json.dumps({'type':'hello', **channel.stamp()}))))
        handler.wfile.flush()
        started = True
        text = ''
        last_data = time.monotonic()
        while True:
            try: raw = handler.rfile.read1(4096)
            except socket.timeout: break
            if not raw: break
            ws.receive_data(raw)
            for event in ws.events():
                if isinstance(event, CloseConnection):
                    handler.wfile.write(ws.send(event.response()));return
                if isinstance(event, Ping): handler.wfile.write(ws.send(event.response()))
                if isinstance(event, BytesMessage): raise ValueError('Text only')
                if isinstance(event, TextMessage):
                    text += event.data
                    if len(text)>8192: raise ValueError('Message too large')
                    if event.message_finished:
                        response = channel.handle(json.loads(text));text=''
                        handler.wfile.write(ws.send(TextMessage(data=json.dumps(response))))
                        handler.wfile.flush()
                        last_data = time.monotonic()
            if time.monotonic()-last_data > 1: break
    except Exception as exc:
        record({'time':time.time(),'event':'socket_error','detail':str(exc)[:300]})
        if not started:
            try: handler.send_error(400)
            except OSError: pass
    finally:
        if channel.owner_run: disconnected(channel.owner_run)
        slots.release()
