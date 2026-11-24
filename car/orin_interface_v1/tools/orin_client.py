#!/usr/bin/env python3
"""Single-owner UART test client. Motion needs a new physical start press."""
import argparse
import binascii
import datetime as dt
import json
import math
import time
from pathlib import Path


def encode_frame(seq, command, *args):
    body = ",".join(map(str, (seq, command, *args))).encode("ascii")
    return b"$" + body + b"*%04X\n" % binascii.crc_hqx(body, 0xFFFF)


class Client:
    def __init__(self, port, log=None, uart_log=None):
        import serial
        self.serial = serial.Serial(port, 115200, timeout=0, write_timeout=0.2,
                                    exclusive=True)
        self.seq = 0
        self.buffer = bytearray()
        self.status = None
        self.status_at = 0.0
        self.generation = None
        self.pending = {}
        self.acks = {}
        self.errors = []
        self.log = Path(log).open("a", encoding="utf-8") if log else None
        self.prepared = False
        self.init_uart_diagnostics(uart_log)

    def init_uart_diagnostics(self, path):
        self.opened_at = time.monotonic()
        self.uart_log = Path(path).open("a", encoding="utf-8") if path else None
        self.uart_stats = {"bytes": 0, "json_lines": 0, "bad_startup": 0,
                           "bad_steady": 0}
        self.record_uart("open", baud=115200, startup_window_s=2)

    def record_uart(self, event, **fields):
        if self.uart_log:
            self.uart_log.write(json.dumps({
                "host_time": dt.datetime.now().astimezone().isoformat(),
                "elapsed_s": round(time.monotonic() - self.opened_at, 6),
                "event": event, **fields}, ensure_ascii=False) + "\n")
            self.uart_log.flush()

    def diagnose(self, seconds):
        """Keep one connection open and capture RX only, including startup."""
        deadline = time.monotonic() + seconds
        next_report = time.monotonic() + 10
        while time.monotonic() < deadline:
            self.pump()
            if time.monotonic() >= next_report:
                print("UART RX:", self.uart_stats, flush=True)
                next_report = time.monotonic() + 10
            time.sleep(0.005)
        print("UART RX final:", self.uart_stats, flush=True)

    def prepare(self):
        """Receive for two seconds once per open, before issuing commands.

        This handles the startup issue observed on this vehicle. It is not a
        guarantee against persistent UART corruption; pump still reports it.
        Never flush while commands are pending, as that could discard ACKs.
        """
        if self.prepared:
            return
        print("UART startup: receiving for 2 seconds before commands...")
        self.record_uart("startup_flush", waiting_bytes=self.serial.in_waiting)
        self.serial.reset_input_buffer()
        self.buffer.clear()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.pump()
            time.sleep(0.01)
        self.prepared = True

    def send(self, command, *args):
        self.prepare()
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        data = encode_frame(self.seq, command, *args)
        if self.serial.write(data) != len(data):
            raise RuntimeError("UART write incomplete")
        self.pending[self.seq] = time.monotonic()
        return self.seq

    def pump(self):
        chunk = self.serial.read(min(self.serial.in_waiting, 8192))
        if chunk:
            self.uart_stats["bytes"] += len(chunk)
            self.record_uart("rx_chunk", length=len(chunk), hex=chunk.hex())
        self.buffer.extend(chunk)
        while b"\n" in self.buffer:
            raw, _, remaining = self.buffer.partition(b"\n")
            self.buffer = bytearray(remaining)
            try:
                msg = json.loads(raw)
            except (ValueError, UnicodeDecodeError):
                startup = time.monotonic() - self.opened_at < 2
                self.uart_stats["bad_startup" if startup else "bad_steady"] += 1
                self.record_uart("invalid_line", startup=startup,
                                 length=len(raw), hex=raw.hex())
                print("Unparsed UART:", raw[:160])
                continue
            self.uart_stats["json_lines"] += 1
            if not isinstance(msg, dict):
                continue
            if self.log:
                self.log.write(json.dumps({"host_time": dt.datetime.now().astimezone().isoformat(),
                                           "rx": msg}, ensure_ascii=False) + "\n")
                self.log.flush()
            if "generation" in msg:
                self.generation = msg["generation"]
            if msg.get("type") == "status":
                self.status = msg
                self.status_at = time.monotonic()
            elif msg.get("type") == "ack":
                seq = msg.get("seq")
                if seq in self.pending:
                    self.pending.pop(seq)
                    self.acks[seq] = msg
                    if not msg.get("ok"):
                        self.errors.append(msg)
                # Bound storage even during long monitoring sessions.
                while len(self.acks) > 64:
                    self.acks.pop(next(iter(self.acks)))
            elif msg.get("type") == "event":
                print("EVENT", msg.get("event"), msg.get("detail"))
        if len(self.buffer) > 4096:
            self.record_uart("line_overflow", length=len(self.buffer),
                             hex=self.buffer.hex())
            self.buffer.clear()
            raise RuntimeError("UART line too long / missing newline")

    def request(self, command, *args, timeout=3.0):
        seq = self.send(command, *args)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if seq in self.acks:
                ack = self.acks.pop(seq)
                if not ack.get("ok"):
                    raise RuntimeError(f"{command} rejected: {ack.get('reason')}")
                return ack
            time.sleep(0.005)
        raise TimeoutError(f"No ACK for {command}")

    def fresh_status(self, timeout=3.0):
        self.prepare()
        after = time.monotonic()
        deadline = after + timeout
        while time.monotonic() < deadline:
            self.pump()
            if self.status_at >= after:
                return self.status
            time.sleep(0.005)
        raise TimeoutError("No firmware status; check UART wiring/baud/firmware")

    def drive(self, throttle, steer, duration):
        self.request("SYNC")
        print("Raise the drive wheels. Release ESTOP, then press/release physical START.")
        deadline = time.monotonic() + 30
        while True:
            status = self.fresh_status()
            if status["mode"] != "ORIN":
                raise RuntimeError("Select MODE ORIN before driving")
            if status["state"] == "FAULT":
                raise RuntimeError(status["reason"])
            if status["state"] == "READY":
                generation = status["generation"]
                break
            if time.monotonic() > deadline:
                raise TimeoutError("Physical START not received")
        try:
            self.request("ARM", generation)
            self.request("MOVE", generation, 0, 0)
            self.errors.clear()
            until = time.monotonic() + duration
            next_send = time.monotonic()
            while time.monotonic() < until:
                self.pump()
                now = time.monotonic()
                if self.errors:
                    raise RuntimeError(f"MOVE rejected: {self.errors[-1]['reason']}")
                if self.generation != generation:
                    raise RuntimeError("Authorization changed / vehicle stopped")
                if now - self.status_at > 1.0:
                    raise TimeoutError("Telemetry lost")
                if any(now - sent > 1.0 for sent in self.pending.values()):
                    raise TimeoutError("Motion ACK lost")
                if now >= next_send:
                    self.send("MOVE", generation, throttle, steer)
                    next_send = now + 0.05
                time.sleep(0.005)
        finally:
            # STOP latches LOCKED. Never auto-rearm after an exception/reconnect.
            try:
                self.request("STOP")
            except Exception as exc:
                print("STOP could not be confirmed:", exc, "(firmware command timeout: 500 ms)")

    def close(self):
        self.record_uart("close", stats=self.uart_stats,
                         partial_hex=self.buffer.hex())
        self.serial.close()
        if self.log:
            self.log.close()
        if self.uart_log:
            self.uart_log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyTHS1")
    parser.add_argument("--log", help="Append received messages with host timestamps to JSONL")
    parser.add_argument("--uart-log", help="Append raw RX hex, timing and UART diagnostics to a separate JSONL")
    subs = parser.add_subparsers(dest="action", required=True)
    subs.add_parser("status")
    subs.add_parser("monitor")
    subs.add_parser("stop")
    diagnose = subs.add_parser("diagnose", help="RX only; keep one UART connection open")
    diagnose.add_argument("--seconds", type=float, default=60)
    mode = subs.add_parser("mode")
    mode.add_argument("value", choices=["ORIN", "BLE"])
    drive = subs.add_parser("drive")
    drive.add_argument("--throttle", type=int, default=0)
    drive.add_argument("--steer", type=int, default=0)
    drive.add_argument("--seconds", type=float, default=2.0)
    args = parser.parse_args()
    if args.action == "diagnose" and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("diagnose seconds must be finite and >0")
    if args.action == "drive":
        if not (-25 <= args.throttle <= 25 and -100 <= args.steer <= 100):
            parser.error("First version: throttle -25..25, steer -100..100")
        if not math.isfinite(args.seconds) or not 0 < args.seconds <= 30:
            parser.error("seconds must be >0 and <=30")
    client = Client(args.port, args.log, args.uart_log)
    try:
        if args.action == "status":
            print(json.dumps(client.fresh_status(), indent=2, ensure_ascii=False))
        elif args.action == "diagnose":
            client.diagnose(args.seconds)
        elif args.action == "stop":
            print(client.request("STOP"))
        elif args.action == "mode":
            client.request("SYNC")
            print(client.request("MODE", args.value))
        elif args.action == "drive":
            client.drive(args.throttle, args.steer, args.seconds)
        else:
            while True:
                msg = client.fresh_status()
                print(f"{msg['ms']:>10} {msg['mode']} {msg['state']} {msg['reason']} "
                      f"PWM={msg['motor_a']['pwm_command']}/{msg['motor_b']['pwm_command']} "
                      f"ENC={msg['enc_a']['delta']}/{msg['enc_b']['delta']} "
                      f"seen={msg['enc_a']['seen']}/{msg['enc_b']['seen']}")
    except KeyboardInterrupt:
        print("Interrupted")
    except Exception as exc:
        parser.exit(1, f"Error: {exc}\n")
    finally:
        client.close()


if __name__ == "__main__":
    main()
