import importlib.util
import json
from pathlib import Path
import subprocess
import unittest
import tempfile
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("orin_client", ROOT / "tools/orin_client.py")
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)


class ClientTests(unittest.TestCase):
    def test_startup_waits_once_before_write_without_flushing_ack(self):
        clock = [0.0]
        class UART:
            def __init__(self):
                self.data = bytearray(b"stale startup bytes")
                self.flushes = 0
                self.writes = []

            @property
            def in_waiting(self):
                # Emulate periodic telemetry after the UART settles.
                if clock[0] >= 1.0 and not self.data and not self.writes:
                    self.data.extend(b'{"type":"status","state":"LOCKED","generation":3}\n')
                return len(self.data)

            def read(self, count):
                chunk = bytes(self.data[:count])
                del self.data[:count]
                return chunk

            def reset_input_buffer(self):
                self.flushes += 1
                self.data.clear()

            def write(self, data):
                self.writes.append((clock[0], data))
                seq = int(data.split(b",")[0][1:])
                self.data.extend(json.dumps({"type":"ack", "seq":seq, "ok":True,
                                             "reason":"OK", "generation":4}).encode() + b"\n")
                return len(data)

        instance = client.Client.__new__(client.Client)
        instance.serial = UART()
        instance.seq = 0
        instance.buffer = bytearray()
        instance.status = None
        instance.status_at = 0
        instance.generation = None
        instance.pending = {}
        instance.acks = {}
        instance.errors = []
        instance.log = None
        instance.prepared = False
        instance.init_uart_diagnostics(None)
        with patch.object(client.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(client.time, "sleep", side_effect=lambda n: clock.__setitem__(0, clock[0] + n)), \
             patch("builtins.print"):
            ack = instance.request("STOP")
            self.assertTrue(ack["ok"])
            self.assertGreaterEqual(instance.serial.writes[0][0], 2.0)
            self.assertEqual(instance.status["state"], "LOCKED")
            before = clock[0]
            instance.request("STOP")
            self.assertEqual(clock[0], before)
            self.assertEqual(instance.serial.flushes, 1)
            self.assertFalse(instance.pending)

    def test_partial_json_and_rejected_ack(self):
        class UART:
            def __init__(self):
                self.data = bytearray()

            @property
            def in_waiting(self):
                return len(self.data)

            def read(self, count):
                chunk = bytes(self.data[:count])
                del self.data[:count]
                return chunk

        instance = client.Client.__new__(client.Client)
        instance.serial = UART()
        instance.buffer = bytearray()
        instance.status = None
        instance.status_at = 0
        instance.generation = None
        instance.pending = {7: 0}
        instance.acks = {}
        instance.errors = []
        instance.log = None
        instance.init_uart_diagnostics(None)
        instance.serial.data.extend(b'{"type":"ack","seq":7,"ok":false,')
        instance.pump()
        self.assertIn(7, instance.pending)
        instance.serial.data.extend(b'"reason":"NOT_ARMED","generation":9}\n')
        instance.pump()
        self.assertNotIn(7, instance.pending)
        self.assertEqual(instance.generation, 9)
        self.assertEqual(instance.errors[0]["reason"], "NOT_ARMED")

    def test_raw_capture_keeps_corrupt_and_fragmented_bytes_without_writing(self):
        class UART:
            data = bytearray()
            @property
            def in_waiting(self):
                return len(self.data)
            def read(self, count):
                result = bytes(self.data[:count])
                del self.data[:count]
                return result
            def close(self):
                pass
            def write(self, data):
                raise AssertionError("RX diagnostic must not transmit")

        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as tmp:
            instance = client.Client.__new__(client.Client)
            instance.serial = UART()
            instance.buffer = bytearray()
            instance.log = None
            path = Path(tmp) / "raw.jsonl"
            instance.init_uart_diagnostics(path)
            original = b'\x00\xff\n{"type":"status","state":"LOCKED"}\n'
            with patch("builtins.print"):
                instance.serial.data.extend(original[:8])
                instance.pump()
                instance.serial.data.extend(original[8:])
                instance.pump()
                instance.diagnose(0)
            instance.close()
            records = [json.loads(line) for line in path.read_text().splitlines()]
            captured = b''.join(bytes.fromhex(r["hex"]) for r in records if r["event"] == "rx_chunk")
            self.assertEqual(captured, original)
            self.assertEqual(instance.uart_stats["bad_startup"], 1)
            self.assertEqual(instance.uart_stats["bad_steady"], 0)
            self.assertEqual(instance.uart_stats["json_lines"], 1)
            self.assertEqual(instance.status["state"], "LOCKED")
            self.assertEqual(records[-1]["event"], "close")

    def test_crc_reference(self):
        # Published CCITT-FALSE check vector, independent of firmware encoder.
        self.assertEqual(client.binascii.crc_hqx(b"123456789", 0xFFFF), 0x29B1)
        self.assertEqual(client.encode_frame(1, "STOP"), b"$1,STOP*CC4D\n")

    def test_actual_firmware_behavior_and_json(self):
        exe = ROOT / "tests/firmware_test.exe"
        result = subprocess.run([str(exe)], capture_output=True, text=True, check=True)
        records = [json.loads(line) for line in result.stdout.splitlines()]
        statuses = [msg for msg in records if msg["type"] == "status"]
        self.assertTrue(statuses)
        self.assertEqual(statuses[-1]["rx_good"], 0xFFFFFFFF)
        self.assertIsNone(statuses[-1]["voltage_v"])
        self.assertTrue(any(msg.get("reason") == "STALE_SEQUENCE" for msg in records))
        self.assertTrue(any(msg.get("reason") == "WRONG_MODE" for msg in records))


if __name__ == "__main__":
    unittest.main()
