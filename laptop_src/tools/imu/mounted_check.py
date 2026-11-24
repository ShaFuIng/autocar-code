"""Bounded MPU6050 check. No UART, GPIO motor outputs, services or boot edits."""
from pathlib import Path
import datetime
import hashlib
import json
import math
import statistics
import struct
import time


def rotate(values, matrix):
    return [sum(row[j] * values[j] for j in range(3)) for row in matrix]


def convert(values, calibration, gyro_bias, matrix):
    # Offsets are expressed in SENSOR axes: correct BEFORE rotating.
    accel = [(values[j] - calibration['accel_bias_g'][j]) /
             calibration['accel_scale'][j] for j in range(3)]
    gyro = [values[j + 4] - gyro_bias[j] for j in range(3)]
    return rotate(accel, matrix), rotate(gyro, matrix)


def stats(values):
    return {'mean': statistics.mean(values), 'std': statistics.pstdev(values),
            'min': min(values), 'max': max(values)}


def main():
    from smbus2 import SMBus, i2c_msg
    root = Path(__file__).resolve().parent
    config_bytes = (root / 'mount_config.json').read_bytes()
    cfg = json.loads(config_bytes)
    matrix = cfg['sensor_to_vehicle']
    assert matrix == [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]
    assert cfg['vehicle_control_enabled'] is False
    source = root / cfg['accel_calibration_file']
    cal_bytes = source.read_bytes()
    assert hashlib.sha256(cal_bytes).hexdigest() == cfg['accel_calibration_sha256']
    cal = json.loads(cal_bytes)
    rows, errors = [], 0
    with SMBus(cfg['bus']) as bus:
        addr = cfg['address']

        def read(reg, count):
            w = i2c_msg.write(addr, [reg]); r = i2c_msg.read(addr, count)
            bus.i2c_rdwr(w, r)
            return bytes(r)

        assert read(0x75, 1)[0] == 0x68, 'Unexpected WHO_AM_I'
        regs = {r: read(r, 1)[0] for r in (0x6B, 0x6C, 0x1B, 0x1C, 0x1A, 0x19)}
        try:
            bus.write_byte_data(addr, 0x6B, regs[0x6B] & 0x1F)
            bus.write_byte_data(addr, 0x6C, 0)
            print('ID OK; 10s settling then 30s stationary capture', flush=True)
            time.sleep(10)
            asc = (16384, 8192, 4096, 2048)[(regs[0x1C] >> 3) & 3]
            gsc = (131, 65.5, 32.8, 16.4)[(regs[0x1B] >> 3) & 3]
            start = time.monotonic()
            for _ in range(1500):
                try:
                    raw = struct.unpack('>7h', read(0x3B, 14))
                    values = [x / asc for x in raw[:3]] + [raw[3] / 340 + 36.53] + [x / gsc for x in raw[4:]]
                    rows.append({'t': time.monotonic() - start, 'raw': raw, 'values': values})
                except OSError:
                    errors += 1
                time.sleep(.02)
        finally:
            bus.write_byte_data(addr, 0x6C, regs[0x6C])
            bus.write_byte_data(addr, 0x6B, regs[0x6B])
            restored = read(0x6B, 1)[0] == regs[0x6B] and read(0x6C, 1)[0] == regs[0x6C]
            print('power_restored', restored, flush=True)
    if len(rows) < 1400:
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        (root / ('mounted_failed_' + stamp + '.json')).write_text(json.dumps({'samples': rows, 'errors': errors}))
        raise RuntimeError('Insufficient samples; partial data saved, no bias accepted')
    training, holdout = rows[:750], rows[750:]
    bias = [statistics.mean(r['values'][j] for r in training) for j in (4, 5, 6)]
    converted = [convert(r['values'], cal, bias, matrix) for r in holdout]
    acc_stats = [stats([r[0][j] for r in converted]) for j in range(3)]
    gyro_stats = [stats([r[1][j] for r in converted]) for j in range(3)]
    norm = stats([math.sqrt(sum(x*x for x in r[0])) for r in converted])
    stable = all(statistics.pstdev(r['values'][j] for r in rows) < .02 for j in range(3)) and all(
        statistics.pstdev(r['values'][j] for r in rows) < .3 for j in (4, 5, 6))
    # Screening limits for this stationary diagnostic, not safety qualification.
    passed = stable and errors == 0 and restored and abs(norm['mean'] - 1) < .03 and all(
        abs(s['mean']) < .1 for s in gyro_stats)
    report = {'status': 'stationary_screen_pass' if passed else 'review_required',
              'samples': len(rows), 'errors': errors, 'power_restored': restored,
              'stable_screen': stable, 'mount_config': cfg,
              'mount_config_sha256': hashlib.sha256(config_bytes).hexdigest(),
              'registers_before': regs, 'gyro_bias_sensor_dps_this_run_only': bias,
              'bias_estimation_samples': len(training), 'independent_holdout_samples': len(holdout),
              'vehicle_accel_g_holdout': acc_stats, 'vehicle_gyro_dps_holdout': gyro_stats,
              'corrected_accel_norm_g_holdout': norm,
              'temperature_estimation_C': stats([r['values'][3] for r in training]),
              'temperature_holdout_C': stats([r['values'][3] for r in holdout]),
              'limitations': ['Yaw/vehicle X-Y alignment cannot be verified while stationary',
                             'No dynamic, vibration, long-term drift or temperature calibration',
                             'Acceleration includes gravity; no speed/position inference',
                             'No vehicle control integration; gyro bias must not be reused blindly']}
    for row in rows:
        acc, gyro = convert(row['values'], cal, bias, matrix)
        row['vehicle_accel_g'] = acc
        row['vehicle_gyro_dps'] = gyro
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out = root / ('mounted_validation_' + stamp + '.json')
    out.write_text(json.dumps({'report': report, 'samples': rows}, indent=2))
    print('SAVED', out, flush=True)
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
