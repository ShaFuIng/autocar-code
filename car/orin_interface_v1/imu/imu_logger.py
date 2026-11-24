"""MPU6050 record-only logger (Orin I2C bus 7, address 0x68).

Separate process; never imported by the vehicle-control service and never
used for control. Samples at RATE_HZ and writes one JSON line per sample to
logs/imu/imu_<start>.jsonl with time.monotonic() (same clock as the PID
records' capture_at / consumed_at), so runs are aligned by time afterwards.

Vehicle frame: X forward, Y left, Z up (mount_config 2026-10-19:
vehicle = (-X, -Y, +Z) of the sensor). Accel calibration: per-axis bias/scale
from the 2026-10-19 six-face candidate. Gyro bias is estimated at every start
from a stationary second (mount_config policy: never reuse an old bias); if
the car is not still, the reference bias is used and marked as such.
pitch_deg: nose up positive (complementary filter, accel tilt + gyro);
roll_deg: left side up positive. Raw vehicle-frame accel/gyro are logged too,
because the horizontal axes were never verified by vehicle motion.

usage: python3 imu_logger.py [--seconds N] [--out DIR] [--rate HZ]
"""
import argparse, json, math, os, signal, struct, sys, time
from pathlib import Path

BUS, ADDR = 7, 0x68
# r5 2026-11-05: the MPU6050 was replaced. The 2026-10-19 six-face values belong to the old chip
# (with them the new chip read |a| = 0.865 g at rest); the new chip reads |a| = 0.992 g raw, so no
# accel calibration is applied until a six-face run of the new chip exists. The flat reference
# (imu_boost.json flat_pitch_deg) absorbs the zero-g offset. Old chip values:
# ACCEL_BIAS_G = (0.049750610351562474, -0.025783203125000043, 0.10277233886718745)
# ACCEL_SCALE = (0.9997081298828124, 0.993606689453125, 1.0340028076171874)
ACCEL_BIAS_G = (0., 0., 0.)
ACCEL_SCALE = (1., 1., 1.)
# r6 2026-11-05: the reference was still the OLD chip's bias; the 18:33 boot start (car moving) used it and the new chip then read
# (2.05, -4.35, 7.19) dps at rest (yaw drift 6.8 deg/s, pitch +4.9 deg). New chip = mean of its 4 still starts 15:37-18:18.
# OLD chip: (-6.185685750636132, -3.07891475826972, -7.829030534351145)
GYRO_REF_BIAS_DPS = (-8.232, 1.31, -0.561)
STILL_WAIT_S = 120.   # r6: at start retry the 1 s stationary check until the car is still (boot service), then fall back
SENSOR_TO_VEHICLE = ((-1, 0, 0), (0, -1, 0), (0, 0, 1))
ALPHA = 0.98                 # complementary filter weight on the gyro path
REVISION = 'imu-logger-r6'
# r4: yaw_deg = integral of the bias-corrected gyro z (left turn positive: live cd6f0c63 left turn
# integrated +90 deg); relative to the logger start, drifts with the residual bias. For heading hold.
# r3: also writes latest.json (atomic, every LATEST_EVERY samples) for the
# uphill PWM boost in live_pid (imu_boost.py); the boost treats it as stale
# after its own max age and then falls back to the base PWM.
LATEST_EVERY = 1   # 2026-11-10 user: latest.json every sample (50 Hz); was 5 (10 Hz)


def rot(v):
    return tuple(sum(SENSOR_TO_VEHICLE[i][j] * v[j] for j in range(3)) for i in range(3))


class MPU:
    def __init__(self, bus):
        import smbus2
        self.b = smbus2.SMBus(bus)
        self.b.write_byte_data(ADDR, 0x6B, 0x01)   # wake, PLL with X gyro
        time.sleep(.1)
        self.b.write_byte_data(ADDR, 0x1A, 0x03)   # DLPF ~44 Hz (motor vibration)
        self.b.write_byte_data(ADDR, 0x1B, 0x00)   # gyro +-250 dps (as calibrated)
        self.b.write_byte_data(ADDR, 0x1C, 0x00)   # accel +-2 g (as calibrated)
        self.who = self.b.read_byte_data(ADDR, 0x75)

    def read(self):
        d = bytes(self.b.read_i2c_block_data(ADDR, 0x3B, 14))
        ax, ay, az, tr, gx, gy, gz = struct.unpack('>hhhhhhh', d)
        acc = tuple((r / 16384. - b) / s for r, b, s in zip((ax, ay, az), ACCEL_BIAS_G, ACCEL_SCALE))
        gyr = (gx / 131., gy / 131., gz / 131.)
        return acc, gyr, tr / 340. + 36.53


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--seconds', type=float, default=0); ap.add_argument('--rate', type=float, default=50)
    ap.add_argument('--out', default=str(Path(__file__).resolve().parents[1] / 'logs' / 'imu')); a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    stop = []; signal.signal(signal.SIGTERM, lambda *_: stop.append(1)); signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    m = MPU(BUS)
    t_warm = time.monotonic() + .3                 # first readings after wake are not valid (10:57 test: norm range 0.92 g)
    while time.monotonic() < t_warm: m.read(); time.sleep(.01)
    # gyro bias from one stationary second; r6: retried until still (at most STILL_WAIT_S), nothing is published meanwhile
    t_wait = time.monotonic() + STILL_WAIT_S; tries = 0
    while True:
        acc_s, gyr_s = [], []; tries += 1
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            ac, gy, _ = m.read(); acc_s.append(ac); gyr_s.append(gy); time.sleep(.01)
        norms = [math.sqrt(sum(x * x for x in v)) for v in acc_s]
        gstd = max(math.sqrt(sum((g[i] - sum(q[i] for q in gyr_s) / len(gyr_s)) ** 2 for g in gyr_s) / len(gyr_s)) for i in range(3))
        still = max(norms) - min(norms) < .05 and gstd < 1.0
        if still or stop or time.monotonic() >= t_wait: break
    bias = tuple(sum(g[i] for g in gyr_s) / len(gyr_s) for i in range(3)) if still else GYRO_REF_BIAS_DPS
    started = time.monotonic(); fn = out / time.strftime('imu_%Y%m%d_%H%M%S.jsonl')
    header = dict(kind='header', revision=REVISION, bus=BUS, address=ADDR, who_am_i=m.who, rate_hz=a.rate, started_monotonic=started, started_wall=time.time(),
                  gyro_bias_dps=bias, gyro_bias_source=('this start, stationary 1 s (try %d)' % tries) if still else 'reference new chip 2026-11-05 (car was not still for %d s)' % STILL_WAIT_S,
                  stationary_check=dict(accel_norm_range_g=max(norms) - min(norms), gyro_std_dps=gstd), frame='vehicle X forward, Y left, Z up',
                  pitch='nose up positive', roll='left side up positive', control_use=False, pid=os.getpid())
    pitch = roll = None; yaw = 0.; last = None; errors = 0; n = 0; period = 1. / a.rate; nxt = time.monotonic()
    with fn.open('a', encoding='utf8') as f:
        f.write(json.dumps(header) + '\n'); f.flush()
        (out / 'current.json').write_text(json.dumps(dict(file=fn.name, pid=os.getpid(), started_monotonic=started)), encoding='utf8')
        while not stop and (a.seconds <= 0 or time.monotonic() - started < a.seconds):
            nxt += period; time.sleep(max(0., nxt - time.monotonic()))
            try:
                ac, gy, temp = m.read()
            except OSError as e:
                errors += 1; f.write(json.dumps(dict(kind='error', t=time.monotonic(), error=str(e), errors=errors)) + '\n'); continue
            t = time.monotonic()
            av = rot(ac); gv = rot(tuple(g - b for g, b in zip(gy, bias)))
            pa = math.degrees(math.atan2(av[0], math.hypot(av[1], av[2])))      # nose up -> +X sees +g
            ra = math.degrees(math.atan2(av[1], av[2]))
            dt = 0. if last is None else t - last; last = t
            yaw += gv[2] * dt
            if pitch is None: pitch, roll = pa, ra
            else:
                pitch = ALPHA * (pitch - gv[1] * dt) + (1 - ALPHA) * pa              # rotation about +Y(left) is nose down
                roll = ALPHA * (roll + gv[0] * dt) + (1 - ALPHA) * ra
            n += 1
            if n % LATEST_EVERY == 0:
                tmp = out / 'latest.json.tmp'
                tmp.write_text(json.dumps(dict(t=round(t, 4), pitch_deg=round(pitch, 2), roll_deg=round(roll, 2), yaw_deg=round(yaw, 3), gyro_dps=[round(x, 3) for x in gv],
                                               revision=REVISION, started_monotonic=started, pid=os.getpid())), encoding='utf8')
                os.replace(tmp, out / 'latest.json')
            f.write(json.dumps(dict(kind='imu', t=round(t, 4), pitch_deg=round(pitch, 2), roll_deg=round(roll, 2), yaw_deg=round(yaw, 3), pitch_acc_deg=round(pa, 2),
                                    acc_g=[round(x, 4) for x in av], gyro_dps=[round(x, 3) for x in gv], temp_c=round(temp, 2))) + '\n')
            if n % 50 == 0: f.flush()
        f.write(json.dumps(dict(kind='footer', t=time.monotonic(), samples=n, errors=errors)) + '\n')
    for name in ('current.json', 'latest.json'):
        try: (out / name).unlink()
        except OSError: pass


if __name__ == '__main__':
    main()
