"""landing3-20261121: the two 'No firmware acknowledgement' endings (run-9072, run-f4a8) from controls.jsonl, against normal downhills.
usage: python ack.py"""
import json, sys, collections
from pathlib import Path
import numpy as np
EXP = Path(__file__).resolve().parents[3]


def load(run):
    out = []
    for l in open(str(EXP / run / 'controls.jsonl'), encoding='utf8'):
        try: x = json.loads(l)
        except ValueError: continue
        d = x.get('dispatch_timing') or {}; v = x.get('vehicle') or {}; c = x.get('command') or {}
        out.append(dict(it=d.get('iteration_at'), sent=d.get('sent_at'), ack_at=d.get('ack_at'), it2ack=d.get('iteration_to_ack_ms'), pend=d.get('ack_pending'), prev_ack=d.get('prev_ack_ms'),
                        ack_wait=d.get('ack_wait_ms'), kick=d.get('kick_wait_ms'), kicked=d.get('kicked'), sup_total=d.get('ack_superseded_total'), mode=d.get('mode'), other={k: d[k] for k in d if k not in (
                            'iteration_at', 'sent_at', 'ack_at', 'iteration_to_ack_ms', 'ack_pending', 'prev_ack_ms', 'ack_wait_ms', 'kick_wait_ms', 'kicked', 'ack_superseded_total', 'mode', 'pid_target_us', 'dispatch_sent_monotonic_s', 'period_includes_ack', 'ack_superseded', 'ack_supersede', 'direction_change_wait')},
                        ms=v.get('ms'), cmd_age=v.get('cmd_age_ms'), rx_good=v.get('rx_good'), rx_bad=v.get('rx_bad'), rx_rej=v.get('rx_rejected'), tx_drop=v.get('tx_dropped'), loop=v.get('max_loop_us'),
                        i2c=v.get('i2c_errors'), state=v.get('state'), reason=v.get('reason'), steer_mcu=v.get('steer_us_command'), pwm_a=(v.get('motor_a') or {}).get('pwm_command'),
                        enc_ms=v.get('enc_sample_ms'), cmd=(c.get('right'), c.get('steer_us')), in_age=x.get('input_age_ms'), uart=(x.get('feedback_observation') or {}).get('uart_error')))
    return out


def tail(run, n=40):
    R = load(run); print('=' * 100); print(run, 'records', len(R)); t_end = R[-1]['it']
    print('  dt_end  d_iter_ms  pend prev_ack_ms kick_ms kicked | mcu ms (d)  cmd_age  rx_good(d) rx_rej tx_drop i2c state/reason | cmd(pwm,steer) mcu steer/pwm | in_age  other')
    for a, b in zip(R[-n - 1:-1], R[-n:]):
        print('  %+6.2f  %7.1f  %4s %10s %7s %5s | %s (%s)  %5s  %s (%s) %s %s %s %s/%s | %s %s/%s | %s %s' % (
            b['it'] - t_end, (b['it'] - a['it']) * 1000., b['pend'], None if b['prev_ack'] is None else round(b['prev_ack'], 1), None if b['kick'] is None else round(b['kick'], 1), b['kicked'],
            b['ms'], None if a['ms'] is None or b['ms'] is None else b['ms'] - a['ms'], b['cmd_age'], b['rx_good'], None if a['rx_good'] is None or b['rx_good'] is None else b['rx_good'] - a['rx_good'],
            b['rx_rej'], b['tx_drop'], b['i2c'], b['state'], (b['reason'] or '')[:22], b['cmd'], b['steer_mcu'], b['pwm_a'], b['in_age'], b['other'] or ''))
    return R


def stats(run, label, sel=None):
    R = load(run); it = np.array([r['it'] for r in R], float); d = np.diff(it) * 1000.
    pend = np.array([r['pend'] if r['pend'] is not None else np.nan for r in R], float)
    prev = np.array([r['prev_ack'] if r['prev_ack'] is not None else np.nan for r in R], float)
    cmd_age = np.array([r['cmd_age'] if r['cmd_age'] is not None else np.nan for r in R], float)
    msd = np.diff(np.array([r['ms'] if r['ms'] is not None else np.nan for r in R], float))
    print('%s %s: n %d | dispatch interval ms: median %.1f P95 %.1f P99 %.1f max %.1f, > 100 ms: %d (%.2f%%), > 150 ms: %d | ack_pending: max %s, >=2 on %d records | prev_ack_ms P50 %.1f P95 %.1f P99 %.1f max %.1f | mcu cmd_age_ms P50 %.0f P99 %.0f max %.0f | records with no new mcu status (ms unchanged): %d, longest stale run %d | rx_rejected %s -> %s, tx_dropped %s -> %s, rx_bad %s -> %s, max_loop_us %s -> %s' % (
        run[4:8], label, len(R), np.median(d), np.percentile(d, 95), np.percentile(d, 99), d.max(), int((d > 100).sum()), 100. * (d > 100).mean(), int((d > 150).sum()), np.nanmax(pend), int((pend >= 2).sum()),
        np.nanmedian(prev), np.nanpercentile(prev, 95), np.nanpercentile(prev, 99), np.nanmax(prev), np.nanmedian(cmd_age), np.nanpercentile(cmd_age, 99), np.nanmax(cmd_age),
        int((msd == 0).sum()), max([len(list(g)) for k, g in __import__('itertools').groupby(msd == 0) if k] or [0]),
        R[0]['rx_rej'], R[-1]['rx_rej'], R[0]['tx_drop'], R[-1]['tx_drop'], R[0]['rx_bad'], R[-1]['rx_bad'], R[0]['loop'], R[-1]['loop']))
    # where do the > 100 ms dispatch gaps and the rx_rejected / tx_dropped steps happen (seconds before the end)
    gaps = [(round(it[i + 1] - it[-1], 1), round(d[i])) for i in np.flatnonzero(d > 100)]
    steps = [(round(R[i]['it'] - it[-1], 1), k, R[i - 1][k], R[i][k]) for i in range(1, len(R)) for k in ('rx_rej', 'tx_drop', 'rx_bad', 'loop', 'i2c') if R[i][k] != R[i - 1][k]]
    print('    dispatch gaps > 100 ms (s before the end, ms): %s' % gaps[-25:]); print('    counter steps (s before the end, counter, from, to): %s' % steps[-25:])


if __name__ == '__main__':
    for run in ('run-9072-20261121', 'run-f4a8-20261121'): tail(run)
    print('=' * 100)
    for run, label in (('run-9072-20261121', 'ended by the ack timeout on the slope'), ('run-f4a8-20261121', 'ended by the ack timeout after the landing'), ('run-9934-20261121', 'normal, 4 laps'), ('run-4a78-20261121', 'normal, 4 laps')):
        if (EXP / run / 'controls.jsonl').exists(): stats(run, label)
