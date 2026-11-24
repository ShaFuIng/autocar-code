# -*- coding: utf-8 -*-
"""到場驗收：對一輪（或多輪）紀錄印出階梯表要看的數字（每圈），與 e69d 基準比較。
用法：
  1) python extract.py <暫存夾> <run 資料夾 ...>        （先抽欄位；SP_SCR 環境變數＝暫存夾）
  2) python fieldcheck.py <run 4 碼> [...]
欄位：v@點＝MCU 重算車速（m/s）；emax＝|image_error| 最大；e99＝|e|≥0.99 的格數；in／out＝arc_lat clr_in_cm／clr_out_cm 最小；
st＝送出舵量−1475 的最大（右＋）；gz＝inputs.gz 最小（右彎為負）。"""
import sys
import numpy as np
from common import *

CHECKS = [
    # 名稱, 窗起(地標,偏移), 窗止(地標,偏移), 車速觀測點(地標,偏移)
    ('彎1入口＋彎1', ('pC4e', 8.0), ('C1e', 1.0), ('C1s', 0.0)),
    ('彎2入口＋彎2', ('C1e', 9.0), ('C2e', 0.5), ('C2s', 0.0)),
    ('左彎→彎3入口', ('C2e', 6.6), ('C3s', 0.8), ('C3s', 0.0)),
    ('彎3前段', ('C3s', 0.0), ('C3s', 4.3), ('C3s', 2.0)),
    ('彎3拐角＋尾段', ('C3s', 4.3), ('RAMP', 0.3), ('C3s', 4.3)),
    ('坡頂→虛線轉彎', ('TOP', 0.0), ('DASHe', 0.0), ('DASHs', 0.0)),
    ('轉彎後直線→坡口', ('DASHe', 0.0), ('DOWN', 0.0), ('DOWN', 0.0)),
    ('落地0-1.5', ('LAND8', 0.0), ('LAND8', 1.5), ('LAND8', 1.5)),
    ('彎4後半', ('LAND8', 3.4), ('C4e', 0.0), ('C4e', 0.0)),
    ('主直線前3m', ('C4e', 0.0), ('C4e', 3.0), ('C4e', 3.0)),
]


def get(lap, prev, a):
    if a == 'pC4e': return prev['X'].get('C4e') if prev else None
    return lap['X'].get(a)


def one(run):
    R = load(run); L, _ = laps(R)
    print('=' * 130)
    lt = [(l['lap'], l['t1'] - l['t0']) for l in L if l['lap'] > 0 and l['t1'] is not None]
    print('run-%s 圈時 %s' % (run, '／'.join('L%d %.2f' % x for x in lt)))
    for name, w0, w1, pv in CHECKS:
        rows = []
        for i, lap in enumerate(L):
            if lap['lap'] == 0: continue
            prev = L[i - 1] if i > 0 else None
            a = get(lap, prev, w0[0]); b = get(lap, prev, w1[0]); p = get(lap, prev, pv[0])
            if a is None or b is None: continue
            a += w0[1]; b += w1[1]
            F = [r for r in R if a <= r['x'] < b]
            if len(F) < 3: continue
            vp = None
            if p is not None:
                q = [r for r in R if r['v'] is not None and abs(r['x'] - (p + pv[1])) < 0.15]
                if q: vp = float(np.median([r['v'] for r in q]))
            e = [abs(r['e']) for r in F if r['e'] is not None and r['valid']]
            cin = [r['cin'] for r in F if r['cin'] is not None]; cout = [r['cout'] for r in F if r['cout'] is not None]
            st = [r['st'] - 1475 for r in F if r['st'] is not None]; gz = [r['gz'] for r in F if r['gz'] is not None]
            T = t_at(*__import__('common').XT, b) - t_at(*__import__('common').XT, a)
            rows.append('L%d T %.2f v@%s%+.1f %s emax %s e99 %d in %s out %s st %s gz %s' % (
                lap['lap'], T, pv[0], pv[1], '—' if vp is None else '%.2f' % vp, '%.2f' % max(e) if e else '—', sum(1 for x in e if x >= 0.99),
                '%.0f' % min(cin) if cin else '—', '%.0f' % min(cout) if cout else '—', '%+.0f' % max(st, key=abs) if st else '—', '%.0f' % min(gz) if gz else '—'))
        print('  %s' % name)
        for r in rows: print('     ' + r)


if __name__ == '__main__':
    for run in sys.argv[1:]: one(run)
