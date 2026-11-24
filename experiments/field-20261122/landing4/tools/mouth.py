"""slope-mouth events and slope geometry per descent."""
from desc import *
import statistics as S
R = all_desc()
print('run lap | 坡口前 8 格最大誤差(第幾格,俯仰) | jump_guard 保持格數/秒 | down_straight CAP 格 | 坡口 0.5 s 內實送−中心 最大/最小 | 坡口 1 s 內 gz 最小 | 坡長(俯仰<-5→落地) m / s | 落地右限(頂 cap) 格數 | 落地誤差峰值')
Ls = []; caps = 0
for d in R:
    F = d['F']; a = d['a']; im = d['i_mouth'] - a; il = None if d['i_land'] is None else d['i_land'] - a
    es = [(F[n].get('image_error'), n - im, pitch(F[n])) for n in range(im, im + 8) if F[n].get('image_error') is not None]
    em = max(es) if es else (None, None, None)
    jg = [n for n in range(im, im + 40) if n < len(F) and 'held_us' in (F[n].get('jump_guard') or {})]
    jgs = (d['t'][jg[-1]] - d['t'][jg[0]] + 0.035) if jg else 0.
    cap = sum(1 for n in range(im, im + 40) if n < len(F) and g(F[n], 'down_straight', 'mode') == 'CAP')
    w = [n for n in range(im, len(F)) if d['t'][n] - d['t'][im] <= 0.5]
    sm = [sent(F[n]) - C0 for n in w]
    gzs = [g(F[n], 'inputs', 'gz') for n in range(im, len(F)) if d['t'][n] - d['t'][im] <= 1.0 and g(F[n], 'inputs', 'gz') is not None]
    if il is not None:
        L = d['dist'](d['t'][il]) - d['dist'](d['t'][im]); Ls.append(L); Ts = d['t'][il] - d['t'][im]
        nc = sum(1 for n in range(il, len(F)) if d['x'][n] is not None and d['x'][n] <= 1.5 and g(F[n], 'bottom_owner', 'cap') is not None)
        caps += nc > 0
        ep = max([F[n].get('image_error') or 0 for n in range(il, len(F)) if d['x'][n] is not None and d['x'][n] <= 1.5])
    else: L = Ts = nc = ep = None
    print(d['run'], d['lap'], '| %s' % (None if em[0] is None else '%.2f(%d,%.1f)' % em), '| %d/%.2f' % (len(jg), jgs), '| %d' % cap, '| %+.0f/%+.0f' % (max(sm), min(sm)), '| %.0f' % min(gzs),
          '| %s' % (None if L is None else '%.2f / %.2f' % (L, Ts)), '| %s' % nc, '| %s' % (None if ep is None else round(ep, 2)))
print('坡長 mean %.2f sd %.2f min %.2f max %.2f' % (S.mean(Ls), S.pstdev(Ls), min(Ls), max(Ls)), '；落地後 x<=1.5 有被 bottom_owner 上限夾到的落地：%d/23' % caps)
