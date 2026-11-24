"""任意起點設定產生器（2026-11-23 使用者定案的規則）。
用法：python mkstart_rule.py --back <車比旗子位置往後幾公尺> [--base base_flag] [--out <資料夾>]
  --base ＝旗子位置（往後 0 m）的四個檔：lap_pos.json、brake_zones.json、lap_ff.json、start_bias.json（start_bias 必須是開著的版本）。
規則：
  1. D ＝ 3.44 ＋ back；back > 0 時少報 0.25 m（實際比輸入近只容許約 0.2 m，遠可到 1.5 m）。可用範圍 back 0～7。
  2. 一律加 lap_pos skip_after_by.C1s 9.0（定位保險：距離輸入錯也不會整圈錯位）。
  3. back > 2.5 m：關掉起步左舵（start_bias enabled false）；back <= 2.5 m：照 base。
     依據 11-23：往後 6 m 有左舵時直線誤差 +0.4→−0.3、前 5 m 油門 30～35；關掉後誤差 ±0.13、1 m 後油門 50。往後 2.5 m 有左舵正常。
輸出四個檔與一行摘要；逐鍵改動由 anystart/tools/mkstart.py 印出。"""
import argparse, hashlib, json, subprocess, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
ap = argparse.ArgumentParser(); ap.add_argument('--back', type=float, required=True); ap.add_argument('--base', default=str(HERE / 'base_flag')); ap.add_argument('--out')
a = ap.parse_args()
if not 0. <= a.back <= 7.: sys.exit('back 必須在 0～7 m')
D = round(3.44 + a.back - (0.25 if a.back > 0 else 0.), 2)
out = Path(a.out) if a.out else HERE / ('back%.1f' % a.back)
r = subprocess.run([sys.executable, str(HERE.parent / 'anystart' / 'tools' / 'mkstart.py'), '--dist', str(D), '--c1s-skip', '9', '--base', a.base, '--out', str(out)], capture_output=True, text=True, encoding='utf8')
print(r.stdout[-2500:]);
if r.returncode: sys.exit(r.stderr)
sb = out / 'start_bias.json'; raw = sb.read_text(encoding='utf8'); j = json.loads(raw)
if j.get('enabled') is not True: sys.exit('base 的 start_bias 不是開著的版本')
if a.back > 2.5:
    j['enabled'] = False; j['note_far_20261123'] = 'mkstart_rule: start more than 2.5 m behind the flag spot -> no left bias (user 2026-11-23).'
    text = json.dumps(j, ensure_ascii=False, indent=1) + '\n'; assert len(text) > 0.5 * len(raw); sb.write_text(text, encoding='utf8', newline='\n')
h = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()[:8]
print('back %.2f m -> D %.2f | 起步左舵 %s | %s' % (a.back, D, '關' if a.back > 2.5 else '開', '  '.join('%s %s' % (f, h(out / (f + '.json'))) for f in ('lap_pos', 'brake_zones', 'lap_ff', 'start_bias'))))
