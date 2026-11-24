"""7afb 上段：左＝原始畫面＋找虛線用的白色塊（黃點），右＝同一時刻的模型遮罩。"""
import sys, json, base64, importlib.util, cv2, numpy as np
from pathlib import Path
EXP = Path(__file__).resolve().parents[1]; SNAP = EXP / 'orin-web-snapshot-20261117c'; RUN = EXP / 'run-7afb-20261117'
sys.path.insert(0, str(SNAP))
spec = importlib.util.spec_from_file_location('dr', SNAP / 'dash_route.py'); DR = importlib.util.module_from_spec(spec); spec.loader.exec_module(DR)
FR = [json.loads(l) for l in open(RUN / 'frames_002.jsonl')]
S = []
for l in open(RUN / 'pid.jsonl', encoding='utf8'):
    d = json.loads(l)
    if d.get('kind') == 'sample' and d['vision'].get('mask_png'): S.append((d['vision']['capture_at'], d['vision']))
print('capture_at range', S[0][0], S[-1][0], 'video', FR[0]['capture_monotonic'], FR[-1]['capture_monotonic'])
cap = cv2.VideoCapture(str(RUN / 'video_002.avi'))
tiles = []
for label, t in [(a.split('=')[0], float(a.split('=')[1])) for a in sys.argv[2:]]:
    f = min(FR, key=lambda o: abs(o['capture_monotonic'] - t)); cap.set(cv2.CAP_PROP_POS_FRAMES, f['frame']); ok, img = cap.read()
    img = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA)
    ta, v = min(S, key=lambda s: abs(s[0] - f['capture_monotonic']))
    m = cv2.imdecode(np.frombuffer(base64.b64decode(v['mask_png']), np.uint8), cv2.IMREAD_GRAYSCALE); m = cv2.resize(m, (640, 360), interpolation=cv2.INTER_NEAREST)
    B = DR.dash_blocks(DR.white(img)); a = img.copy()
    for b in B: cv2.circle(a, (int(b[0]), int(b[1])), 4, (0, 255, 255), -1)
    mm = cv2.cvtColor(((m > 0) * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    dr = v.get('dash_route') or {}
    cv2.putText(a, '%s t=%.2f blocks=%d' % (label, f['capture_monotonic'], len(B)), (5, 20), 0, .55, (0, 255, 0), 2)
    cv2.putText(mm, 'model mask dt=%.2fs dash=%s' % (ta - f['capture_monotonic'], dr.get('reason') or dr.get('mode')), (5, 20), 0, .55, (0, 255, 0), 2)
    tiles.append(np.hstack([a, mm])); print(label, f['capture_monotonic'], 'mask white px', int((m > 0).sum()), 'blocks', len(B), dr.get('reason'))
cv2.imwrite(sys.argv[1], np.vstack(tiles))
