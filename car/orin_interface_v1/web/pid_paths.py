"""Reviewed path registry; independent of recognition weights."""
ROAD_ALGORITHMS = ('road-image-v3', 'road-memory-v3')
ROAD_FILES = ('road_segments_v3.py', 'entrance_paths_v2.py', 'entrance_paths.py', 'temporal_lane.py', 'curve_replay.py', 'ground_calibration.py')

PATHS = {
    'camera-pair-v2-junction-flag': dict(label='正式起點（平地旗子）起跑（同整趟控制）· 平地出發→01…09爬坡→虛線段／虛線轉彎→下坡→11；控制與整趟版本相同（實驗PID）',control=True,preview=True,version=1),   # 2026-11-05 flag-start-r1: same control as camera-pair-v2-junction (route starts WAIT_CLIMB), separate name for flag-start runs
    'camera-pair-v2-junction-crest-full': dict(label='坡頂起跑（同整趟控制）· 一開始就開啟虛線段；虛線轉彎、雙車道車道段、下坡與整趟版本相同（實驗PID）',control=True,preview=True,version=1),   # 2026-11-03: crest start only; turn / two-lane / downhill as the full run
    'camera-pair-v2-junction-uphill': dict(label='上坡起跑測試 · 等爬坡→虛線左轉→雙車道虛線延伸／邊線→下坡入口，與起點版同一控制（實驗PID）',control=True,preview=True,version=1),   # 2026-11-03: same control as camera-pair-v2-junction, separate name for uphill-start test runs
    'camera-pair-v2-junction-crest': dict(label='坡頂起跑測試 · 新視角雙邊界＋路口通道／路線窗口直接開啟／雙車道跟左車道（實驗PID）',control=True,preview=True,version=1),
    'camera-pair-v2-junction': dict(label='新視角雙邊界 r2＋路口通道切換（實驗PID）· 基準控制列無路徑時暫用通道／預測不控制',control=True,preview=True,version=1),
    'camera-pair-v2': dict(label='新視角雙邊界 r2 · 畫面外中心／固定取樣／實驗PID',control=True,preview=True,version=2),
    'camera-pair-v1': dict(label='新視角雙邊界 r1 · 實驗PID／單線急彎易失效／路口未修',control=True,preview=True,version=1),
    'preview-error-v1': dict(label='近遠共同目標 r1 · 彎道候選／路口未修／外移0',control=True,preview=True,version=1),
    'straight-bearing-v1': dict(label='單目標方向 r1 · 實驗PID／路口未修／外移0',control=True,preview=True,version=1),
    'straight-connector-v1': dict(label='即時綠線 v1.2 · 邊界追蹤／短暫預測／曲率轉向／外移0',control=True,preview=True,version=1),
    'standalone-line-v3': dict(label='獨立線估計 v3 · 提早轉向／下一岔口直行一次／外移設0',control=True,preview=True,version=3),
    'standalone-line-v2': dict(label='獨立線估計 v2.1 · 彎中持續轉向／實驗PID／外移設0',control=True,preview=True,version=2),
    'standalone-line-v1': dict(label='獨立線估計 v1 · 實驗PID／無K／外移設0',control=True,preview=True,version=1),
    'junction-k-integrated-r2': dict(label='K 完整整合 r2 · 暖光校色／可信段／歷史補缺口', control=True, preview=True, version=4),
    'junction-k-lane-v3': dict(label='K 完整路口融合＋A接回 · 共用車道 r1', control=True, preview=True, version=3),
    'road-image-v3': dict(label='A v3 分段雙／單線 · 動態取樣PID', control=True, preview=True, version=3),
    'road-memory-v3': dict(label='B v3 分段＋對齊記憶 · 動態取樣PID', control=True, preview=True, version=3),
    'legacy9': dict(label='原9列雙線中心 · 既有實車PID', control=True, preview=True, version=1),
    'temporal-v2': dict(label='v2 整條邊界＋對齊歷史 · 僅預覽', control=False, preview=True, version=2),
    'curve-v1': dict(label='v1 舊投影候選 · 僅06離線比較', control=False, preview=False, version=1),
    'single-left': dict(label='沿左線等距 · 僅06離線比較', control=False, preview=False, version=1),
    'single-right': dict(label='沿右線等距 · 僅06離線比較', control=False, preview=False, version=1),
    'boundary-priority': dict(label='雙線優先／單線備援 · 僅06離線比較', control=False, preview=False, version=1),
    'calibrated-v7': dict(label='校正v7 雙線／單線 · 候選PID', control=True, preview=True, version=7),
}

def selected(value, control=False):
    if value not in PATHS:
        raise ValueError('Unknown path algorithm')
    spec = PATHS[value]
    if not spec['preview'] or (control and not spec['control']):
        raise ValueError('此路徑算法尚未通過即時控制驗證；不可啟動實車')
    return spec
