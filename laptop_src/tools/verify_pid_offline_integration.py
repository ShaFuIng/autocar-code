"""Reproducible local-only acceptance. No remote, serial, or real control imports."""
import collections
import hashlib
import json
import sys
import unittest
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tests'), str(ROOT/'web'), str(ROOT/'laptop_workbench')]
from visual_pid import PID, DEFAULTS
import test_pid_stream_safety as stream


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    suite = unittest.TestSuite()
    for module in ('test_live_pid', 'test_pid_path_worker', 'test_pid_stream_safety',
                   'test_temporal_lane'):
        suite.addTests(unittest.defaultTestLoader.loadTestsFromName(module))
    stream.EVIDENCE.clear()
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    folder = ROOT/'laptop_workbench/data/experiments/pid-offline-integration'
    folder.mkdir(exist_ok=True)
    replays = []
    source = ROOT/'laptop_workbench/data/experiments/curve-v2'
    for file in sorted(source.glob('*/result.json')):
        data = json.loads(file.read_text(encoding='utf-8'))
        pid = PID(dict(DEFAULTS, center_us=1360.))
        rows = []
        reasons = collections.Counter()
        # Fixed recorded paths; cannot simulate a changed physical trajectory.
        for frame in data['frames']:
            path = [[x*2, y*2] for x, y in frame['new']['image_path']]
            if not frame['new']['valid']:
                path = []
            proposal = pid.step_path(frame['t'], path)
            assert proposal['hardware_output'] is False
            assert proposal['steer_us'] is None or 940 <= proposal['steer_us'] <= 1900
            reasons[proposal['reason']] += 1
            rows.append(dict(frame=frame['frame'],
                             source_valid=frame['new']['valid'], **proposal))
        target = folder/(file.parent.name+'.json')
        target.write_text(json.dumps(rows, ensure_ascii=False, allow_nan=False), encoding='utf-8')
        replays.append(dict(run=file.parent.name, frames=len(rows),
                            source_sha256=digest(file), output_sha256=digest(target),
                            source=str(file.relative_to(ROOT)), reasons=dict(reasons),
                            proposals=sum(r['valid'] for r in rows)))
    sources = ['web/live_pid.py', 'web/live_model_worker.py', 'web/pid_paths.py',
               'web/visual_pid.py', 'laptop_workbench/temporal_lane.py',
               'tests/test_pid_stream_safety.py', 'tests/test_pid_path_worker.py',
               'tools/verify_pid_offline_integration.py']
    report = dict(title='新路徑算法：離線與假控制端接入驗證',
                  checked_at=datetime.now(timezone.utc).isoformat(),
                  passed=result.wasSuccessful(), tests=result.testsRun,
                  failures=len(result.failures), errors=len(result.errors),
                  cases=stream.EVIDENCE, replays=replays,
                  source_sha256={name:digest(ROOT/name) for name in sources},
                  hardware_output=False, deployed_to_orin=False,
                  candidate_control_enabled=False,
                  fixes=[
                      '拒絕倒退影格編號與未遞增擷取時間；拒絕非有限或未來擷取時間。',
                      '模型握手核對身分，逐影格核對權重雜湊與執行任務鎖定值。',
                      '取消預覽後不再送出剛取得的影格；明確以UTF-8讀取模型清單。'],
                  method=[
                      '真實 LivePID._loop、TemporalLane 與 PID；假相機、記憶體模型回覆與假控制端，攔截所有子程序啟動。',
                      '候選 running 狀態只在測試fixture中建立，用來驗證目前被實車禁用擋住的控制分支；未修改算法權限或呼叫實車start。',
                      '串流測試使用合成遮罩；另以既有三段真實影片保存的temporal-v2路徑重算PID，保存逐影格提案與來源雜湊。',
                      '假控制端驗證停止請求與停止後不再派送；既有manual測試另驗證零輸出起步與停止命令，不能代替控制板實體驗證。'],
                  limitations=[
                      '本次修補僅在筆電原始碼，尚未部署Orin；Orin沿用先前版本。',
                      '固定影片重播不是閉迴路模擬，提案有效比例不是辨識準確率或行駛成功率。',
                      '合成時鐘延遲注入驗證拒絕邏輯，不是GPU、網路或控制板實際延遲測量。',
                      '實景單線／彎道品質、完整端到端延遲、實體轉向與停止仍待驗證；temporal-v2維持control=false。',
                      '舊影片相機姿態與目前不同，僅驗證1280×720影像座標接入；未套用v7公分標定。'])
    (folder/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2,
                                                allow_nan=False), encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'], tests=report['tests'],
                          stream_cases=len(report['cases']),
                          replay_frames=sum(r['frames'] for r in replays))))
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
