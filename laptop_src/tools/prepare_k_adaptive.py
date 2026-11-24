from pathlib import Path
import shutil
root=Path(__file__).resolve().parents[1];out=root/'laptop_workbench/data/experiments/k-adaptive'
payload=out/'payload/web';payload.mkdir(parents=True,exist_ok=True)
s=(out/'before/live_pid.py').read_text(encoding='utf-8')
s=s.replace('from visual_pid import PID,validate,path_error,path_features','from visual_pid import PID,validate,path_error,path_features\nfrom k_adaptive_pid import KAdaptivePID')
s=s.replace('return dict(control_publish_version=2,','return dict(k_sampling_version="k-adaptive-r1",control_publish_version=2,')
s=s.replace('            if self.path_algorithm in ROAD_ALGORITHMS:\n                from road_live import RoadController\n                self.controller=RoadController(params)','            if self.path_algorithm=="junction-k-lane-v3":\n                self.controller=KAdaptivePID(params)\n            elif self.path_algorithm in ROAD_ALGORITHMS:\n                from road_live import RoadController\n                self.controller=RoadController(params)')
s=s.replace('            road_preview=None\n            if self.path_algorithm in ROAD_ALGORITHMS:','            road_preview=None\n            if self.path_algorithm=="junction-k-lane-v3":\n                from visual_pid import DEFAULTS\n                road_preview=KAdaptivePID(dict(DEFAULTS,row_ratio=self.row_ratio))\n            elif self.path_algorithm in ROAD_ALGORITHMS:')
s=s.replace('                        preview=mark_sample(preview,sample_row)','                        if preview is not None:\n                            from road_live import mark_sample\n                            preview=mark_sample(preview,sample_row)')
s=s.replace('sample_reason=proposal[\'sample_reason\'])','sample_reason=proposal[\'sample_reason\'],sampling_version=proposal.get(\'sampling_version\'),derivative_rebased=proposal.get(\'derivative_rebased\'),sample_clamped=proposal.get(\'sample_clamped\'),sample_bounds=proposal.get(\'sample_bounds\'))')
assert s.count('self.controller=KAdaptivePID(params)')==1
assert s.count('road_preview=KAdaptivePID(')==1
assert "now-captured>self.run['parameters']['max_gap']" in s
compile(s,'live_pid.py','exec');(payload/'live_pid.py').write_text(s,encoding='utf-8')
shutil.copy2(root/'web/k_adaptive_pid.py',payload/'k_adaptive_pid.py')
print('Exact remote-based candidate prepared')
