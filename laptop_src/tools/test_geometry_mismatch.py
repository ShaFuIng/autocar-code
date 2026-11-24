import sys,copy,json
from pathlib import Path
W=Path(__file__).resolve().parents[1]/'laptop_workbench'
sys.path[:0]=[str(W/'runtimes/opencv-4.11'),str(W)]
from geometry_mismatch_offline import GeometryMismatch
from analysis_jobs import read,write

uid=read(W/'data/experiments/geometry-mismatch-v1/job.json')['job']
d=read(W/'data/analysis'/uid/'result.json')
old=read(W/'data/analysis'/d['baseline_job']/'result.json')
checks={}
# Real isolated disagreement must not trigger the sustained-evidence branch.
checks['isolated_disagreement_kept']=d['models']['B'][141]['A']['mode']=='BASELINE'
checks['same_frame_baseline']=all(a==old['models']['C'][1600+i] for i,a in enumerate(d['models']['A']))
checks['all_pid_disabled']=all(not g['pid_enabled'] for g in d['spec']['groups'])
checks['single_not_slope']=all(a['A']['geometry']['slope_detected'] is None for a in d['models']['B'])
checks['trim_supported_near_only']=all(all(n['y']>=270 and n['source'] not in ('history','single_extrapolated') for n in a['A']['path_samples']) for a in d['models']['B'] if a['A']['mode']=='SINGLE_WIDTH_UNVERIFIABLE')
# Construct persistent current-pair mismatch from a real isolated observation.
fixture=copy.deepcopy(old['models']['C'][1741]['A']);k=GeometryMismatch()
outputs=[k.step(fixture) for _ in range(3)]
checks['confirmed_on_third']=outputs[0]['mode']==outputs[1]['mode']=='BASELINE' and outputs[2]['mode']=='CURRENT_PAIR_MISMATCH'
checks['confirmed_observed_only']=all(n['source']=='double' and n['age_s']==0 for n in outputs[2]['path_samples'])
missing=copy.deepcopy(fixture);missing['boundaries']=[]
checks['missing_pair_resets_votes']=k.step(missing)['geometry']['mismatch_votes']==0
report=dict(checks=checks,passed=all(checks.values()),synthetic_scope='repeated real observation tests confirmation logic only',hardware_output=False)
write(W/'data/experiments/geometry-mismatch-v1/validation.json',report)
print(json.dumps(report));assert report['passed']
