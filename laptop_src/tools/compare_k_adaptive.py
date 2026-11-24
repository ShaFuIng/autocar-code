import sys,json,time,hashlib
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'laptop_workbench/data/experiments/k-adaptive'
sys.path.insert(0,str(out/'before'));sys.path.insert(0,str(out/'payload/web'))
from visual_pid import PID,path_error
from k_adaptive_pid import KAdaptivePID,continuous_paths
e=json.loads((root.parent/'firmware_builds/first_fault_v1/run-db7089a3/evidence.json').read_text(encoding='utf-8'))
logs=[json.loads(l) for k,v in e['files'].items() if k.endswith('/pid.jsonl') for l in v.splitlines()]
params=logs[0]['parameters'];assert params['max_gap']==.4
cases={'tunnel':[dict(index=x['vision']['seq'],captured=x['vision']['capture_at'],path=x['vision']['path'],mode=x['vision']['path_diagnostic']['mode'],valid=bool(x['vision']['path'])) for x in logs if x.get('kind') in ('sample','stop_diagnostic')]}
cached=json.loads((root/'laptop_workbench/data/experiments/k-parallel/comparison-full.json').read_text())
for name in ('intersection1','intersection2'):cases[name]=cached[name]['background']['rows']
summary={};outputs={}
for name,rows in cases.items():
 old=PID(params);new=KAdaptivePID(params);results=[];last=None
 for r in rows:
  path=r['path'] if r['valid'] else [];t=r['captured']
  a=old.step_path(t,path);start=time.perf_counter();b=new.step_path(t,path);elapsed=(time.perf_counter()-start)*1000
  same=abs(b['sample_row_ratio']-.6)<1e-8
  results.append(dict(index=r['index'],captured=t,mode=r['mode'],baseline=a,candidate=b,ms=elapsed,path=path))
  if b['valid']:assert any(c[0][1]<=b['sample_row_ratio']*719<=c[-1][1] for c in continuous_paths(path))
  if last and last['candidate']['valid'] and b['valid']:
   # Existing final backlash/steering rate bound must remain in force.
   assert abs(b['steer_us']-last['candidate']['steer_us'])<=params['steer_rate']*(t-last['captured'])+1e-6
  last=results[-1]
 failures=lambda key:sum(not r[key]['valid'] and r[key]['reason']!='WARMUP' for r in results)
 unchanged=[r for r in results if r['candidate']['sample_reason']=='REQUESTED' and r['baseline']['valid'] and r['candidate']['valid']]
 summary[name]=dict(frames=len(rows),baseline_invalid=failures('baseline'),candidate_invalid=failures('candidate'),adjusted=sum(r['candidate']['sample_reason']=='ADAPTED_NEAR' for r in results),max_compute_ms=max(r['ms'] for r in results),max_command_step_us=max([abs(b['candidate']['steer_us']-a['candidate']['steer_us']) for a,b in zip(results,results[1:]) if a['candidate']['valid'] and b['candidate']['valid']] or [0]),path_and_mode_unchanged=True)
 outputs[name]=results
# Gap rejection and pure coordinate change must not create derivative spikes.
c=KAdaptivePID(params);path=[[500,y] for y in range(200,701,8)];c.step_path(0,path);c.step_path(.1,path);p=c.step_path(.2,[[500,y] for y in range(444,669,8)]);assert abs(p['d'])<1e-9 and p['derivative_rebased']
gap=[[600,400],[600,408],[600,500],[600,508]];p=c.step_path(.3,gap);assert p['sample_row_ratio']*719>500
assert not c.step_path(.4,[])['valid'];assert not c.step_path(1,path)['valid']
report=dict(summary=summary,parameters=params,focus=[r for r in outputs['tunnel'] if r['index']>=368],limitations=['Recorded-path replay, not closed-loop driving','No recognition or K switching changes; identical supplied paths/modes','Sample slew .2 fraction/sec inherited from A/B; hard clamp only into actual support; existing steering slew retained'],tests=['no interpolation across gaps','row shift derivative rebased','empty path rejected','400ms time limit preserved','final steering slew bounded'])
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');(out/'comparison.json').write_text(json.dumps(outputs),encoding='utf-8')
print(json.dumps(summary));print('FOCUS',[(r['index'],r['candidate']['sample_row_ratio'],r['baseline']['valid'],r['candidate']['valid'],r['candidate'].get('steer_us'),r['candidate'].get('d')) for r in outputs['tunnel'][-5:]])
