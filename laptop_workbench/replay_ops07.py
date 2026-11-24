"""Offline comparison of recorded route inputs, not a GPU/closed-loop replay."""
import ast
import importlib.util
import json
from pathlib import Path
import types
from build_ops07 import ROOT, OUT, PACKAGES
import ops07_runtime


def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def method(path,name,scope):
    tree=ast.parse(path.read_text(encoding='utf8'))
    node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),scope)
    return scope[name]


def run():
    reports=[]
    for pkg,uid in zip(PACKAGES,('7fbfa3bc','46db59ec','97edc5c5')):
        baseline=ROOT/'data/ops07-packages'/pkg/'web';candidate=OUT/'candidates'/pkg/'web'
        a=load(baseline/'curve_map.py','original_cm').CurveMap()
        b=load(candidate/'curve_map.py','candidate_cm').CurveMap()
        rec=next((ROOT/'data/orin/records/manual').glob(uid+'*'))
        version=json.loads((rec/'current.json').read_text(encoding='utf8'))
        # Every current pointer has a version field; inspect if schema changes.
        files=list(rec.rglob('pid.jsonl'))
        if len(files)!=1:raise ValueError('Choose current synced record version explicitly: '+str(rec))
        p=files[0];count=0;checks=0;previous=None;transitions=[];last=None
        scope_a={'time':types.SimpleNamespace(monotonic=lambda:0)}
        scope_b={'time':scope_a['time'],'ops07_support':ops07_runtime}
        check_a=method(baseline/'live_pid.py','check_safety',scope_a)
        check_b=method(candidate/'live_pid.py','check_safety',scope_b)
        for line in p.open(encoding='utf8'):
            record=json.loads(line);pr=record.get('proposal') or {};cm=pr.get('curve_map') or {}
            if record.get('kind')!='sample' or not cm:continue
            now=pr.get('t');gyro=(pr.get('yaw_inner') or {}).get('gz');vision=record.get('vision') or {}
            dr=vision.get('dash_route') or {};route=dr.get('route');pitch=dr.get('pitch_rel_deg')
            if type(now) not in (int,float):continue
            distance=cm.get('dist_m');speed=None
            if previous and distance is not None and now>previous[0]:speed=max(0.,(distance-previous[1])/(now-previous[0]))
            if distance is not None:previous=(now,distance)
            args=(now,gyro,route,route=='WAIT_CLIMB' and pitch is not None and abs(pitch)<=3)
            kw=dict(near=pr.get('image_error'),far=(pr.get('turn_preview') or {}).get('far_projected_error'),speed=speed)
            if a.step(*args,**kw)!=b.step(*args,**kw):raise AssertionError(pkg+' route diverged')
            transition=(route,cm.get('seg'),cm.get('idx'))
            if transition!=last:transitions.append(transition);last=transition
            # Both original modes, using identical supplied browser freshness.
            # This does not invent missing recorded heartbeat traffic.
            scope_a['time'].monotonic=lambda:now
            for mode in ('browser_lease','orin_timed'):
                outcomes=[]
                for check in (check_a,check_b):
                    ended=[];dummy=types.SimpleNamespace(lock=__import__('threading').RLock(),
                        run=dict(state='running',execution_mode=mode,deadline=now+1,parameters={'max_gap':.4}),
                        latest={'capture_at':vision.get('capture_at',now)},browser_at=now,
                        manual=types.SimpleNamespace(busy=lambda:True),end=ended.append)
                    check(dummy);outcomes.append(ended)
                if outcomes[0]!=outcomes[1]:raise AssertionError('Legacy safety changed')
                checks+=1
            count+=1
        reports.append(dict(package=pkg,record=str(p.relative_to(ROOT)),frames=count,
                            curve_map_comparison='identical',legacy_safety_comparisons=checks,
                            observed_route_transitions=transitions))
    report=dict(scope='CPU route component and legacy safety branch replay',results=reports,
                limitations=['Gyro and route inputs are rounded recorded observations.',
                             'Not full LivePID/GPU/camera/UART or closed-loop reproduction.',
                             'No physical steering-center, stop distance or 3 m calibration verification.'])
    (OUT/'replay.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    return reports


if __name__=='__main__':
    for row in run():print(row['package'],row['frames'],row['legacy_safety_comparisons'])
