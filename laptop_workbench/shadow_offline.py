"""Offline comparison specification, shared by the existing analysis registry."""
import copy
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent/'web'))
from visual_pid import validate

ALGORITHMS = ['junction-k-lane-v3','road-entrance-v3.2','junction-k-entrance-v2','junction-k-mask-v1','junction-k-v1','junction-straight-v1','road-memory-v3.1', 'legacy9', 'temporal-v2', 'calibrated-v7', 'entrance-image-v1', 'entrance-temporal-v1', 'entrance-v7-v1', 'entrance-image-v2', 'entrance-temporal-v2', 'entrance-v7-v2','road-image-v3','road-memory-v3','road-v7-v3']

def specification(app, payload, count):
    start, end, stride = (payload.get(k, default) for k,default in
                          [('start',0),('end',count-1),('stride',1)])
    if any(type(v) is not int for v in (start,end,stride)) or not 0 <= start <= end < count or not 1 <= stride <= 120:
        raise ValueError('影格區間或抽幀設定無效')
    pose = payload.get('pose','unknown')
    if pose not in ('matched','different','unknown'):
        raise ValueError('相機姿態標記無效')
    groups = payload.get('groups')
    if not isinstance(groups,list) or not 1 <= len(groups) <= 3:
        raise ValueError('請設定1～3組比較')
    result=[]
    for i,g in enumerate(groups):
        algorithm=g.get('algorithm')
        if algorithm not in ALGORITHMS:raise ValueError('路徑算法無效')
        model=g.get('model','v4-640')
        if model not in ('v4-640','v4-960'):raise ValueError('影子比較目前支援V4 640／960')
        doc=app.store.get('visual-pid')
        revision=g.get('pid_revision',doc['rev'])
        if revision != doc['rev']:
            with app.store.lock:
                row=app.store.db.execute("SELECT body FROM history WHERE key='visual-pid' AND rev=?",(revision,)).fetchone()
            if not row:raise ValueError('找不到指定的PID保存版本')
            doc={'rev':revision,'data':json.loads(row[0])}
        if not doc['rev']:raise ValueError('請先在04保存PID參數；06不建立第二份控制參數')
        params=validate(doc['data']['parameters'])
        calibration=None
        if algorithm in ('calibrated-v7','entrance-v7-v1','entrance-v7-v2','road-v7-v3'):
            calibration=json.loads((ROOT.parent/'web/calibration_v7.json').read_text(encoding='utf-8'))
            width=g.get('lane_width_cm',calibration['lane_width_cm'])
            if type(width) not in (int,float) or not math.isfinite(width) or not 10<width<200:
                raise ValueError('跑道寬度須介於10～200cm')
            calibration['lane_width_cm']=width
            matrix=g.get('matrix')
            if matrix is not None:
                if (not isinstance(matrix,list) or len(matrix)!=3 or
                    any(not isinstance(row,list) or len(row)!=3 for row in matrix) or
                    any(type(v) not in (int,float) or not math.isfinite(v) for row in matrix for v in row)):
                    raise ValueError('投影矩陣須為3×3有限數值')
                a,b,c,d,e,f,h,j,k=[v for row in matrix for v in row]
                if abs(a*(e*k-f*j)-b*(d*k-f*h)+c*(d*j-e*h))<1e-12:raise ValueError('投影矩陣不可退化')
                calibration['matrix']=matrix
        result.append(dict(id=chr(65+i),label=str(g.get('label',chr(65+i)))[:80],
            algorithm=algorithm,model=model,calibration=calibration,
            pid_enabled=g.get('pid_enabled',True) is True,pid_revision=doc['rev'],
            parameters=copy.deepcopy(params)))
    return dict(version=1,start=start,end=end,stride=stride,pose=pose,groups=result,
                warmup='區間起點冷啟動；逐張結果保留暖機，不以區間外資料預熱',
                hardware_output=False)
