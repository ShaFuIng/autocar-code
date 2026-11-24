"""Append-only human reviews, separate from immutable automatic paths."""
import json, math, re, time

STAGES=('未標記','接近','進入','穿越','離開')
ISSUES=('未確認','未見問題','白線漏辨識','橫線誤當邊界','選錯出口','路徑偏左','路徑偏右','路徑跳動','太早切換','太晚切換','路口漏判','路口誤判','未接回道路','無法判斷')

def catalog(app):
    return json.loads((app.store.root/'experiments/junction-cases/cases.json').read_text(encoding='utf8'))

def case_for(app,key):
    return next((c for c in catalog(app)['cases'] if c['id']==key),None) or fail('未知路口案例')

def fail(message):raise ValueError(message)

def reviews(app,key):
    case_for(app,key)
    return app.store.events('junction-review-'+key)

def save(app,p):
    case=case_for(app,p.get('case'))
    if p.get('job')!=case['job']:fail('結果版本不符，請重新載入')
    for k in ('start','end','anchor'):
        if type(p.get(k)) is not int or not 0<=p[k]<case['frames']:fail('影格超出案例範圍')
    if not p['start']<=p['anchor']<=p['end']:fail('標註影格必須位於評語區間內')
    if p.get('stage') not in STAGES or p.get('issue') not in ISSUES:fail('標註分類無效')
    if p.get('group') not in ('A','B','C','D','E','F','G','H','I','J','both','all'):fail('比較組無效')
    if p.get('group') in ('C','all') and (not case.get('candidate_job') or p.get('candidate_job')!=case['candidate_job']):fail('C版結果已更新，請重新載入')
    if p.get('group') in ('D','all') and (not case.get('revision_job') or p.get('revision_job')!=case['revision_job']):fail('D版結果已更新，請重新載入')
    if p.get('group') in ('E','all') and (not case.get('memory_job') or p.get('memory_job')!=case['memory_job']):fail('E版結果已更新，請重新載入')
    if p.get('group')=='F' and (not case.get('corridor_job') or p.get('corridor_job')!=case['corridor_job']):fail('F版結果版本不符或本案例未執行F版')
    if p.get('group')=='G' and (not case.get('fused_job') or p.get('fused_job')!=case['fused_job']):fail('G版結果版本不符或本案例未執行G版')
    if p.get('group')=='H' and (not case.get('left_job') or p.get('left_job')!=case['left_job']):fail('H版結果版本不符或本案例未執行H版')
    if p.get('group')=='I' and (not case.get('reference_job') or p.get('reference_job')!=case['reference_job']):fail('I版結果版本不符或本案例未執行I版')
    if p.get('group')=='J' and (not case.get('geometry_job') or p.get('geometry_job')!=case['geometry_job']):fail('J版結果版本不符或未執行')
    if p.get('intent')!='straight':fail('本案例任務為直行')
    if p.get('visibility') not in ('visible','unclear'):fail('出口可見性無效')
    points=p.get('polygon')
    if not isinstance(points,list) or len(points)>32:fail('出口區域最多32點')
    if p['visibility']=='visible' and len(points)<3:fail('請至少點選三點圈出出口，或選出口不清楚')
    if p['visibility']=='unclear' and points:fail('出口不清楚時不保存推測區域')
    for pt in points:
        if not isinstance(pt,list) or len(pt)!=2:fail('座標無效')
        if any(type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<=1 for v in pt):fail('座標須位於影像範圍')
    if points:
        area=abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1])))/2
        if area<1e-6:fail('出口區域不可為一直線')
    if not isinstance(p.get('notes'),str) or len(p['notes'])>5000:fail('評語最多5000字')
    if not isinstance(p.get('id'),str) or not re.fullmatch('[0-9a-f]{32}',p['id']):fail('標註識別碼無效')
    item={k:p[k] for k in ('id','case','job','start','end','anchor','stage','issue','group','intent','visibility','polygon','notes')}
    if p.get('group') in ('C','all'):item['candidate_job']=case['candidate_job']
    if p.get('group') in ('D','all'):item['revision_job']=case['revision_job']
    if p.get('group') in ('E','all'):item['memory_job']=case['memory_job']
    if p.get('group')=='F':item['corridor_job']=case['corridor_job']
    if p.get('group')=='G':item['fused_job']=case['fused_job']
    if p.get('group')=='H':item['left_job']=case['left_job']
    if p.get('group')=='I':item['reference_job']=case['reference_job']
    if p.get('group')=='J':item['geometry_job']=case['geometry_job']
    result=app.analysis.result(case['job'])
    frame=result['frames'][p['anchor']]
    item.update(created=time.time(),source='human_review',geometry_scope='anchor_frame_only',coordinate_space='normalized_image',raw_sha256=frame['sha256'],source_video_frame=case['source_frame_range'][0]+p['anchor'],player_s=case['player_range_s'][0]+p['anchor']/catalog(app)['fps'],training_truth=False)
    with app.store.lock:
        existing=next((r for r in reviews(app,case['id']) if r['id']==p['id']),None)
        if existing:
            if any(existing[k]!=item[k] for k in p if k in item):fail('識別碼已用於不同標註')
            return existing
        app.store.event('junction-review-'+case['id'],item)
    return item
