"""B：今天各輪 (1) 影格真正中斷（frame_seq 跳號或 consumed 間隔 >0.15 s）；(2) 連續無效路徑（失線列）長度與路段；(3) WARMUP 格。"""
import pickle,glob,os,json,collections
from flagsim import TMP
out=dict(frame_gaps=[],streaks=[],warmups=[])
for fn in sorted(glob.glob(os.path.join(TMP,'*.pkl'))):
    run=os.path.basename(fn)[:4];rows=[r for r in pickle.load(open(fn,'rb'))['rows'] if r['con'] is not None]
    last={}
    for r in rows:
        if r['lseg'] is not None:last=dict(lap=r['lap'],lseg=r['lseg'],anchor=r['anchor'],since=r['since'])
        elif last:r.update(last)
    for a,b in zip(rows,rows[1:]):
        dfs=(b['fs']-a['fs']) if (a['fs'] is not None and b['fs'] is not None) else None
        dt=b['con']-a['con']
        if dt>0.15 or (dfs is not None and dfs>4):
            out['frame_gaps'].append(dict(run=run,lap=b['lap'],lseg=b['lseg'],seg=b['seg'],dt=round(dt,3),dfs=dfs,dcap=round(b['cap']-a['cap'],3),inf=b['inf'],stage=b['stage'],reason_after=b['reason']))
    i=0;n=len(rows)
    while i<n:
        if rows[i]['t'] is None:
            j=i
            while j<n and rows[j]['t'] is None:j+=1
            dur=rows[j-1]['con']-rows[i]['con']+0.033
            nxt=rows[j]['reason'] if j<n else 'END'
            gap_to_next=(rows[j]['cap']-rows[i-1]['cap']) if (j<n and i>0) else None
            if dur>=0.2 or nxt=='WARMUP':
                out['streaks'].append(dict(run=run,lap=rows[i]['lap'],lseg=rows[i]['lseg'],seg=rows[i]['seg'],anchor=rows[i]['anchor'],since=rows[i]['since'],n=j-i,dur=round(dur,3),
                    reasons=dict(collections.Counter(r['reason'] for r in rows[i:j])),next=nxt,valid_gap=None if gap_to_next is None else round(gap_to_next,3),route=rows[i]['route']))
            i=j
        else:i+=1
    for k,r in enumerate(rows):
        if r['reason']=='WARMUP':
            prev=rows[k-1] if k else None
            out['warmups'].append(dict(run=run,lap=r['lap'],lseg=r['lseg'],seg=r['seg'],anchor=r['anchor'],since=r['since'],held=r['warm'],prev_reason=None if prev is None else prev['reason'],t_from_start=round(r['con']-rows[0]['con'],2)))
json.dump(out,open(os.path.join(TMP,'..','proof','B_gaps.json'),'w'),ensure_ascii=False,indent=0)
print('真正影格中斷（>0.15 s 或跳號>4）:',len(out['frame_gaps']))
for g in out['frame_gaps'][:30]:print('  ',g)
W=out['warmups'];print('WARMUP 格:',len(W),'；起跑首格以外:',sum(1 for w in W if w['prev_reason'] is not None))
print(' WARMUP 前一格原因:',collections.Counter(w['prev_reason'] for w in W))
print(' WARMUP 路段(lseg):',collections.Counter(w['lseg'] for w in W if w['prev_reason'] is not None))
S=out['streaks'];print('失線列 >=0.2 s 或接 WARMUP:',len(S))
print(' 依路段:',collections.Counter(s['lseg'] for s in S))
print(' 接 WARMUP 的:',sum(1 for s in S if s['next']=='WARMUP'),' 依路段:',collections.Counter(s['lseg'] for s in S if s['next']=='WARMUP'))
print(' 結束輪次的(END):',[ (s['run'],s['lseg'],s['dur']) for s in S if s['next']=='END'])
