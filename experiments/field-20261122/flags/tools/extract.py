"""逐行串流讀 run-*/pid.jsonl，抽出每格精簡欄位存成 pickle（tmp/<run>.pkl）。"""
import json,sys,os,pickle,glob
from multiprocessing import Pool
EXP='D:/OneDrive/Desktop/自走車/Orin_Interface_v1/laptop_workbench/data/experiments/'
OUT=os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','tmp')
KEEP=['arc2_c3','lap_ff','lap_ff4','land4','bottom_owner','bottom_guide','arc2_land','down_straight','i_ramp','curve_ff']
def small(d):
    if not isinstance(d,dict):return d
    return {k:(v if not isinstance(v,(dict,list)) or k in('band',) else None) for k,v in d.items()}
def one(run):
    fn=EXP+f'run-{run}-20261122/pid.jsonl'
    rows=[];meta={}
    for l in open(fn,encoding='utf-8'):
        try:d=json.loads(l)
        except Exception:continue
        k=d.get('kind')
        if k!='sample':
            meta.setdefault(k,[]).append({x:d.get(x) for x in ('reason','at','started','wall') if x in d});continue
        v=d.get('vision') or {};p=d.get('proposal') or {};dr=v.get('dash_route') or {}
        lp=p.get('lap_pos') or {};ib=p.get('imu_boost') or {};inp=p.get('inputs') or {};sa=p.get('speed_adapt') or {}
        r=dict(wall=d.get('wall'),con=v.get('consumed_at'),cap=v.get('capture_at'),fs=v.get('frame_seq'),inf=v.get('inference_ms'),stage=v.get('stage_ms'),
               vvalid=v.get('valid'),pseq=d.get('pid_input_seq'),flags=[x for x,y in d.items() if y is True and x not in('command_dispatched',)],
               route=dr.get('route'),dpr=dr.get('pitch_rel_deg'),dr_age=dr.get('imu_age_s'),dr_reason=dr.get('reason') if 'pitch_rel_deg' not in dr else None,
               fdg=(v.get('flat_dip_guard') or {}).get('raw_pitch_rel_deg'),rel=(v.get('downhill_release') or {}).get('level_s'),
               t=p.get('t'),reason=p.get('reason'),owner=p.get('owner'),pwm=p.get('pwm'),steer=p.get('steer_us'),err=p.get('image_error'),seg=p.get('seg'),
               ibp=ib.get('pitch_rel_deg'),ibt=ib.get('imu_t'),gz=inp.get('gz'),enc=inp.get('enc_b'),
               lap=lp.get('lap'),lseg=lp.get('seg'),anchor=lp.get('anchor'),since=lp.get('since_anchor_m'),odo=lp.get('odo_m'),s_m=lp.get('s_m'),nxt=lp.get('next'),lost=lp.get('lost'),
               sa_reason=sa.get('reason'),sa_level=sa.get('level'),warm=p.get('warmup_held'))
        for kk in KEEP:
            if kk in p:r[kk]=small(p[kk])
        rows.append(r)
    os.makedirs(OUT,exist_ok=True)
    pickle.dump(dict(rows=rows,meta=meta),open(os.path.join(OUT,run+'.pkl'),'wb'))
    return run,len(rows)
if __name__=='__main__':
    runs=sys.argv[1:] or sorted(os.path.basename(x)[4:8] for x in glob.glob(EXP+'run-*-20261122'))
    done=set(os.path.basename(x)[:4] for x in glob.glob(os.path.join(OUT,'*.pkl')))
    runs=[r for r in runs if r not in done]
    with Pool(6) as pool:
        for r,n in pool.imap_unordered(one,runs):print(r,n,flush=True)
