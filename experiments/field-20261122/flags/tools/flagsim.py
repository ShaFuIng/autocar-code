"""以紀錄的 dash_route 俯仰重播 live_pid 2118~2150 行（down_until / down_steep / down_level_t0），
逐輪列出每次下坡窗內回平旗標的設定／清除，並與紀錄的 land4 狀態、downhill_release 對照。"""
import pickle,glob,os,sys,json
TMP=os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','tmp')
DOWN_STEEP=-8.;TAIL=4.
def sim(rows,steep=DOWN_STEEP,min_steep_s=None,min_steep_n=None,hyst=0.):
    """回傳每格 (down_tail, down_steep, dlt0)。min_steep_s/min_steep_n/hyst = 候選規則（None/0 = 現況）。"""
    du=None;ds=False;dl=None;prev_route=None;t_steep0=None;n_steep=0;out=[]
    for r in rows:
        now=r['con'];dpr=r['dpr']
        if r['fdg'] is not None and dpr is not None:dpr=-2.0+.1
        if prev_route=='DONE' and r['route']=='WAIT_CLIMB':du=None
        prev_route=r['route']
        if dpr is not None and dpr< -5.:du=now+TAIL
        if dpr is not None and dpr<steep:
            if not ds:t_steep0=now;n_steep=0
            ds=True;dl=None;n_steep+=1
        elif dpr is not None and ds and dl is None:
            ok=dpr>=steep+hyst
            if ok and min_steep_s is not None and t_steep0 is not None and now-t_steep0<min_steep_s:ok=False
            if ok and min_steep_n is not None and n_steep<min_steep_n:ok=False
            if ok:dl=now
        tail=du is not None and now<=du
        if not tail:ds=False;dl=None;t_steep0=None;n_steep=0
        out.append((tail,ds,dl))
    return out
def episodes(rows,st):
    eps=[];cur=None
    for i,(r,(tail,ds,dl)) in enumerate(zip(rows,st)):
        if tail and cur is None:cur=dict(i0=i,t0=r['con'],odo0=r['odo'],sets=[],l4land=None,l4slope=None,lap=r['lap'],fs0=r['fs'])
        if cur is not None and not tail:cur['i1']=i;eps.append(cur);cur=None;continue
        if cur is None:continue
        prev=st[i-1][2] if i>0 else None
        if dl is not None and prev is None:cur['sets'].append(dict(i=i,t=round(r['con']-cur['t0'],3),d=None if r['odo'] is None or cur['odo0'] is None else round(r['odo']-cur['odo0'],2),dpr=r['dpr'],prev_dpr=rows[i-1]['dpr'],fs=r['fs'],cleared=None))
        if dl is None and prev is not None and ds and cur['sets']:cur['sets'][-1]['cleared']=round(r['con']-rows[cur['sets'][-1]['i']]['con'],3);cur['sets'][-1]['clr_dpr']=r['dpr']
        l4=r.get('land4') or {}
        if l4.get('state')=='SLOPE' and cur['l4slope'] is None:cur['l4slope']=i
        if l4.get('state')=='LAND' and cur['l4land'] is None:cur['l4land']=i
    if cur is not None:cur['i1']=len(rows);eps.append(cur)
    return eps
def check_rel(rows,st):
    """downhill_release.level_s 與重建 dlt0 對照：回傳 (n, max abs diff)"""
    n=0;mx=0.
    for r,(tail,ds,dl) in zip(rows,st):
        if r['rel'] is not None:
            n+=1
            if dl is None:mx=max(mx,99.)
            else:mx=max(mx,abs((r['con']-dl)-r['rel']))
    return n,mx
if __name__=='__main__':
    runs=sys.argv[1:] or sorted(os.path.basename(x)[:4] for x in glob.glob(os.path.join(TMP,'*.pkl')))
    tot=dict(eps=0,bounce=0,eps_bounce=0);out=[]
    for run in runs:
        D=pickle.load(open(os.path.join(TMP,run+'.pkl'),'rb'));rows=[r for r in D['rows'] if r['con'] is not None]
        st=sim(rows);eps=episodes(rows,st);n,mx=check_rel(rows,st)
        for e in eps:
            bounces=[s for s in e['sets'] if s['cleared'] is not None]
            # land4 檢查：重建的第一次設定是否就是 land4 轉 LAND 的那格（防護前）
            l4=e['l4land'];first=e['sets'][0]['i'] if e['sets'] else None
            line=dict(run=run,lap=e['lap'],fs0=e['fs0'],dur=round(rows[e['i1']-1]['con']-e['t0'],2),n_sets=len(e['sets']),sets=e['sets'],
                      l4land_i_minus_firstset=None if (l4 is None or first is None) else l4-first,
                      l4land_t=None if l4 is None else round(rows[l4]['con']-e['t0'],3),
                      l4land_d=None if (l4 is None or rows[l4]['odo'] is None or e['odo0'] is None) else round(rows[l4]['odo']-e['odo0'],2))
            out.append(line);tot['eps']+=1;tot['bounce']+=len(bounces);tot['eps_bounce']+=bool(bounces)
        print(run,'episodes',len(eps),'rel_check n=%d maxdiff=%.4f'%(n,mx),flush=True)
    json.dump(out,open(os.path.join(TMP,'..','proof','A_episodes.json'),'w'),ensure_ascii=False,indent=0)
    print(tot)
