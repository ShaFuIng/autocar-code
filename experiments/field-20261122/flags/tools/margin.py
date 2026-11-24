"""每次下坡：第一格 dpr<-8 到回平旗標的時間／距離；第一格 <-8 後 1.5 s 內（回平前）dpr 的最高值（離 -8 的餘裕）。"""
import pickle,glob,os,json
from flagsim import sim,TMP
res=[]
for fn in sorted(glob.glob(os.path.join(TMP,'*.pkl'))):
    run=os.path.basename(fn)[:4];rows=[r for r in pickle.load(open(fn,'rb'))['rows'] if r['con'] is not None];st=sim(rows)
    i=0;n=len(rows)
    while i<n:
        if st[i][0] and (i==0 or not st[i-1][0]):
            j=i
            while j<n and st[j][0]:j+=1
            ep=list(range(i,j))
            k0=next((k for k in ep if st[k][1]),None)
            kl=[k for k in ep if st[k][2] is not None and (k==0 or st[k-1][2] is None)]
            if k0 is not None:
                land=kl[-1] if kl else None
                win=[k for k in ep if k>k0 and (land is None or k<land) and rows[k]['con']-rows[k0]['con']<=1.5 and rows[k]['dpr'] is not None]
                mx=max((rows[k]['dpr'] for k in win),default=None)
                res.append(dict(run=run,lap=rows[i]['lap'],t_steep_to_level=None if land is None else round(rows[land]['con']-rows[k0]['con'],3),
                    d_steep_to_level=None if (land is None or rows[land]['odo'] is None or rows[k0]['odo'] is None) else round(rows[land]['odo']-rows[k0]['odo'],2),
                    max_dpr_1p5s=mx,n_level_sets=len(kl),dpr_at_k0=rows[k0]['dpr'],dpr_next=[rows[k]['dpr'] for k in ep if k>k0][:4]))
            i=j
        else:i+=1
json.dump(res,open(os.path.join(TMP,'..','proof','A_margin.json'),'w'),indent=0)
t=sorted(r['t_steep_to_level'] for r in res if r['t_steep_to_level'] is not None)
print('n',len(res),'t_steep_to_level min..max',t[:6],t[-4:])
m=sorted((r['max_dpr_1p5s'],r['run'],r['lap']) for r in res if r['max_dpr_1p5s'] is not None)
print('closest to -8 (max dpr within 1.5 s after first steep, before landing):',m[-8:])
print('count max>-9',sum(1 for x in m if x[0]>-9),'> -10',sum(1 for x in m if x[0]>-10))
