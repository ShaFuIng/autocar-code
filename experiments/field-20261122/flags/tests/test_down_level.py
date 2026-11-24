"""down-level-guard-r1 測試。用法：python test_down_level.py [--all]
(1) 合成序列：坡口 1 格彈跳被擋、正常落地不變、擋下後俯仰停在 -8 以上時於 min_steep_s 後補設、檔案缺／鍵缺＝原規則。
(2) 紀錄重播（需 ../tmp/*.pkl，tools/extract.py 產生）：原程式 vs 候選（無檔、鍵缺、min_steep_s 0.5）逐格比對 (down_tail, down_steep, down_level_t0)。"""
import os,sys,glob,json
HERE=os.path.dirname(os.path.abspath(__file__));sys.path.insert(0,HERE)
from harness import Replayer,rows_of
BASE=open(os.path.join(HERE,'..','..','..','orin-web-snapshot-20261122c','live_pid.py'),encoding='utf-8').read()
CAND=open(os.path.join(HERE,'..','candidate','live_pid.py'),encoding='utf-8').read()
TMP=os.path.join(HERE,'..','tmp')
fails=[]
def ok(c,msg):
    print(('通過 ' if c else '失敗 ')+msg)
    if not c:fails.append(msg)
def seq(pitches,dt=0.033,route='DONE'):
    return [dict(con=100+k*dt,dpr=p,route=route,fdg=None) for k,p in enumerate(pitches)]
def run(src,rows,cfg=None):
    R=Replayer(src,cfg);out=[R.step(r) for r in rows];R.close();return out
# ---- (1) 合成
bounce=[0.4,-2.0,-8.18,-7.95,-12.75]+[-15.5]*60+[-6.0,-1.0,0.5]+[0.3]*30
o_base=run(BASE,seq(bounce));o_c=run(CAND,seq(bounce),{'min_steep_s':0.5})
ok(o_base[3][2] is not None,'原程式：坡口 -8.18→-7.95 這格立起回平旗標（重現 54cb）')
ok(o_c[3][2] is None and o_c[3][3] is not None,'候選 0.5 s：該格不立旗標、並記 down_level_guard')
land_b=next(k for k,o in enumerate(o_base) if k>5 and o[2] is not None);land_c=next(k for k,o in enumerate(o_c) if k>5 and o[2] is not None)
ok(land_b==land_c and o_base[land_b][2]==o_c[land_c][2],'真正落地那格兩者相同（第 %d 格）'%land_b)
ok(all(a[:3]==b[:3] for k,(a,b) in enumerate(zip(o_base,o_c)) if k!=3),'除彈跳格外逐格相同')
early=[0.0,-9.0]+[-7.0]*40
o_c2=run(CAND,seq(early),{'min_steep_s':0.5});k2=next(k for k,o in enumerate(o_c2) if o[2] is not None)
ok(abs((seq(early)[k2]['con']-seq(early)[1]['con'])-0.5)<0.034,'擋下後俯仰一直在 -8 以上：於陡坡開始後約 0.5 s 補設（第 %d 格）'%k2)
for cfg,name in ((None,'無檔'),({'note':'x'},'鍵缺'),({'min_steep_s':None},'值 null')):
    ok(all(a[:3]==b[:3] for a,b in zip(run(BASE,seq(bounce)),run(CAND,seq(bounce),cfg))),'候選 %s＝原程式（合成彈跳序列逐格相同）'%name)
# ---- (2) 紀錄
runs=sorted(os.path.basename(x)[:4] for x in glob.glob(os.path.join(TMP,'*.pkl')))
if '--all' not in sys.argv:runs=[r for r in runs if r in ('2728','9af6','7658','54cb','e69d','6f61','c6f3','e0cd','ccf1','d60e','18fc','3197','beb1')]
diff_total=[];nfr=0
for rn in runs:
    rows=rows_of(rn,TMP);nfr+=len(rows)
    b=run(BASE,rows);c0=run(CAND,rows,None);c1=run(CAND,rows,{'note':'no key'});c5=run(CAND,rows,{'min_steep_s':0.5})
    ok(all(x[:3]==y[:3]==z[:3] for x,y,z in zip(b,c0,c1)),'%s：候選無檔／鍵缺與原程式逐格相同（%d 格）'%(rn,len(rows)))
    # 原程式重建的 down_level_t0 與紀錄的 downhill_release.level_s 吻合
    mx=max((abs((r['con']-x[2])-r['rel']) if x[2] is not None else 99. for r,x in zip(rows,b) if r['rel'] is not None),default=0.)
    ok(mx<0.002,'%s：重播的回平時刻與紀錄 downhill_release.level_s 相符（最大差 %.4f s）'%(rn,mx))
    d=[(rows[k]['fs'],rows[k]['lap'],x[2],y[2]) for k,(x,y) in enumerate(zip(b,c5)) if x[:3]!=y[:3]]
    diff_total+= [(rn,)+t for t in d]
print('min_steep_s 0.5 與原程式不同的格：',diff_total)
ok(all(t[0]=='54cb' and t[1]==55617 for t in diff_total) and any(t[0]=='54cb' for t in diff_total) if '54cb' in runs else not diff_total,
   'min_steep_s 0.5：只有 54cb 第 3 圈 frame_seq 55617 一格不同（%d 輪 %d 格）'%(len(runs),nfr))
print('\n結果：%s'%('全部通過' if not fails else '失敗 %d 項'%len(fails)));sys.exit(1 if fails else 0)
