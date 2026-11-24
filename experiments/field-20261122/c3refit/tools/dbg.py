import numpy as np, sim, plan, evalm
pl=plan.plan(y_t=-7.0,psi_end=1.0,y_end=-9.0,wy=0.15); xb=plan.xband(pl,k_e=80.,k_h=8.,k_y=8.)
bad=0
for i in range(60):
    r=np.random.default_rng(1000+i); st=evalm.start(r.normal(0,4.),r.normal(0,3.))
    kw=dict(seed=i,noise=0.06,xerr=r.normal(0,0.08),g_scale=1+r.normal(0,0.08),u0_shift=r.normal(0,12.),h_bias=r.normal(0,2.),y_bias=r.normal(0,2.),cam_bias=[[a,b*r.normal(0,0.7)] for a,b in evalm.CB])
    R=sim.run(xb,**st,**kw); m=(R[:,0]>=0.9)&(R[:,0]<=4.2); j=np.argmax(np.where(m,R[:,1],-1e9))
    if R[j,1]>260:
        bad+=1
        print('run %d: cmd max %+.0f at x %.2f share %.2f err %.2f y %+.1f psi %+.1f | start dh %.1f dy %.1f g %.2f u0 %+.0f xerr %+.2f | y min %.1f'%(i,R[j,1],R[j,0],R[j,7],R[j,4],R[j,2],R[j,3],st['h0']-np.interp(0.45,plan.BX,plan.BH),st['y0']-np.interp(0.45,plan.BX,plan.BY),kw['g_scale'],kw['u0_shift'],kw['xerr'],R[m,2].min()))
print(bad,'of 60 above +260')
