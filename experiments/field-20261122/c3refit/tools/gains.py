import numpy as np, sim, plan, evalm
def scat(xb, n=60):
    Q=[]
    for i in range(n):
        r=np.random.default_rng(1000+i); st=evalm.start(r.normal(0,4.),r.normal(0,3.))
        Q.append(evalm.met(sim.run(xb,seed=i,noise=0.06,xerr=r.normal(0,0.08),g_scale=1+r.normal(0,0.08),u0_shift=r.normal(0,12.),h_bias=r.normal(0,2.),y_bias=r.normal(0,2.),cam_bias=[[a,b*r.normal(0,0.7)] for a,b in evalm.CB],**st)))
    f=lambda k: np.array([q[k] for q in Q])
    return 'ymin med %+5.1f worst %+5.1f | ymax med %+5.1f worst %+5.1f | <-14: %2d >+3: %2d | y435 p5 %+5.1f med %+5.1f p95 %+5.1f | psi435 p5 %+4.1f p95 %+4.1f | cmd max med %+4.0f worst %+4.0f | du med %3.0f p90 %3.0f | corner e med %.2f | h57 p5 %.0f p95 %.0f'%(
        np.median(f('ymin')),f('ymin').min(),np.median(f('ymax')),f('ymax').max(),(f('ymin')<-14).sum(),(f('ymax')>3).sum(),np.percentile(f('y435'),5),np.median(f('y435')),np.percentile(f('y435'),95),np.percentile(f('psi435'),5),np.percentile(f('psi435'),95),np.median(f('umax')),f('umax').max(),np.median(f('du')),np.percentile(f('du'),90),np.median(f('ecorner')),np.percentile(f('h57'),5),np.percentile(f('h57'),95))
pk=dict(y_t=-7.0,psi_end=1.0,y_end=-9.0,wy=0.15); pl=plan.plan(**pk)
for ke,kh,ky,rate in ((150,5,0,900),(120,4,0,700),(250,0,0,900),(80,8,5,900),(80,8,8,900),(100,6,6,900),(120,6,0,900)):
    xb=plan.xband(pl,k_e=ke,e_lim=110.,k_h=kh,h_lim=100.,k_y=ky,y_lim=80.,rate=rate)
    print('k_e %3d k_h %2d k_y %d rate %3d | %s'%(ke,kh,ky,rate,scat(xb)))
