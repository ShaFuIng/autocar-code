"""Regression scenarios: one-frame loss, expiry, false alignment and seam conflict."""
import sys
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parents[1]/'web'),str(Path(__file__).resolve().parents[1]/'web/k_lane_runtime')]
import numpy as np
import history_gap_candidate as m
im=np.zeros((360,640,3),np.uint8);mask=np.zeros((360,640),bool);mask[:,218:223]=True;mask[:,418:423]=True
path=[[640,float(y*2)] for y in range(200,331,2)]
r=dict(path=path,valid=True,mode='KL_A')
empty=dict(path=[],valid=False,mode='KL_A')
original=m.estimate_motion
m.estimate_motion=lambda a,b:(np.eye(3),dict(valid=True,reason='TEST_IDENTITY'))
f=m.HistoryGap();f.step(im,mask,0,r)
a=f.step(im,mask,.1,empty);assert a['valid'] and a['gap_repair']['used']
a=f.step(im,mask,.2,empty);assert a['gap_repair']['observation_age_s']==.2
assert not f.step(im,mask,.31,empty)['valid'] # repaired output cannot renew memory
f=m.HistoryGap();f.step(im,mask,0,r)
assert not f.step(im,np.zeros_like(mask),.1,empty)['valid']
assert not f.step(im,mask,.1,dict(empty,mode='KL_CROSS'))['valid']
f=m.HistoryGap();f.step(im,mask,0,r)
cut=dict(r,path=path[8:]);a=f.step(im,mask,.1,cut);assert a['gap_repair']['used'] and a['path']==path
f=m.HistoryGap();f.step(im,mask,0,r)
a=f.step(im,mask,.1,dict(cut,geometry_guard={'cutoff_y':216}));assert a['path']==cut['path']
f=m.HistoryGap();f.step(im,mask,0,r)
a=f.step(im,mask,.1,dict(cut,path=[[x+60,y] for x,y in cut['path']]));assert not a['gap_repair']['used']
m.estimate_motion=lambda a,b:(None,dict(valid=False,reason='TEST_BAD_FLOW'))
f=m.HistoryGap();f.step(im,mask,0,r);assert not f.step(im,mask,.1,empty)['valid']
m.estimate_motion=original
print('PASS: whole gap, no age renewal, expiry, missing mask, mode switch, partial gap, deliberate geometry cut, conflict, alignment failure')
