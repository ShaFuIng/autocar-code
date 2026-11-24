"""Image-space outward target, shared with offline candidate. No hardware operations."""
import numpy as np
class Outward:
 def __init__(self,amount):self.amount=amount;self.offset=0.;self.t=None;self.bend=0.
 def step(self,path,envelope,row,t):
  p=np.asarray(path,dtype=float).reshape(-1,2)/2;dt=0 if self.t is None else max(0,min(.3,t-self.t));self.t=t
  env=np.asarray(envelope,dtype=float).reshape(-1,3);reason='SUPPORTED';score=0.;width=None
  if len(p)>=8 and np.ptp(p[:,1])>=65:
   # Curvature, not lateral displacement: a slanted straight remains straight.
   q=np.polyfit((p[:,1]-np.mean(p[:,1]))/np.ptp(p[:,1]),p[:,0],2);score=float(q[0]/4)
   if np.sqrt(np.mean((np.polyval(q,(p[:,1]-np.mean(p[:,1]))/np.ptp(p[:,1]))-p[:,0])**2))>12:score=0.;reason='FIT_UNCERTAIN'
  else:reason='SHORT_PATH'
  self.bend+=(score-self.bend)*min(1,dt/.5)
  if len(env)>1 and env[0,2]<=row<=env[-1,2]:width=float(np.interp(row,env[:,2],env[:,1]-env[:,0]))
  if width is None or width<20:reason='NO_WIDTH_AT_SAMPLE'
  strength=float(np.clip((abs(self.bend)-3)/7,0,1)) if reason=='SUPPORTED' else 0.
  target=-float(np.sign(self.bend))*self.amount*strength
  self.offset+=float(np.clip(target-self.offset,-12*dt,12*dt))
  shifted=p.copy();clipped=0
  if width is not None and width>=20:
   for i,(x,y) in enumerate(p):
    if len(env)>1 and env[0,2]<=y<=env[-1,2]:
     left=float(np.interp(y,env[:,2],env[:,0]));right=float(np.interp(y,env[:,2],env[:,1]));w=right-left
     if w<=0 or not left<=x<=right:continue
     delta=self.offset*np.clip(w/width,.25,2);limit=min(.1*w,max(0,x-left),max(0,right-x));limited=float(np.clip(delta,-limit,limit));clipped+=int(abs(limited-delta)>1e-6);shifted[i,0]+=limited
  # Never fabricate missing geometry. Zero candidate is byte/value identical.
  result=path if self.amount==0 else (shifted*2).tolist()
  return dict(path=result,bend_score=self.bend,direction='右彎' if self.bend>3 else '左彎' if self.bend< -3 else '直線／不確定',requested_px=self.amount,smoothed_px=self.offset,width_px=width,reason=reason,clipped_points=clipped)

def validate_amount(value):
 import math
 if type(value) not in (int,float) or not math.isfinite(value) or not 0<=value<=320:
  raise ValueError('彎道外移量須為0～320的有限數值（640寬）；實際外移另受道路寬度限制')
 return float(value)

def overlay(jpeg,path):
 import cv2
 image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),1)
 if image is None:raise ValueError('Invalid outward preview')
 scale=np.array([image.shape[1]/1280,image.shape[0]/720])
 points=np.asarray(path,dtype=float).reshape(-1,2)*scale
 for a,b in zip(points,points[1:]):cv2.line(image,tuple(a.astype(int)),tuple(b.astype(int)),(0,150,255),2)
 return cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,75])[1].tobytes()
