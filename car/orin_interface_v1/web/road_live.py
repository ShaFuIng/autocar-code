"""Road v3 live adapter, sharing the offline sampling and PID implementation."""
import sys
from pathlib import Path
import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'laptop_workbench'))
from entrance_paths import AdaptivePID


class RoadController:
    def __init__(self,parameters):
        if parameters.get('direction_enabled'):
            raise ValueError('A/B v3目前沿用v37單列動態取樣；請關閉近遠列方向混合')
        self.adaptive=AdaptivePID(parameters)

    def step_path(self,t,path):
        return self.adaptive.step(t,path)


def draw_segments(image,diagnostic):
    scale=np.array([image.shape[1]/1280,image.shape[0]/720])
    for segment in diagnostic.get('road_segments',[]):
        source=segment['source'];points=np.asarray(segment['points'])*scale
        color=(0,255,0) if source=='double' else (0,180,255) if 'history' in source else (220,80,220) if source=='single_extrapolated' else (0,210,0)
        for i,(a,b) in enumerate(zip(points,points[1:])):
            if source!='double' and i%3==2:continue
            cv2.line(image,tuple(a.astype(int)),tuple(b.astype(int)),color,2)


def mark_sample(jpeg,row):
    image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),1)
    if image is None:raise ValueError('Invalid road preview')
    y=round(row*(image.shape[0]-1))
    cv2.line(image,(0,y),(image.shape[1]-1,y),(0,220,255),2)
    return cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,75])[1].tobytes()
