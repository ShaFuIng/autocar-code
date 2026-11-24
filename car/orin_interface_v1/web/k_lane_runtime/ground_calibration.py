"""Candidate planar projection only. Coordinates cm: x right of lane centre,
z forward from front tyre foremost edge. Not a vehicle/camera-centred frame.
"""
import cv2
import numpy as np

def project(points,H):
    p=np.column_stack([np.asarray(points,float),np.ones(len(points))])@np.asarray(H).T
    if np.any(abs(p[:,2])<1e-9):raise ValueError('Projection at horizon')
    out=p[:,:2]/p[:,2,None]
    if not np.isfinite(out).all():raise ValueError('Nonfinite projection')
    return out

def fit(image,world):
    image=np.asarray(image,float);world=np.asarray(world,float)
    if len(image)<4 or image.shape!=world.shape or image.shape[1]!=2:raise ValueError('Need matching planar points')
    if not np.isfinite(image).all() or not np.isfinite(world).all():raise ValueError('Nonfinite input')
    if np.linalg.matrix_rank(image-image.mean(0))<2 or np.linalg.matrix_rank(world-world.mean(0))<2:raise ValueError('Collinear calibration')
    H,_=cv2.findHomography(image,world,0)
    if H is None or abs(np.linalg.det(H))<1e-12:raise ValueError('Degenerate calibration')
    residual=project(image,H)-world
    return H,dict(rmse_cm=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))),
        max_cm=float(np.linalg.norm(residual,axis=1).max()),residual_cm=residual.tolist())

def calculate(lines,width_cm=46,require_confirmed=True):
    if len(lines)!=3 or (require_confirmed and not all(l['confirmed'] for l in lines)):raise ValueError('Three confirmed distance lines required')
    if not 10<width_cm<200:raise ValueError('Invalid width')
    if len({l['cm'] for l in lines})!=3:raise ValueError('Distinct distances required')
    image=[];world=[]
    for l in lines:
        if l['left'][0]>=l['right'][0]:raise ValueError('Reversed endpoints')
        image.extend([l['left'],l['right']]);world.extend([[-width_cm/2,l['cm']],[width_cm/2,l['cm']]])
    image=np.asarray(image,float);world=np.asarray(world,float)
    H,metrics=fit(image,world)
    withheld=[]
    # Hold out an entire distance row, not one endpoint of the same row.
    for i,l in enumerate(lines):
        indices=[j for j in range(6) if j//2!=i]
        h,_=fit(image[indices],world[indices])
        predicted=project(image[2*i:2*i+2],h)
        withheld.append(dict(distance_cm=l['cm'],predicted_cm=predicted.tolist(),
            max_error_cm=float(np.linalg.norm(predicted-world[2*i:2*i+2],axis=1).max())))
    return dict(matrix=H.tolist(),inverse=np.linalg.inv(H).tolist(),fit=metrics,
        held_out_rows=withheld,image_points=image.tolist(),world_points=world.tolist())
