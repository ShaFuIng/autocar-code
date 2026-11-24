"""把指定輪次、frame_seq 範圍的 vision.mask_png（模型白線遮罩，小圖）拼成一張圖。用法：masks.py run fs0 fs1 out.png"""
import json,sys,base64,io
import numpy as np,cv2
EXP='D:/OneDrive/Desktop/自走車/Orin_Interface_v1/laptop_workbench/data/experiments/'
run,f0,f1,out=sys.argv[1],int(sys.argv[2]),int(sys.argv[3]),sys.argv[4]
ims=[]
for l in open(EXP+f'run-{run}-20261122/pid.jsonl',encoding='utf-8'):
    if '"mask_png"' not in l:continue
    d=json.loads(l);v=d.get('vision') or {}
    fs=v.get('frame_seq')
    if fs is None or not f0<=fs<=f1:continue
    m=v.get('mask_png')
    if not m:continue
    if ',' in m[:40]:m=m.split(',',1)[1]
    im=cv2.imdecode(np.frombuffer(base64.b64decode(m),np.uint8),cv2.IMREAD_COLOR)
    rs=(((v.get('path_diagnostic') or {}).get('road_state')) or {})
    lab=f"{fs} {'V' if v.get('valid') else 'x'} px{rs.get('observed_mask_pixels')} {(d['proposal'] or {}).get('reason','')[:12]}"
    ims.append((im,lab))
h,w=ims[0][0].shape[:2];S=3;cols=5;rows_=(len(ims)+cols-1)//cols
sheet=np.full((rows_*(h*S+16),cols*w*S,3),90,np.uint8)
for k,(im,lab) in enumerate(ims):
    x=(k%cols)*w*S;y=(k//cols)*(h*S+16)
    sheet[y+16:y+16+h*S,x:x+w*S]=cv2.resize(im,(w*S,h*S),interpolation=cv2.INTER_NEAREST)
    cv2.putText(sheet,lab,(x+2,y+12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,255),1)
cv2.imwrite(out,sheet);print(len(ims),w,h)
