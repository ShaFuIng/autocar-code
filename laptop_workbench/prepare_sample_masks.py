"""Recover separate display masks from preserved weights; no path/control changes."""
import overnight_runner as r
import json
r.torch.set_num_threads(4)
r.cv2.setNumThreads(2)
report=json.loads((r.OUT/'report.json').read_text(encoding='utf-8'))
configs={'v4-640':('yolo26_sem_v4_trial/runs/trial01/weights/best.pt',640),'v4-960':('yolo26_sem_v4_trial/runs/trial01/weights/best.pt',960),'v3-640':('yolo26_sem_v3_trial/runs/trial01/weights/best.pt',640),'v5-focus02':('yolo26_sem_v5_focus70/runs/focus02/weights/best.pt',640),'v5-focus03':('yolo26_sem_v5_focus70/runs/focus03/weights/best.pt',960)}
for name in [x['model'] for x in report['results']]:
    if name in configs:
        path,size=configs[name];pred=r.YoloPredictor(r.ROOT/path,size)
    elif name=='opencv-local':pred=r.opencv
    else:
        model=(r.TinyUNet() if name=='tiny-unet' else r.FastInspired()).cuda()
        model.load_state_dict(r.torch.load(r.OUT/(name+'.pt'),weights_only=True)['model']);pred=r.SmallPredictor(model)
    for v in report['videos']:
        if v['model']!=name:continue
        rows=json.loads((r.OUT/v['rows']).read_text(encoding='utf-8'))
        source=r.ROOT/('完整跑道錄影_20261016_155847_3e961ecb' if v['source']=='fulltrack' else 'Orin_Interface_v1/field_test/review_latest/08dfcb8c29f74ae78dba9f294060e8e9')
        caps={}
        for s in v['samples']:
            p=r.OUT/'masks'/name/v['source']/(str(s['frame']).zfill(6)+'.png')
            row=rows[s['frame']]
            if row['source'] not in caps:caps[row['source']]=r.cv2.VideoCapture(str(source/row['source']))
            cap=caps[row['source']];cap.set(r.cv2.CAP_PROP_POS_FRAMES,row['source_frame']);ok,im=cap.read();assert ok
            mask=pred(im)
            p.parent.mkdir(parents=True,exist_ok=True)
            r.cv2.imencode('.png',mask.astype('uint8')*255)[1].tofile(str(p))
        for cap in caps.values():cap.release()
    print(name,flush=True)
    del pred;r.torch.cuda.empty_cache()
r.save('display_masks.json',dict(note='Display masks recomputed from original AVI frames and preserved weights/settings. Original path rows and metrics unchanged.',models=8))
