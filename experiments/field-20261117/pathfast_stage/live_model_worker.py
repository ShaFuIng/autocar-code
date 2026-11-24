"""GPU-only stdio model adapter. No network, UART, or actuator device access.

Future adapters implement predict(image)->binary white-line mask at input size.
Output contract: protocol=1, seq, path in 1280x720 pixels, quality, valid,
model_id and weights_sha256. Capture timestamps stay with the host request.
"""
import base64,contextlib,hashlib,json,sys,time,gc
from pathlib import Path
import threading,queue
import cv2
import numpy as np
from pid_paths import selected, ROAD_ALGORITHMS
try:   # upper-edges-r1 (upper-edges-20261115): deck-edge points for upper_single, computed only when live_pid's request carries 'ue'
    import upper_edges as _UE
    _UE_H, _ = _UE.load_h(); _UE_CFG = {'lines': False}   # points only by default (cost); web/upper_edges_worker.json overrides (read at model load)
    try: _UE_CFG.update(json.loads(Path(__file__).with_name('upper_edges_worker.json').read_text(encoding='utf8')))
    except Exception: pass
except Exception: _UE = None

class V4:
    def __init__(self,weights):
        from ultralytics import YOLO
        self.model=YOLO(str(weights))
    def predict(self,image):
        result=self.model.predict(image,imgsz=640,device=0,verbose=False)[0]
        mask=result.semantic_mask.data.cpu().numpy()
        mask=np.squeeze(mask)==1
        return cv2.resize(mask.astype('uint8'),(image.shape[1],image.shape[0]),interpolation=cv2.INTER_NEAREST)>0

# pathfast2-r1 (png-async): the control packet's mask_png (PNG of the full-size mask) is encoded in a helper thread that starts
# when the model returns, while the path stage runs (cv2.imencode releases the GIL); publish_control waits for it. Same
# expression, same mask object (the path stage does not modify the mask), same bytes. pathfast.json "png_async" true = on
# (read at worker start); absent = the original in-line encoding. Only for camera-pair-v2-junction.
def _mask_png(mask):return base64.b64encode(cv2.imencode('.png',mask.astype('uint8')*255)[1]).decode()
try:_PNG_ASYNC=bool(json.loads(Path(__file__).with_name('pathfast.json').read_text(encoding='utf8')).get('png_async',False))
except Exception:_PNG_ASYNC=False
_PNG_POOL=[]
def _png_submit(mask):
    if not _PNG_POOL:
        from concurrent.futures import ThreadPoolExecutor
        _PNG_POOL.append(ThreadPoolExecutor(1,thread_name_prefix='mask-png'))
    return _PNG_POOL[0].submit(_mask_png,mask)

ADAPTERS={'v4':V4}  # Registry: reviewed adapters only; no arbitrary browser imports.

def corridor(mask,row_ratio):
    height,width=mask.shape;path=[]
    for fraction in np.linspace(max(.1,row_ratio-.08),min(.95,row_ratio+.08),9):
        y=int(fraction*(height-1));line=mask[max(0,y-2):y+3].mean(axis=0)>.3
        starts=np.flatnonzero(np.diff(np.r_[False,line,False].astype('int8'))==1)
        ends=np.flatnonzero(np.diff(np.r_[False,line,False].astype('int8'))==-1)
        centers=[(a+b-1)/2 for a,b in zip(starts,ends) if 2<=b-a<=width*.16]
        pairs=[(a,b) for a,b in zip(centers,centers[1:]) if width*.12<=b-a<=width*.95 and a<width/2<b]
        if len(pairs)!=1:continue
        a,b=pairs[0];path.append([(a+b)/2*1280/width,y*720/height])
    quality=len(path)/9
    return path,quality

_output_lock=threading.Lock()
def emit(value):
    with _output_lock:print(json.dumps(value),file=sys.__stdout__,flush=True)

def render_preview(image,mask,path,diagnostic,algorithm,p):
    prediction=diagnostic.get('road_state',{}).get('predicted',False)
    path_color=(0,165,255) if prediction else (0,255,0)
    preview=image.copy();preview[mask]=(preview[mask]*.5+np.array([220,40,220])*.5).astype('uint8')
    for x,y in path:cv2.circle(preview,(int(x*image.shape[1]/1280),int(y*image.shape[0]/720)),4,path_color,-1)
    if algorithm in ROAD_ALGORITHMS:
        from road_live import draw_segments
        draw_segments(preview,diagnostic)
    else:
        cv2.line(preview,(0,int(p['row_ratio']*image.shape[0])),(image.shape[1]-1,int(p['row_ratio']*image.shape[0])),(0,220,255),2)
    if algorithm in ('junction-k-lane-v3', 'junction-k-integrated-r2', 'standalone-line-v1', 'standalone-line-v2', 'standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction'):
        for i,(a,b) in enumerate(zip(path,path[1:])):
            if i%4>=2:continue
            scale=np.array([image.shape[1]/1280,image.shape[0]/720])
            cv2.line(preview,tuple((np.array(a)*scale).astype(int)),tuple((np.array(b)*scale).astype(int)),((0,165,255) if prediction else (255,120,20)),3)
    if algorithm in ('camera-pair-v2', 'camera-pair-v2-junction') and any(not 0<=x<1280 for x,y in path):
        side='RIGHT' if sum(x for x,y in path)/max(1,len(path))>=1280 else 'LEFT'
        cv2.putText(preview,'INFERRED CENTER OUTSIDE '+side,(10,55),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,220,255),2)
    if prediction:cv2.putText(preview,'PREDICTED <=150ms',(10,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,165,255),2)
    preview=cv2.resize(preview,(640,360))
    encoded=cv2.imencode('.jpg',preview,[cv2.IMWRITE_JPEG_QUALITY,75])[1]
    return base64.b64encode(encoded).decode()

class PreviewRenderer:
    def __init__(self):
        self.queue=queue.Queue(maxsize=1)
        threading.Thread(target=self.loop,daemon=True).start()
    def submit(self,item):
        try:self.queue.get_nowait()
        except queue.Empty:pass
        try:self.queue.put_nowait(item)
        except queue.Full:pass
    def loop(self):
        while True:
            args,identity=self.queue.get()
            try:emit(dict(identity,kind='preview',preview=render_preview(*args)))
            except Exception as e:print('Preview failed: '+str(e),file=sys.stderr,flush=True)


def main():
    config=json.loads(Path(sys.argv[1]).read_text());model_id=sys.argv[2]
    spec=config[model_id];weights=Path(spec['weights']);digest=hashlib.sha256(weights.read_bytes()).hexdigest()
    cv2.setNumThreads(2)
    with contextlib.redirect_stdout(sys.stderr):
        model=ADAPTERS[spec['adapter']](weights)
        model.predict(np.zeros((720,1280,3),dtype='uint8'))
    gc.collect();gc.freeze()  # Long-lived warmed model remains until worker exit; normal GC stays enabled.
    standalone_only=len(sys.argv)>3 and sys.argv[3] in ('standalone-line-v1', 'standalone-line-v2', 'standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction')
    k_lane=None
    if not standalone_only:
        from k_lane_live_path import KLaneLivePath
        k_lane=KLaneLivePath(.6)  # Warm K process before accepting captured frames.
    print(json.dumps({'ready':True,'protocol':1,'model_id':model_id,'weights_sha256':digest}),flush=True)
    renderer=PreviewRenderer()
    tracker=None
    calibrated=None
    road=None
    integrated=None
    if not standalone_only:
        from k_integrated_fast import ControlPriorityIntegratedK as IntegratedK
        integrated=IntegratedK(.6)
    from standalone_line_path import StandaloneLine
    from straight_preview_path import StraightPreviewLine
    from road_state_path import RoadStateLine as LiveConnectorLine
    from preview_error import PreviewErrorLine
    from camera_pair import CameraPairLine
    from offscreen_center import OffscreenLine
    from junction_live import JunctionLine
    standalone=StandaloneLine(.6)
    previous_algorithm=None
    route_epoch=None
    for line in sys.stdin:
        p=json.loads(line);start=time.monotonic()
        image=cv2.imdecode(np.frombuffer(base64.b64decode(p['jpeg']),dtype='uint8'),cv2.IMREAD_COLOR)
        if image is None:raise ValueError('Invalid image')
        decoded_at=time.monotonic()
        algorithm=p.get('path_algorithm','legacy9');selected(algorithm)
        if standalone_only and algorithm not in ('standalone-line-v1', 'standalone-line-v2', 'standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction'):raise ValueError('Restart worker to change standalone algorithm')
        if algorithm in ('standalone-line-v1', 'standalone-line-v2', 'standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction'):
            if previous_algorithm!=algorithm or standalone.row!=p['row_ratio']:
                standalone = (JunctionLine(p['row_ratio']) if algorithm=='camera-pair-v2-junction' else OffscreenLine(p['row_ratio']) if algorithm in ('camera-pair-v2', 'camera-pair-v2-junction') else CameraPairLine(p['row_ratio']) if algorithm=='camera-pair-v1' else PreviewErrorLine(p['row_ratio']) if algorithm=='preview-error-v1' else LiveConnectorLine(p['row_ratio']) if algorithm in ('straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction') else StraightPreviewLine(p['row_ratio']) if algorithm in ('standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction')
                              else StandaloneLine(p['row_ratio'], continue_steep=algorithm=='standalone-line-v2'))
            if algorithm in ('standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction') and route_epoch!=p.get('route_epoch','preview'):
                standalone.estimate.reset_route()
                route_epoch=p.get('route_epoch','preview')
            if algorithm in ('straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction'):standalone.center_px=320.+float(p.get('visual_center_offset_px_640',0.))
            original_image=image
            image=standalone.preprocess(image,p['captured'])
        elif algorithm=='junction-k-integrated-r2':
            if previous_algorithm!=algorithm or integrated.row!=p['row_ratio']:integrated.set_row(p['row_ratio'])
            original_image=image
            image=integrated.preprocess(image,p['captured'])
        with contextlib.redirect_stdout(sys.stderr):mask=model.predict(image)
        model_at=time.monotonic()
        algorithm=p.get('path_algorithm','legacy9');selected(algorithm)
        png_job=_png_submit(mask) if _PNG_ASYNC and algorithm=='camera-pair-v2-junction' else None   # pathfast2-r1
        diagnostic={}
        control_sent=False
        def publish_control(r):
            nonlocal control_sent
            if control_sent:raise RuntimeError('Duplicate control result')
            now=time.monotonic()
            identity=dict(protocol=1,seq=p['seq'],model_id=model_id,weights_sha256=digest,path_algorithm=algorithm)
            emit(dict(identity,kind='control',mask_png=png_job.result() if png_job is not None else base64.b64encode(cv2.imencode('.png',mask.astype('uint8')*255)[1]).decode(),path_diagnostic=r['path_diagnostic'],path=r['path'],quality=r['quality'],valid=r['valid'],stage_ms=dict(decode=(decoded_at-start)*1000,model=(model_at-decoded_at)*1000,path=(now-model_at)*1000,render=0),inference_ms=(now-start)*1000))
            control_sent=True
        if algorithm in ('standalone-line-v1', 'standalone-line-v2', 'standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction'):
            if hasattr(standalone,'estimate'):standalone.estimate.cs_zone=p.get('zone')   # crossing-source-r2: zone flag from live_pid (absent = no zone)
            result=standalone.step(image,mask,p['captured'])
            if p.get('ue') and _UE is not None:   # upper-edges-r1 (absent flag = unchanged)
                try:result['path_diagnostic']['upper_edges']=_UE.detect(mask,_UE_H,_UE_CFG)
                except Exception as e_:result['path_diagnostic']['upper_edges']=dict(rev='upper-edges-r1',error=type(e_).__name__)
            if algorithm=='standalone-line-v2':
                result['path_diagnostic'].update(path_revision='standalone-line-v2',estimator_revision='standalone-steep-continuity-r1',controller_revision='preview-turn-r2')
            if algorithm in ('standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction'):result['path_diagnostic']['route_epoch']=route_epoch
            path=result['path'];quality=result['quality'];valid=result['valid'];diagnostic=result['path_diagnostic']
            image=cv2.resize(original_image,(640,360))
        elif algorithm=='junction-k-integrated-r2':
            result=integrated.step(image,mask,p['captured'],on_a=publish_control)
            # Early control already left stdout. Record completion separately;
            # never issue a second control packet or refresh its capture time.
            if control_sent:
                print(json.dumps(dict(kind='path_completion',seq=p['seq'],capture_s=p['captured'],
                                      parallel=result['path_diagnostic'].get('parallel'))),file=sys.stderr,flush=True)
            path=result['path'];quality=result['quality'];valid=result['valid'];diagnostic=result['path_diagnostic']
            image=cv2.resize(original_image,(640,360))
        elif algorithm=='junction-k-lane-v3':
            if k_lane.row!=p['row_ratio']:k_lane.set_row(p['row_ratio'])
            result=k_lane.step(image,mask,p['captured'],on_a=publish_control)
            path=result['path'];quality=result['quality'];valid=result['valid'];diagnostic=result['path_diagnostic']
        elif algorithm in ROAD_ALGORITHMS:
            if road is None or road.algorithm!=algorithm or road.row!=p['row_ratio']:
                sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'laptop_workbench'))
                from road_segments_v3 import RoadPath
                road=RoadPath(algorithm,row=p['row_ratio'])
            result=road.step(image,mask,p['captured'])
            path=result['path'];quality=result['quality'];valid=result['valid']
            diagnostic={k:result[k] for k in ('mode','road_segments','history_segments','coverage','motion','rejections','path_semantics')}
        elif algorithm=='calibrated-v7':
            if calibrated is None:
                from calibrated_path import CalibratedPath
                calibrated=CalibratedPath()
            result=calibrated.step(image,mask,p['captured'])
            path=result['path'];quality=result['quality'];valid=result['valid']
            diagnostic={k:result[k] for k in ('calibration_sha256','mode','projected_width_cm','reason',
                                             'candidates','transition','switch_version')}
        elif algorithm=='temporal-v2':
            if tracker is None:
                sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'laptop_workbench'))
                from temporal_lane import TemporalLane
                tracker=TemporalLane()
            result=tracker.step(image,mask,p['captured'])
            path=[[x*2,y*2] for x,y in result['image_path']]
            quality=min(1,len(path)/30)
            valid=bool(result['valid'])
            diagnostic={k:result[k] for k in ('motion','double_rows','single_rows','reason')}
        else:
            path,quality=corridor(mask,p['row_ratio']);valid=quality>=.66
        previous_algorithm=algorithm
        path_at=time.monotonic()
        if algorithm in ('junction-k-lane-v3', 'junction-k-integrated-r2', 'standalone-line-v1', 'standalone-line-v2', 'standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction'):
            identity=dict(protocol=1,seq=p['seq'],model_id=model_id,weights_sha256=digest,path_algorithm=algorithm)
            if not control_sent:publish_control(result)
            renderer.submit(((image,mask,path,diagnostic,algorithm,dict(p)),identity))
            continue
        encoded_preview=render_preview(image,mask,path,diagnostic,algorithm,p)
        print(json.dumps(dict(protocol=1,seq=p['seq'],model_id=model_id,weights_sha256=digest,
                              path_algorithm=algorithm,path_diagnostic=diagnostic,
                              path=path,quality=quality,valid=valid,
                              stage_ms=dict(decode=(decoded_at-start)*1000,model=(model_at-decoded_at)*1000,path=(path_at-model_at)*1000,render=(time.monotonic()-path_at)*1000),
                              inference_ms=(time.monotonic()-start)*1000,
                              preview=encoded_preview)),flush=True)

    if k_lane is not None:k_lane.close()
    if integrated is not None:integrated.close()

if __name__=='__main__':main()
