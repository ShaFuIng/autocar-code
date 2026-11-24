"""Read-only camera snapshots, local segmentation. No vehicle control."""
import json,subprocess,threading,time,uuid,os
from pathlib import Path
from analysis_jobs import PY,write
ROOT=Path(__file__).resolve().parent
class CameraMeasure:
    def __init__(self,app):
        self.app=app;self.lock=threading.Lock();self.state={'state':'idle'}
        self.root=app.store.root/'experiments/camera-measure';self.root.mkdir(parents=True,exist_ok=True)
    def status(self):
        with self.lock:return dict(self.state)
    def start(self):
        with self.lock:
            if self.state['state']=='running':return dict(self.state)
            uid=uuid.uuid4().hex
            self.state=dict(state='running',id=uid)
        threading.Thread(target=self.run,args=(uid,),daemon=True).start()
        return self.status()
    def run(self,uid):
        folder=self.root/uid;folder.mkdir()
        try:
            before=time.time()
            body=self.app.orin.call('camera')
            (folder/'raw.jpg').write_bytes(body)
            metadata=dict(id=uid,requested_at=before,received_at=time.time(),
                timestamp_kind='laptop_receipt_not_sensor_exposure',hardware_output=False)
            write(folder/'capture.json',metadata)
            with (folder/'worker.log').open('wb') as log:
                result=subprocess.run([str(PY),str(ROOT/'camera_measure_worker.py'),str(folder)],
                    stdout=log,stderr=log,timeout=120,
                    creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
            if result.returncode:raise ValueError('本機V4分析失敗；原始快照已保留')
            data=json.loads((folder/'result.json').read_text(encoding='utf8'))
            with self.lock:self.state=dict(data,state='completed')
        except Exception as e:
            with self.lock:self.state=dict(state='failed',id=uid,error=str(e))
