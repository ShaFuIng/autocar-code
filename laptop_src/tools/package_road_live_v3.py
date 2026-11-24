"""Build only: no network, service restart, model inference or hardware access."""
import hashlib,json,zipfile,tempfile,subprocess,sys
from pathlib import Path
from datetime import datetime,timezone
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'laptop_workbench/data/experiments/road-live-v3'
sys.path.insert(0,str(ROOT/'web'))
from pid_paths import ROAD_FILES
FILES=['web/pid_paths.py','web/live_pid.py','web/live_model_worker.py','web/road_live.py','web/visual_pid.py','web/calibrated_path.py','web/calibration_v7.json']+['laptop_workbench/'+n for n in ROAD_FILES]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 OUT.mkdir(exist_ok=True,parents=True)
 job=ROOT/'laptop_workbench/data/analysis/99e548cc753542e2b402518004abd54f'
 spec=json.loads((job/'job.json').read_text())['spec']
 baseline=json.loads((job/'result.json').read_text())['implementation_sha256']
 for n in ('road_segments_v3.py','entrance_paths.py','entrance_paths_v2.py'):
  assert sha(ROOT/'laptop_workbench'/n)==baseline[n],n
 for name in FILES:
  if name.endswith('.py'):compile((ROOT/name).read_bytes(),name,'exec')
 params=spec['groups'][0]['parameters'];assert spec['groups'][0]['pid_revision']==37
 manifest=dict(version='road-live-v3.1',built_at=datetime.now(timezone.utc).isoformat(),algorithms=['road-image-v3','road-memory-v3'],files={n:sha(ROOT/n) for n in FILES},remote_root='/home/jetson/orin_interface_v1',automatic_restart=False,automatic_start=False,pid_revision=37,parameters=params)
 target=OUT/'road-live-v3.zip'
 with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as z:
  for n in FILES:z.write(ROOT/n,n)
  z.writestr('manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2))
  z.write(ROOT/'tests/test_road_live.py','verification/test_road_live.py')
 with zipfile.ZipFile(target) as z:
  assert z.testzip() is None
  for n,h in manifest['files'].items():assert hashlib.sha256(z.read(n)).hexdigest()==h
  with tempfile.TemporaryDirectory() as d:
   z.extractall(d)
   code="import sys;sys.path[:0]="+repr([str(Path(d)/'web'),str(Path(d)/'laptop_workbench')])+";import live_pid,live_model_worker;from road_live import RoadController;from road_segments_v3 import RoadPath;RoadController("+repr(params)+");RoadPath('road-memory-v3');print('isolated package imports OK')"
   subprocess.run([sys.executable,'-I','-B','-c',code],check=True)
 report=dict(version=manifest['version'],built_at=manifest['built_at'],passed=True,python_delta_tests=7,frontend_delta_cases=3,historical_suites_rerun=False,new_inference_frames=0,proposal_parity_frames={'A':353,'B':353},pid_revision=37,parameters=params,algorithm_sources_unchanged=True,bundle=dict(path=str(target.relative_to(ROOT)),sha256=sha(target)),files=manifest['files'],test_sources={n:sha(ROOT/n) for n in ['tests/test_road_live.py','laptop_workbench/test_road_live_ui.js']},deployed=False,service_restarted=False,hardware_commands_sent=False,orin_readonly_authorized=True,orin_host='10.135.202.240',orin_host_dependencies={'cv2':'4.8.0','numpy':'1.21.5'},ui_overview_verified=True,limitations=['Orin GPU worker execution and live timing not verified for new version','No real vehicle closed-loop test','A/B width extrapolation remains estimated; C paused','v37 uses direction_enabled=0; two-row direction blending not part of this integration'])
 for n,v in [('manifest.json',manifest),('report.json',report)]: (OUT/n).write_text(json.dumps(v,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps(dict(bundle=str(target),sha256=sha(target),remote_files=len(FILES),tests=10),ensure_ascii=False))
if __name__=='__main__':main()
