"""Build exact remote patches against the fetched source; do not overwrite laptop-only features."""
from pathlib import Path
import json,hashlib,ast,shutil,difflib
R=Path(__file__).resolve().parents[1];O=R/'laptop_workbench/data/experiments/k-integrated-r2';P=O/'payload';ks="('junction-k-lane-v3','junction-k-integrated-r2')"
for name in ['web/pid_paths.py','web/live_pid.py','web/live_model_worker.py']:
 s=(O/'remote-before'/name).read_text(encoding='utf8')
 if name.endswith('pid_paths.py'):
  s=s.replace('PATHS = {',"PATHS = {\n    'junction-k-integrated-r2': dict(label='K 完整整合 r2 · 暖光校色／可信段／歷史補缺口', control=True, preview=True, version=4),")
 elif name.endswith('live_pid.py'):
  s=s.replace("algorithm!='junction-k-lane-v3'","algorithm not in "+ks).replace('self.path_algorithm=="junction-k-lane-v3"','self.path_algorithm in '+ks).replace("self.path_algorithm=='junction-k-lane-v3'",'self.path_algorithm in '+ks)
  s=s.replace('return dict(curve_outward_version=1','return dict(integrated_k_version=2,curve_outward_version=1',1).replace("            amount=validate_amount(p.get('curve_outward_px_640',0.))","            if algorithm=='junction-k-integrated-r2' and not (ROOT/'k_integrated_live.py').is_file():raise ValueError('完整K r2依賴未部署')\n            amount=validate_amount(p.get('curve_outward_px_640',0.))")
 else:
  s=s.replace("if algorithm=='junction-k-lane-v3':",'if algorithm in '+ks+':')
  s=s.replace('    road=None','    road=None\n    from k_integrated_live import IntegratedK\n    integrated=IntegratedK(.6)\n    previous_algorithm=None')
  s=s.replace("        with contextlib.redirect_stdout(sys.stderr):mask=model.predict(image)","        algorithm=p.get('path_algorithm','legacy9');selected(algorithm)\n        if algorithm=='junction-k-integrated-r2':\n            if previous_algorithm!=algorithm or integrated.row!=p['row_ratio']:integrated.set_row(p['row_ratio'])\n            original_image=image\n            image=integrated.preprocess(image,p['captured'])\n        with contextlib.redirect_stdout(sys.stderr):mask=model.predict(image)")
  s=s.replace('        if algorithm in '+ks+':\n            if k_lane.row',"        if algorithm=='junction-k-integrated-r2':\n            result=integrated.step(image,mask,p['captured'],on_a=publish_control)\n            path=result['path'];quality=result['quality'];valid=result['valid'];diagnostic=result['path_diagnostic']\n            image=cv2.resize(original_image,(640,360))\n        elif algorithm=='junction-k-lane-v3':\n            if k_lane.row")
  s=s.replace('        path_at=time.monotonic()','        previous_algorithm=algorithm\n        path_at=time.monotonic()').replace('    if k_lane is not None:k_lane.close()','    if k_lane is not None:k_lane.close()\n    integrated.close()')
 ast.parse(s);p=P/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(s,encoding='utf8')
 (O/(Path(name).name+'.diff')).write_text(''.join(difflib.unified_diff((O/'remote-before'/name).read_text().splitlines(True),s.splitlines(True),fromfile='remote/'+name,tofile='r2/'+name)),encoding='utf8')
for name in ['k_integrated_live.py','k_integrated_parallel.py','warm_color_candidate.py','history_gap_candidate.py','junction_segment_candidate.py']:
 shutil.copy2(R/'web'/name,P/'web'/name)
manifest={str(p.relative_to(P)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in P.rglob('*.py')}
(O/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8');print(list(manifest))
