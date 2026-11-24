"""Build isolated, hash-checked candidates. No SSH, deployment or device access."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import shutil
from ops07 import inspect_package

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'data/experiments/ops07-adapter-20261109'
PACKAGES=('slow-steer-7fbf','best-today-46db','latest-2lap-97ed')


def replace(s,old,new):
    if s.count(old)!=1:raise ValueError('Patch drift: '+old[:90]+' count='+str(s.count(old)))
    return s.replace(old,new)


def live_patch(s):
    # New branches only. Existing package algorithms/configs remain intact.
    s='import ops07_runtime as ops07_support\n'+s
    s=replace(s,"if mode not in ('browser_lease','orin_timed'):","if mode not in ('browser_lease','orin_timed','orin_continuous'):")
    needle="        owner=p.get('owner');seconds=p.get('seconds')"
    s=replace(s,needle,"        if mode=='orin_continuous' and (not getattr(self,'ops07',None) or p.get('_ops07_ticket') is not self.ops07.ticket):raise ValueError('Use 07 arm-hold API')\n"+needle)
    needle="            if params.get('curve_outward_px_640',0.)!=getattr(self,'curve_outward_px_640',0.):"
    s=replace(s,needle,"            if mode=='orin_continuous' and self.latest.get('weights_sha256')!=self.ops07.package['recognition']['weights_sha256']:raise ValueError('07 模型權重與方案不符')\n"+needle)
    s=replace(s,"if type(seconds) not in (int,float) or not math.isfinite(seconds) or seconds<1:raise ValueError('Duration must be finite and at least 1 second')",
                "if mode!='orin_continuous' and (type(seconds) not in (int,float) or not math.isfinite(seconds) or seconds<1):raise ValueError('Duration must be finite and at least 1 second')")
    s=replace(s,"decel_seconds=0,seconds=seconds)","decel_seconds=0,seconds=seconds,execution_mode=mode)")
    s=replace(s,"deadline=time.monotonic()+seconds,","deadline=None if mode=='orin_continuous' else time.monotonic()+seconds,")
    s=replace(s,"max(0,self.run['deadline']-time.monotonic())","(None if ops07_support.continuous(self.run) else max(0,self.run['deadline']-time.monotonic()))")
    s=replace(s,"elif now>=self.run['deadline']:","elif ops07_support.deadline_expired(self.run,now):")
    needle="                    proposal=None;sample_row=self.row_ratio"
    s=replace(s,needle,"                    if ops07_support.holding_frame(self,result,captured,frame,seq,preview):continue\n"+needle)
    needle="            self.check_safety()"
    s=replace(s,needle,needle+"\n            if getattr(self,'ops07',None):\n                try:self.ops07.tick()\n                except Exception:self.end('OPS07_RUNTIME_ERROR')")
    needle="                self.run.update(state='stopped',reason=reason,ended=time.time())"
    s=replace(s,needle,needle+"\n                if ops07_support.continuous(self.run) and getattr(self,'ops07',None):self.ops07.ended(reason)")
    needle="    def _pid_send(self,p):"
    s=replace(s,needle,needle+"\n        if ops07_support.continuous(self.run) and self.ops07.holding:\n            p=dict(p,target_us=self.run['parameters']['center_us'],pwm=0)\n            p.pop('pwm_right',None);p.pop('pwm_left',None)")
    # Every normal output frame supplies position/timing before log serialization.
    needle="                        self.log.write(json.dumps({'kind':'sample','wall':time.time(),'vision':self.latest,'proposal':proposal,"
    if s.count(needle)!=1:raise ValueError('sample insertion drift')
    s=s.replace(needle,"                        if ops07_support.continuous(self.run):self.ops07.observe(proposal)\n"+needle)
    return s


def manual_patch(s):
    s='from ops07_runtime import deadline_expired\n'+s
    old="                if type(seconds) not in (int,float) or not math.isfinite(seconds) or seconds<1:raise ValueError('PID duration must be finite and at least 1 second')\n                run['deadline']=self.clock()+seconds;run['pid_target_us']=profile['center_us']"
    new="""                mode=p.get('execution_mode','browser_lease')
                if mode not in ('browser_lease','orin_timed','orin_continuous'):raise ValueError('Unknown PID execution mode')
                if mode!='orin_continuous' and (type(seconds) not in (int,float) or not math.isfinite(seconds) or seconds<1):raise ValueError('PID duration must be finite and at least 1 second')
                run['execution_mode']=mode;run['ops07_hold']=mode=='orin_continuous'
                run['deadline']=None if mode=='orin_continuous' else self.clock()+seconds;run['pid_target_us']=profile['center_us']"""
    s=replace(s,old,new)
    s=replace(s,"self.clock()>=self.current['deadline']","deadline_expired(self.current,self.clock())")
    needle="                            if nb:self.kick.clear()"
    s=replace(s,needle,"                            if run.get('ops07_hold'):\n                                target=run['limits']['center_us'];power=0;rl=None;hard_zero=True\n"+needle)
    return s


def server_patch(s):
    s='import ops07_runtime as ops07_support\n'+s
    s=replace(s,"        elif path == '/api/pid/status':","        elif path == '/api/ops07/status':\n            r=getattr(live_pid,'ops07',None)\n            payload=json.dumps(r.snapshot() if r else dict(state='idle',revision=ops07_support.REVISION)).encode();kind,code='application/json',200\n        elif path == '/api/pid/status':")
    s=replace(s,"            if self.path not in (", "            if self.path not in ('/api/ops07/load','/api/ops07/arm-hold','/api/ops07/release','/api/ops07/pause','/api/ops07/resume','/api/ops07/stop','/api/ops07/reset',")
    s=replace(s,"            if self.path.startswith('/api/pid/'):","            if self.path.startswith('/api/ops07/'):\n                result=ops07_support.api(live_pid,self.path.rsplit('/',1)[1],payload)\n            elif self.path.startswith('/api/pid/'):")
    needle="    live_pid=LivePID(manual_drive,pid_camera,control_operation_lock,motor_tests.busy)"
    s=replace(s,needle,needle+"\n    ops07_support.get_runtime(live_pid)  # restore paused state only; never arm on boot")
    return s


def build():
    OUT.mkdir(parents=True,exist_ok=True);all_info=[]
    transport=(ROOT/'integration_remote.py').read_text(encoding='utf8')
    if "'/api/ops07/status'" not in transport:
        transport=replace(transport,"['/api/pid/layers'","['/api/ops07/status','/api/pid/layers'")
        transport=replace(transport,"['/api/pid/preview-config'","['/api/ops07/load','/api/ops07/arm-hold','/api/ops07/release','/api/ops07/pause','/api/ops07/resume','/api/ops07/stop','/api/ops07/reset','/api/pid/preview-config'")
    ast.parse(transport)
    (OUT/'integration_remote.candidate.py').write_text(transport,encoding='utf8')
    for pkg in PACKAGES:
        package=inspect_package(ROOT/'data/ops07-packages',pkg)
        source=ROOT/'data/ops07-packages'/pkg/'web';dest=OUT/'candidates'/pkg/'web';dest.mkdir(parents=True,exist_ok=True)
        manifest=dict(package=pkg,identity=package['identity'],revision='ops07-orin-r3',deployed=False,files={})
        for p in source.iterdir():
            if p.is_file():shutil.copy2(p,dest/p.name)
        for name,fn in [('live_pid.py',live_patch),('manual_drive.py',manual_patch),('server.py',server_patch)]:
            src=(source/name).read_text(encoding='utf8');patched=fn(src);ast.parse(patched)
            (dest/name).write_text(patched,encoding='utf8')
            (dest.parent/(name+'.diff')).write_text(''.join(difflib.unified_diff(src.splitlines(True),patched.splitlines(True),fromfile='baseline/'+name,tofile='candidate/'+name)),encoding='utf8')
        for name in ('ops07_runtime.py','ops07_route.py'):shutil.copy2(ROOT/name,dest/name)
        tp=ROOT/'data/experiments/track-pos-20261108/candidate'
        for name in ('track_pos.py','track_pos.json'):shutil.copy2(tp/name,dest/name)
        for name,cls,helper in [('curve_map.py','CurveMap','seed_curve_map'),('dash_route.py','DashRoute','seed_dash'),('track_pos.py','TrackPos','seed_track')]:
            p=dest/name;s=p.read_text(encoding='utf8')
            needle='class '+cls+':'
            s=replace(s,needle,needle+'\n    def seed(self, plan):\n        from ops07_route import '+helper+'\n        return '+helper+'(self, plan)\n')
            ast.parse(s);p.write_text(s,encoding='utf8')
        (dest/'ops07-package.json').write_text(json.dumps(package,ensure_ascii=False,indent=2),encoding='utf8')
        (dest/'ops07-site.json').write_text(json.dumps(dict(flag_url='http://192.168.0.149:8080/go',
            source='CAR-INTEGRATION.md',practice=False,network_verified=False),indent=2),encoding='utf8')
        for p in sorted(dest.iterdir()):
            if not p.is_file():continue
            base=source/p.name
            digest=hashlib.sha256(p.read_bytes()).hexdigest()
            before=hashlib.sha256(base.read_bytes()).hexdigest() if base.exists() else None
            manifest['files'][p.name]=dict(before=before,after=digest,changed=before!=digest,
                rollback='restore_exact_backup' if before else 'remove_only_if_candidate_hash_matches')
            if p.suffix=='.py':ast.parse(p.read_text(encoding='utf8'))
        (dest.parent/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
        known=json.loads((ROOT/'data/experiments/ops07-20261108/latest-baseline-2031.json').read_text(encoding='utf8'))
        plan=dict(package=pkg,offline_only=True,remote_readback_required=True,
                  restart_requires_separate_authorization=True,operations=[],inactive=[])
        for name,item in manifest['files'].items():
            if known.get(name)!=item['after']:
                plan['operations'].append(dict(file=name,known_remote_sha256=known.get(name),candidate_sha256=item['after'],
                    precondition='Verify current remote hash; back up exact remote bytes before replacing',
                    rollback='Restore exact backup if preexisting; otherwise remove ONLY when hash still equals candidate'))
        for name in sorted(set(known)-set(manifest['files'])):
            plan['inactive'].append(dict(file=name,action='review_nonreferenced_file_before_optional_move',delete=False))
        (dest.parent/'deployment-plan.json').write_text(json.dumps(plan,indent=2),encoding='utf8')
        all_info.append(dict(package=pkg,changed=[k for k,v in manifest['files'].items() if v['changed']]))
    (OUT/'build.json').write_text(json.dumps(all_info,indent=2),encoding='utf8')
    return all_info


if __name__=='__main__':print(json.dumps(build(),indent=2))
