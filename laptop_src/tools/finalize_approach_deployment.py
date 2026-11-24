import json,shutil,hashlib
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'laptop_workbench/data/experiments/k-approach'
deployment=json.loads((out/'deploy-orin.json').read_text(encoding='utf8'))
preview=json.loads((out/'preview-orin.json').read_text(encoding='utf8'));post=json.loads((out/'status-orin.json').read_text(encoding='utf8'))
assert len(preview['samples'])==10 and all(x['valid'] and x['path_diagnostic']['path_revision']=='k-approach-live-r1' for x in preview['samples'])
assert post['pid']['state']=='idle' and post['vehicle']['state']=='LOCKED'
assert post['vehicle']['motor_a']['pwm_command']==post['vehicle']['motor_b']['pwm_command']==0
assert post['pid']['run']['parameters']['max_gap']==.4
assert post['pid']['run']['id']==deployment['before']['pid']['run']['id'] and post['pid']['run']['state']=='stopped'
before=out/'local-before-deployment';before.mkdir(exist_ok=True)
for name in ['k_parallel_path.py','k_lane_live_path.py','junction_approach_candidate.py']:
 dest=root/'web'/name
 if dest.exists() and not (before/name).exists():shutil.copy2(dest,before/name)
 raw=(out/'candidate-runtime'/name).read_bytes()
 assert hashlib.sha256(raw).hexdigest()==deployment['files']['web/'+name]
 dest.write_bytes(raw)
reportfile=out/'report.json'
if not (out/'offline-report.json').exists():shutil.copy2(reportfile,out/'offline-report.json')
r=json.loads(reportfile.read_text(encoding='utf8'));r.update(status='deployed_stationary_preview_verified',deployment=deployment,
 stationary_preview=dict(samples=10,valid=10,revision='k-approach-live-r1',minimum_age_ms=min(x['control_age_s'] for x in preview['samples'])*1000,maximum_age_ms=max(x['control_age_s'] for x in preview['samples'])*1000,note='Sampled preview consumption age, not actual driving PID consumption or a continuous worst-case measurement'),
 final_state=dict(pid='idle',vehicle='LOCKED',pwm=[0,0],i2c_errors=post['vehicle']['i2c_errors']),motor_started=False,vehicle_service_restarted=False)
r['limits']=[x for x in r['limits'] if 'deployment' not in x]+['Deployed worker files only; no PID trial or actuator commands; 400ms unchanged']
reportfile.write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf8')
f=root/'laptop_workbench/integration.js';s=f.read_text(encoding='utf8');shutil.copy2(f,out/'integration-before-deployment.js')
s=s.replace('workbench-approach-offline-20261024.52','workbench-approach-deployed-20261024.53')
for key,value in [('status','路口接近時序r1已部署Orin，原完整K選項沿用。即時預覽確認k-approach-live-r1，10筆新影格皆有效；未啟動PID或馬達。'),('acceptance','原版已備份；車控MainPID17413不變，僅重載worker。預覽抽樣年齡96～137ms，結束PID idle、LOCKED、雙輪0、I²C0。離線1862格比較保留，400ms不變；待使用者自行試跑。')]:
 start=s.index(' '+key+':',s.index('const WORKBENCH_RELEASE='));end=s.index('\n',start);s=s[:start]+' '+key+':'+json.dumps(value,ensure_ascii=False)+','+s[end:]
s=s.replace('路口接近時序候選僅離線；最新輪錄影止於路口內，該輪接回A與改變行車軌跡後效果尚無實車資料。等待部署指示。','路口接近時序r1已部署；使用者自行試跑後核對實際切換、接回A、路徑與控制資料年齡。')
s=s.replace('完整K · 路口接近時序候選（離線，未部署）','完整K · 路口接近時序 r1（已部署 Orin）')
s=s.replace('原長片1760格完整重播，保留前段歷史；最新輪102格平行候選與順序計算結果一致。只改切換，未加入隧道候選；原版保留、400ms不變。','原長片1760格完整重播，最新輪102格平行計算一致。已部署k-approach-live-r1；不動車預覽10筆新影格皆有效，資料年齡96～137ms（非實車PID驗證）。原版備份、400ms不變；未加入隧道候選。預覽已停止，使用原完整K選項自行試跑。')
f.write_text(s,encoding='utf8')
with (root/'laptop_workbench/README.md').open('a',encoding='utf8') as f:f.write('\n\n## .53 路口接近時序r1已部署\n部署web/k_parallel_path.py、junction_approach_candidate.py、k_lane_live_path.py，備份 '+deployment['backup']+'。原車控MainPID17413未重啟，worker透過預覽重新載入；版本k-approach-live-r1。預覽10筆皆有效，抽樣年齡96～137ms，非實車PID資料年齡驗證。結束idle/LOCKED/雙輪0/I²C0；原完整K選項沿用、400ms未改，未啟動馬達。離線報告另存offline-report.json，總覽與模型頁同步更新。\n')
print('Deployment report, local sources and workbench updated')
