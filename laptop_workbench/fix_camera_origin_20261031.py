"""Correct recorded distance origin and local scene; no vehicle connection."""
import json
from pathlib import Path
from urllib.request import Request, urlopen
R=Path(__file__).resolve().parent
O=R/'data/experiments/camera-origin-20261031';O.mkdir(parents=True,exist_ok=True)
B='http://127.0.0.1:8093'
def dump(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf8')
with urlopen(B+'/api/document/vehicle-measurements') as r:d=json.load(r)
dump(O/'before.json',d)
note='2026-10-31使用者更正：過去39 cm、32 cm及目前27 cm一律從前輪輪胎最前緣沿地面往前量。先前鏡頭投影基準為助手誤記；42／65／49 cm及其反推俯角不成立，不能沿用。原始版本保留供稽核，解讀以本更正為準。'
d['data']['distance_origin_correction']={'date':'2026-10-31','source':'user_confirmation','origin':'front_tire_foremost_edge','historical_values_cm':[39,32],'current_value_cm':27,'note':note}
d['data']['items']['camera_near_ground']['note']='鏡頭投影基準沒有直接量測。過去39／32 cm均為前輪最前緣起算，被誤記在本欄；見distance_origin_correction。'
d['data']['notes']+='\n'+note
d['data']['camera_info']+='\n'+note
req=Request(B+'/api/document',data=json.dumps(d).encode(),headers={'Content-Type':'application/json','Origin':B,'X-Workbench':'1'})
with urlopen(req) as r:saved=json.load(r)
with urlopen(B+'/api/document/vehicle-measurements') as r:actual=json.load(r)
assert actual['data']==d['data'];dump(O/'current.json',actual)

for name in ['scene-source.js','integration.js']:
 p=R/name;s=p.read_text(encoding='utf8');(O/(name+'.before')).write_text(s,encoding='utf8')
 def replace(a,b):
  global s
  assert a in s, (name,a[:60])
  s=s.replace(a,b,1)
 replace('const nearMeasured=items.camera_near_ground?.value??null;', 'const nearMeasured=items.camera_near_ground_front_tire?.value??null;\n const frontEdgeX=fd/2, measuredCameraNear=nearMeasured===null?null:frontEdgeX+nearMeasured-camX;\n assumptions.push("最近可見距離以前輪輪胎最前緣為零點；前輪外徑未量時使用暫估值，只影響白模座標與反推俯角，不改27 cm實量。過去39／32 cm也同此基準。");')
 replace('items.camera_pitch?.value==null&&nearMeasured>0','items.camera_pitch?.value==null&&measuredCameraNear>0')
 replace('Math.atan(measured.height/nearMeasured)','Math.atan(measured.height/measuredCameraNear)')
 replace('由近界與估算視角反推，非實量','由前輪基準近界、暫估輪徑與估算視角換算，非實量')
 replace('let rev=record.rev, s={...measured,...record.data.parameters},draft;', "const savedCompatible=record.data.measurement_revision===measure.rev&&record.data.distance_reference==='front_tire_foremost_edge';\n let rev=record.rev, s={...measured,...(savedCompatible?record.data.parameters:{})},draft;")
 replace('draft.measurement_revision===measure.rev','draft.measurement_revision===measure.rev&&draft.distance_reference===\'front_tire_foremost_edge\'')
 replace("rev?'已載入模擬方案 v'+rev", "savedCompatible?'已載入模擬方案 v'+rev")
 replace('紫色：已保存 ${nearMeasured??\'未知\'} cm 參考線。','紫色：前輪輪胎最前緣往前 ${nearMeasured??\'未知\'} cm 實量參考線；橘色依模擬參數計算。')
 replace('if(nearMeasured!=null)poly([[camX+nearMeasured,.22,-22],[camX+nearMeasured+.3,.22,-22],[camX+nearMeasured+.3,.22,22],[camX+nearMeasured,.22,22]]', 'if(nearMeasured!=null)poly([[frontEdgeX+nearMeasured,.22,-22],[frontEdgeX+nearMeasured+.3,.22,-22],[frontEdgeX+nearMeasured+.3,.22,22],[frontEdgeX+nearMeasured,.22,22]]')
 replace("if(near!==null&&near<=180)annotation([camX,.3,32],[camX+near,.3,32],'最近地面 ≈'+near.toFixed(1)+' cm');", "if(nearMeasured!==null)annotation([frontEdgeX,.3,32],[frontEdgeX+nearMeasured,.3,32],'前輪最前緣 → 最近地面 '+nearMeasured+' cm（實量）');")
 replace('最近地面（鏡頭投影往前）：${near===null?\'無前方平地交點\':near.toFixed(1)+\' cm\'}；距前軸：', '實量最近地面（前輪最前緣往前）：${nearMeasured??\'未知\'} cm。模擬最近地面（同基準）：${near===null?\'無前方平地交點\':(camX+near-frontEdgeX).toFixed(1)+\' cm\'}；模擬距前軸：')
 s=s.replace('measurement_revision:measure.rev}', "measurement_revision:measure.rev,distance_reference:'front_tire_foremost_edge'}")
 s=s.replace("measurement_revision:measure.rev,kind:'simulation-only'", "measurement_revision:measure.rev,distance_reference:'front_tire_foremost_edge',kind:'simulation-only'")
 if name=='integration.js':
  s=s.replace('workbench-camera-facts-20261031.161','workbench-camera-origin-20261031.162')
  s=s.replace('舊鏡頭基準32 cm及衍生42／65 cm為歷史。','歷史39／32 cm也從前輪最前緣起算，先前鏡頭基準為誤記；衍生42／65／49 cm與舊反推俯角作廢。3D圖改由前輪最前緣畫27 cm，舊基準模擬不自動載入。')
  s=s.replace('量測主來源已保存v8','量測主來源已保存v9')
  s=s.replace('pending:[','pending:["歷史距離解讀已更正為前輪最前緣；仍需新姿態投影與實際俯角核對。",',1)
  s=s.replace('acceptance:', 'acceptance:',1)
  s=s.replace('量測主來源vehicle-measurements v7→v8保存與回讀完全一致：','量測主來源vehicle-measurements v9歷史基準更正與回讀一致；3D前輪起點／27 cm顯示修補：')
  s=s.replace('量測v8。</b>','量測v9；歷史39／32 cm同為前輪基準。</b>')
  s=s.replace('/experiment/camera-current-20261031/acceptance.json','/experiment/camera-origin-20261031/acceptance.json')
  s=s.replace('\n history:[','\n history:["2026-10-31 .162：更正所有歷史最近可見距離基準（前輪最前緣）；量測v9，3D圖起點／參考線／草稿相容修補，舊反推距離及俯角作廢。",',1)
 p.write_text(s,encoding='utf8')

notice='''## 2026-10-31 距離基準再次更正（本段優先）

使用者明確澄清：歷史39 cm、32 cm與目前27 cm，**全部由前輪輪胎最前緣（使用者所指車子最前端）沿地面往前量**，從來不是鏡頭地面投影。先前助手的記錄與交接解讀錯誤。舊版本保留供追溯，但不可依舊欄位名稱解讀成鏡頭距離；先前衍生的前軸42／49 cm、後軸65 cm及反推俯角均不成立。

最新主資料為vehicle-measurements v9：前輪基準27 cm，鏡頭基準未知；camera-origin-20261031/current.json保留完整資料。相機實體角度確已調整，實際角度度數仍未量測。3D圖由前輪最前緣畫27 cm實量線，舊32 cm模擬不自動套入；前輪直徑暫估7.5 cm只用於示意座標及模擬角度換算，不是新量測。未改投影、PID或車端。

工作台版本workbench-camera-origin-20261031.162。下方.161「舊鏡頭基準32 cm」等說法均由本段取代。

'''
for p in [R/'README.md',R.parent.parent/'Claude交接_手推雙路口綠線問題與離線修復_2026-10-31.md',R.parent.parent/'AGENTS.md']:
 t=p.read_text(encoding='utf8');a,b=t.split('\n',1);p.write_text(a+'\n\n'+notice+b,encoding='utf8')
dump(O/'acceptance.json',{'revision':actual['rev'],'readback_equal':True,'origin':'front_tire_foremost_edge','current_cm':27,'historical_cm':[39,32],'old_derived_geometry_invalid':True,'vehicle_modified':False})
print('Updated measurement revision',actual['rev'])
