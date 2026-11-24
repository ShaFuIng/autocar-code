"""Record user-confirmed measurements locally; no vehicle operations."""
import json
from pathlib import Path
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent.parent
OUT = ROOT / 'data/experiments/camera-current-20261031'
OUT.mkdir(exist_ok=True, parents=True)
BASE = 'http://127.0.0.1:8093'

def get():
    with urlopen(BASE + '/api/document/vehicle-measurements', timeout=10) as r:
        return json.load(r)

def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')

doc = get()
write(OUT / 'measurements-before.json', doc)
data = doc['data']
note = '2026-10-31 使用者確認：相機實體安裝角度已調整；目前最近可見地面距離27 cm，零點為前輪輪胎最前緣，沿地面往前量。不是鏡頭地面投影距離。角度度數及新投影尚未量測／重建。'
data['items']['camera_near_ground_front_tire'] = dict(value=27, status='measured', note=note, source='user_confirmed_2026-10-31', reference='front_tire_foremost_edge')
data['items']['camera_near_ground'] = dict(value=None, status='pending', note='新姿態的鏡頭地面投影至最近地面距離未量。舊32 cm為2026-10-19歷史，保留於v7及更早版本；不可用新27 cm直接替換，兩者零點不同。')
data['items']['camera_pitch'] = dict(value=None, status='pending', note='2026-10-31使用者確認相機角度已實際調整並採目前姿態為最新基準；實際俯角度數未量，不能由單一27 cm推定。')
data['notes'] += '\n' + note + '\n以上較早32／42／65 cm等敘述為舊姿態歷史，不代表現況。'
data['camera_info'] += '\n' + note + ' 18px視覺中心與既有camera-inner投影屬原設定，未以本輪量測重新驗證，未修改車端。'
data['camera_pose_current'] = dict(version='camera-current-20261031-r1', confirmed_date='2026-10-31', physical_angle_changed=True, nearest_ground_cm=27, distance_origin='front_tire_foremost_edge', source='user_report', pitch_deg=None, projection_recalibrated=False)
req = Request(BASE + '/api/document', data=json.dumps(dict(key=doc['key'], rev=doc['rev'], data=data)).encode(), headers={'Content-Type':'application/json', 'X-Workbench':'1', 'Origin':BASE})
with urlopen(req, timeout=10) as r:
    saved = json.load(r)
actual = get()
assert actual['data'] == data
write(OUT / 'measurements-current.json', actual)
write(OUT / 'acceptance.json', dict(version='camera-current-20261031-r1', revision=actual['rev'], readback_equal=True, nearest_ground_cm=27, reference='front_tire_foremost_edge', source='user_confirmed', physical_calibration_performed=False, orin_modified=False))
print('Saved and verified vehicle-measurements revision', actual['rev'])
