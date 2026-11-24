"""Update existing workbench and handoff documentation, retaining history."""
from pathlib import Path
import json

root = Path(__file__).resolve().parent
project = root.parent.parent
out = root / 'data/experiments/camera-current-20261031'
p = root / 'integration.js'
s = p.read_text(encoding='utf-8')
(out / 'integration-before.js').write_text(s, encoding='utf-8')
old = "['camera_near_ground','最近可見地面水平距離'"
new = "['camera_near_ground_front_tire','最近可見地面距離（前輪最前緣起算）','2026-10-31使用者確認27 cm；由前輪輪胎最前緣沿地面往前量，非鏡頭投影基準'],['camera_near_ground','最近可見地面水平距離'"
assert s.count(old) == 1
s = s.replace(old, new)
start = s.index('const WORKBENCH_RELEASE={')
end = s.index('\n history:', start)
old_head = s[start:end]
version = 'workbench-camera-facts-20261031.161'
status = '最新確定實體資料（使用者2026-10-31確認）：相機安裝角度已調整，以目前姿態為現況基準；最近可見地面為前輪輪胎最前緣往前27 cm。量測主來源已保存v8，新增前輪基準欄位；舊鏡頭基準32 cm及衍生42／65 cm為歷史。已解除「是否刻意調整尚待確認」的資訊缺口。實際俯角度數、新姿態投影與18px中心適用性尚未重新量測；27 cm不是完整標定。手推e1c9f9e4／749cb80f與舊部署幾何的影像差異及離線比較仍保留，不能把不匹配直接歸因於算法。此次只更新本機資料與交接，未部署、未改PID、未動車。'
pending = ['以目前已確認姿態核對新快照的50／80／120 cm對位與像素座標，必要時重建投影；27 cm單一距離不能決定完整幾何。','核對各錄影的拍攝姿態與校正版本；舊部署camera-inner幾何不自動視為適用，保留像素與遮罩離線比較。','18px視覺中心在新姿態下是否適用待核對，保存值不自行歸零。','既有切換候選的接管跳動、閉迴路表現與Orin耗時仍待相應實驗。']
acceptance = '量測主來源vehicle-measurements v7→v8保存與回讀完全一致：27 cm／前輪輪胎最前緣／使用者確認。舊版量測與總覽保留；這是資料更新驗收，非重新標定、算法或行駛驗收。'
head = 'const WORKBENCH_RELEASE={\n version:'+json.dumps(version)+',\n date:"2026-10-31",\n status:'+json.dumps(status, ensure_ascii=False)+',\n pending:'+json.dumps(pending, ensure_ascii=False)+',\n acceptance:'+json.dumps(acceptance, ensure_ascii=False)+','
s = s[:start] + head + s[end:]
marker = '\n history:['
i = s.index(marker, start) + len(marker)
s = s[:i] + json.dumps('2026-10-31 .161：使用者確認相機角度已調整、前輪最前緣到最近可見地面27 cm；主資料v8。前版.160完整總覽保存於camera-current-20261031/integration-before.js；其「是否調整待確認」已被本輪確認取代。', ensure_ascii=False) + ',' + s[i:]
marker = '<section class="card" id="recentAcceptance"><h2>最近驗收</h2>'
s = s.replace(marker, marker + '<p><b>相機現況已確認：前輪最前緣往前27 cm，量測v8。</b> <a href="/experiment/camera-current-20261031/acceptance.json">資料保存驗收</a> · <a href="/experiment/camera-current-20261031/measurements-current.json">完整量測與來源</a>；未完成新投影標定。</p>', 1)
# Prevent the old derived 42/65 cm explanation being read as current.
s = s.replace('<p>2026-10-19 相機下俯後：', '<p><b>歷史姿態（已由2026-10-31前輪起算27 cm更新取代）：</b>2026-10-19 相機下俯後：', 1)
p.write_text(s, encoding='utf-8')

notice = '''## 最新確定相機資料（2026-10-31，本段優先於下方歷史）

- 使用者明確確認相機實體安裝角度已調整，採目前姿態作為最新實體基準，不再追問是否曾調整。
- 最近可見地面距離 **27 cm**，從 **前輪輪胎最前緣沿地面往前量**；不是鏡頭中心地面投影，也不是前軸起算。
- 主資料：筆電8093的 `vehicle-measurements` **v8**；欄位 `camera_near_ground_front_tire`=27、`status=measured`、來源為使用者確認。舊鏡頭基準32 cm保存在v7歷史，新姿態該欄位未知，不能把27直接填進舊欄位。
- 目前相機實際俯角度數尚未量；27 cm不能單獨求出完整像素→公分投影。既有相機高度14.75 cm、前伸10 cm為過往量測，本輪未重測；66°×40°是舊估算，不能提升為新標定真值。
- 50／80／120 cm對位需核對目前姿態與保存版本；截圖的「待對位」不能當已完成新校正。18px視覺中心保持原設定，適用性待核對，不能自行歸零或宣稱已重新驗證。
- e1c9f9e4與749cb80f的影像比較、消失點差異仍是有效觀測，但「使用者尚未確認相機調整」已過時。拍攝姿態對應需依來源核對，不將舊校正下的路徑失效直接當作候選算法失敗。原影片仍可做影像／遮罩比較和人工標註。
- 本輪只更新本機資料、介面與交接，未改Orin部署、PID、舵機／馬達或控制服務。確定的是實體調整與27 cm量測，不是宣稱新幾何／行駛驗收完成。

資料：`Orin_Interface_v1/laptop_workbench/data/experiments/camera-current-20261031/measurements-current.json`；保存回讀驗收：同目錄 `acceptance.json`。工作台版本 `workbench-camera-facts-20261031.161`。

'''
for target in [project / 'Claude交接_手推雙路口綠線問題與離線修復_2026-10-31.md', root / 'README.md']:
    text = target.read_text(encoding='utf-8')
    (out / (target.stem+'-before.md')).write_text(text, encoding='utf-8')
    first, rest = text.split('\n', 1)
    target.write_text(first+'\n\n'+notice+rest, encoding='utf-8')
with (project / 'AGENTS.md').open('a', encoding='utf-8') as f:
    f.write('\n\n'+notice)
print('Updated measurement UI, four overview sections, README, AGENTS and existing Claude handoff.')
