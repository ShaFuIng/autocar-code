from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=ROOT/'Telemetry.cpp';s=p.read_text(encoding='utf-8');s=s.replace(r'\"web_timed_test_start\":%s',r'\"web_calibration_start\":true,\"web_timed_test_start\":%s');p.write_text(s,encoding='utf-8')
p=ROOT/'web/manual.html';s=p.read_text(encoding='utf-8')
s=s.replace("v?.state!=='READY'||v?.fw", "!(v?.state==='READY'||(v?.state==='LOCKED'&&v?.web_manual_start===true))||v?.pins?.D11!==1||v?.fw")
s=s.replace("v?.state!=='READY'?'請放開急停，按下及放開實體啟動鍵'", "v?.pins?.D11!==1?'實體急停尚未釋放':!(v?.state==='READY'||(v?.state==='LOCKED'&&v?.web_manual_start===true))?'目前狀態／韌體不允許網頁啟用'")
s=s.replace('先按放實體啟動鍵，確認相機與車況，再啟用手動模式。','確認相機與車況後，按網頁啟用即可；支援的韌體不需實體 START。')
s=s.replace('已要求停止並鎖定；重新按放實體啟動鍵才可再啟用。','已要求停止並鎖定；確認環境後可再按網頁啟用，不會自動恢復。')
s=s.replace('先勾選周圍環境確認，再啟用手動模式。','實車授權獨立於模擬任務；先確認環境，再按網頁啟用。')
p.write_text(s,encoding='utf-8')
p=ROOT/'web/index.html';s=p.read_text(encoding='utf-8')
s=s.replace("'關閉 · 按實體啟動鍵開啟'", "'關閉 · 點擊啟用校準（零動力）'")
s=s.replace("document.getElementById('enableSwitch').disabled=!!d?.motor_busy;", "document.getElementById('enableSwitch').disabled=busy||!!d?.motor_busy||(!authorized&&!(fresh&&correct&&vehicle?.web_calibration_start===true&&vehicle?.state==='LOCKED'&&vehicle?.pins?.D11===1&&d?.servo_settings_synced));")
s=s.replace('LOCKED：放開急停，按一下實體啟動鍵再放開。','未啟用：急停釋放、設定同步後，點擊網頁開關；FAULT 不可啟用。')
s=s.replace("else message.textContent='請按實體啟動鍵；網頁不能取代急停後的重新啟動。'", "else sendServo({action:'enable',generation:vehicle?.generation})")
s=s.replace("else message.textContent=payload.action==='stop'?", "else message.textContent=payload.action==='enable'?'校準已啟用，未送位置／動力指令；等待車況更新。':payload.action==='stop'?")
p.write_text(s,encoding='utf-8')
p=ROOT/'web/manual_drive.py';s=p.read_text(encoding='utf-8').replace('Synchronize settings and press/release physical START','Check synchronized settings, ESTOP and web-start authorization');p.write_text(s,encoding='utf-8')
p=ROOT/'laptop_workbench/integration.js';s=p.read_text(encoding='utf-8')
old='實體 START 授權成為 READY 後，可連續調整舵機或執行正常完成的限時馬達測試，不用每次重按。每個量測點不必按「關閉」；關閉並鎖定、急停或失聯停止後，才需重新實體授權。網頁不會自動解除急停鎖定。'
new='03 用於舵機校準／固定 PWM 限時測試；04 用於手動駕駛／直行微調／錄影。目前共用既有控制介面與後端。支援網頁授權的韌體不需按實體 START：校準按「啟用校準」，駕駛按「啟用手動模式」。正常啟用期間可持續調整；停止、急停或失聯後不自動恢復，確認環境後再按網頁啟用。實體急停與故障仍會阻擋啟動。上方「停止模擬任務」只停止模擬；實車請用「實車停止並鎖定」。'
s=s.replace(old,new);p.write_text(s,encoding='utf-8')
