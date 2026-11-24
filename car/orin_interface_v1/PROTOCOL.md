## 2026-10-14：v1.4.0 手動模式

此部署仍禁止舊 ARM/MOVE 與 BLE 行車；沿用 PROFILE、SERVO、CENTER、TEST_ARM/TEST/TEST_END。新命令沿用序號、CRC、generation，保持單一 UART 擁有者。

| 命令內文 | 條件／效果 |
| --- | --- |
| `SEQ,MANUAL_ARM,GEN` | ORIN/READY、新實體啟動、PCA 正常、profile 已同步；進入 MANUAL，馬達零輸出，尚未移動舵機。 |
| `SEQ,MANUAL,GEN,R,L,US` | MANUAL、有效 GEN、R/L 整數 ±25%、同方向或零、US 在 profile 限位內。首包必須 R=L=0 且 US=profile.center，明確初始化舵機。每筆有效指令續 500 ms lease。 |
| `SEQ,MANUAL_END,GEN` | 停止鎖定、generation 更新，需要新的實體啟動。 |

零動力指令立即撤除 PWM；非零動力沿用 ramp。舵機每 120 ms 最多改 10 µs，追蹤最新 US；放鍵回中心由 Orin 產生目標。正反切換由 MCU 檢查前次非零命令至少 500 ms、PWM=0、編碼器 delta=0，未滿足回 DIRECTION_CHANGE_WAIT，不延長租期。

MANUAL 中拒絕 SYNC、PROFILE、校準 SERVO/CENTER 與 TEST_ARM；STOP 和實體急停仍優先。保存中心時 Orin 在退出手動後才同步 PROFILE，行駛中只送既有限位內的 US。主狀態新增 MANUAL，cmd_age_ms 與 D12 指示包含此模式。詳細前端意圖/保存/錄影規則見 MANUAL_DRIVING.md。

以下舊版通用協定段落保留作背景；若 ARM/MOVE 能力與本節衝突，以本節所述部署版本為準。

# Orin UART v1 協定

這是本專案新定義的介面，不宣稱與廠商的既有 UART PDF 相容。
Serial1 115200 baud、8N1、無流量控制。文字編碼 ASCII，指令大寫。

## Orin → 控制板

```text
$SEQ,COMMAND[,ARG...]*CCCC\n
```

- `$` 是起始符號；收到新的 `$` 重新開始收包。
- `SEQ` 是十進位 uint32。SYNC 後一般指令序號必須遞增；允許 uint32 繞回，但單次間距須小於 2^31。
- CRC16/CCITT-FALSE：poly=0x1021、init=0xFFFF、refin=false、refout=false、xorout=0。
- CRC 範圍是 `$` 後到 `*` 前的所有 ASCII bytes，不包含 `$`、`*`、CRC、換行。
- `CCCC` 是恰好四個十六進位字元。結尾 LF，可使用 CRLF。
- 自 `$` 開始 100 ms 內須收完，內文最多 127 字元。
- CRC/格式錯誤直接丟棄並計數 `rx_bad`，不把不可相信的序號拿來回 ACK。

CRC 範例：`$1,STOP*CC4D\n`。Python 的 `encode_frame()` 可直接產生所有命令。

| 命令內文 | 條件／效果 |
|---|---|
| `SEQ,SYNC` | LOCKED、READY 或 FAULT 才接受；建立序號起點，generation 加一，回 ACK。ARMED 時拒絕。 |
| `SEQ,STATUS` | 需先 SYNC；回 status 與 ACK，不延長 MOVE 租期。狀態也會自動約 5 Hz 廣播。 |
| `SEQ,ARM,GEN` | ORIN 模式、READY、急停未按、PCA 正常、GEN 相符；進入 ARMED，開始 500 ms 租期。 |
| `SEQ,MOVE,GEN,T,S` | ORIN 模式、ARMED、GEN 相符；T/S 各為整數 −100～100，首包必須 0,0。T 限幅到 ±25%。 |
| `SEQ,STOP` | 撤除動力輸出、鎖定、GEN 加一；不要求 SYNC 或新序號，CRC 正確即接受。 |
| `SEQ,MODE,ORIN` | 需先 SYNC，且已 LOCKED；停用 BLE 控制及嘗試關閉廣播。 |
| `SEQ,MODE,BLE` | 需先 SYNC，且已 LOCKED；初始化 BLE/開廣播，等待手機連線及新的實體啟動。 |

USB 也能接收完整框架，與 Serial1 共用序號會話。只允許一個控制指令寫入者；
平常 USB 用 `?`、`!` 診斷，不與 Orin 同時傳行車框架。

一般有效格式但拒絕的指令會消耗其新序號；不能修正同一序號後重送。
重複 STOP 仍有效，且每次更新 GEN。錯誤/重複 MOVE 都不能更新行車時間。
停止不會主動回正；新的啟用流程要求中立 MOVE，屆時回正。

`GEN` 從 ACK/status 取得，是本次開機內的授權版本，不能跨重開機存用。
CRC、序號與 GEN 用於完整性及舊命令排除，並非加密或身分驗證。
重新連線後先讀狀態；如果仍 ARMED，先 STOP，SYNC，重新實體啟動，再 ARM。禁止自動續跑。

## 控制板 → Orin

每行一個 JSON 物件，LF 結尾；v1 回覆沒有附 CRC。
Orin 要檢查 JSON、必須欄位及相符 ACK；缺少 ACK/狀態時停止送 MOVE，不能僅因 write 成功就宣稱執行成功。

```json
{"type":"ack","seq":1,"ok":true,"reason":"OK","generation":1}
```

ACK `ok=true` 代表接受命令，不代表已量測到馬達轉動或輪角到位。
錯誤 reason 是固定代碼。事件包含 `ms/event/detail/generation`。
status 完整欄位見 `Telemetry.cpp`，主要為：

- `mode/state/reason/generation/neutral_required`：控制權、授權與停止原因。
- `cmd_age_ms`：ARMED 時最後接受的 MOVE 距今時間；其他狀態 −1。
- `pins`：D10/D11 的邏輯讀值、D12 輸出命令。
- `motor_a/motor_b`：PWM 命令、斜坡目標、目前方向及換向目標。
- `steer_us_command`：舵機脈寬設定。
- `pca_ok/i2c_errors/ble_*`：模組診斷。
- `enc_a/enc_b`：編碼器脈衝資訊，未驗證接線/比例前不能換算車速。
- `voltage_v/current_a/speed_mps`：v1 固定 null。
- `rx_good/rx_bad/rx_rejected/tx_dropped/max_loop_us`：傳輸及迴圈診斷。

MOVE 建議 20 Hz。新授權 ARM 後先立即送 0,0，再持續送希望的動力/轉向值；
狀態查詢與 heartbeat 不能替代 MOVE。停止行車測試應送 STOP，不能只依賴工具退出。


## 舵機校準擴充（orin-v1.1.2-cal）

`SEQ,SERVO,generation,pulse_us` 仍使用原 CRC16 外框，只能在校準韌體 READY 且急停放開時接受。必須 SYNC，序號與 generation 規則相同。第一筆只能明確設定 1340 µs 參考位置；之後單次最多 ±100 µs、兩次至少間隔 100 ms、電氣測試範圍 500～3000 µs。這些不是機械行程或角度量測。校準模式的 READY 不再因 30 秒閒置鎖定；STOP／急停／故障仍停用脈波並鎖定。ARM／MOVE 禁止。


### v1.1.3-cal 手動測試

取消人為脈寬行程範圍。SERVO 命令保留單次最多100 µs、間隔至少100 ms、初始化1340、READY／急停／序號／generation；脈寬轉換必須符合PCA9685 1～4095計數器。USB +/-每次100 µs，不會自動連轉。


## v1.2.0-cal 中心與限位設定

- `SEQ,PROFILE,generation,revision,min_us,center_us,max_us`：只有LOCKED或READY可更新，generation必須相符。更新不輸出PWM、不授予啟動。如果已啟用，範圍不能排除目前指令；必須min<center<max，符合PCA有效計數器。PROFILE同步前SERVO/CENTER拒絕。
- `SEQ,CENTER,generation`：必須READY、急停放開、舵機已啟用且PROFILE同步。ACK代表接受逐步回中心，並非完成；查看servo_returning及steer_us_command。每120ms改10us，STOP/急停/故障或generation改變會中斷。
- 首次SERVO必須設定到已同步的中心，不再固定1340。後續所有SERVO都受已同步min/max限位。USB i使用已同步中心、+/-每次10us。
- 狀態增加servo_profile（synced/revision/min_us/center_us/max_us）和servo_returning。


## v1.3.0-bench 後輪測試

- `SEQ,TEST_ARM,generation,duration_ms`：READY、profile已同步、急停放開、舵機未回中心。duration 500～30000ms，進入MOTOR_TEST，不動力輸出。
- `SEQ,TEST,generation,right_pct,left_pct`：各側±25%，第一筆必須0,0，第二筆開始計時。有效新指令續命500ms，查詢、錯誤與重複序號不續命。左右獨立，舵機保持不變。
- `SEQ,TEST_END,generation`或韌體定時完成：PWM歸零、READY、generation更新、保留舵機設定；不能用舊TEST恢復。STOP／急停／故障／逾時則LOCKED，要求新實體啟動。
- 測試時SERVO、CENTER、PROFILE、MODE、SYNC不能改變動作授權，舊ARM/MOVE仍禁止。
- 狀態新增MOTOR_TEST及motor_test.active/started/start_ms/duration_ms/right_pct/left_pct、enc_sample_ms。Orin取樣使用累積編碼器計數，正規化右輪-1、左輪+1。
