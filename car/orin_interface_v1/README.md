> 目前為 **orin-v1.4.0-manual**：新增 WASD 手動駕駛、即時中心/左右 PWM 微調及保存、相機失聯停止、影片與控制紀錄。操作入口 `/manual`，詳見 [MANUAL_DRIVING.md](MANUAL_DRIVING.md)。舊 ARM/MOVE 自主行車仍禁止；原舵機校準和定時後輪測試保留。

# BMduino → Jetson Orin 介面第一版

目前版本：`orin-v1.4.0-manual`，更新日期：2026-10-14。以下 v1.0.1 段落為早期背景，手動模式依 MANUAL_DRIVING.md。

來源是 `程式修復_新增Serial1/2026_9_17_Serial1_Fix/2026_9_17_Serial1_Fix.ino`。
原版未修改；本資料夾是獨立 Arduino 專案，主檔與資料夾同名。

## 這版完成什麼

### 2026-10-09：行車 ACK 遺失修正

實車紀錄顯示 ARM、中立 MOVE 與大部分行車 ACK 正常，但約每 200 ms 有指令/ACK 序號缺口，
控制板 `rx_bad/rx_rejected/tx_dropped` 沒有增加。檢查本機 HT32 core 1.3.0 發現：
`noInterrupts()` 會先 `__disable_irq()`，接著呼叫 `SerialAllFlush()`。
原版編碼器每 200 ms 拷貝計數時使用它，因此會在全域中斷關閉期間等待串口送完，造成 UART 接收遺失的風險。

v1.0.1 改成 CMSIS IRQ 遮罩：保存 PRIMASK、短暫關閉中斷、只拷貝計數欄位，再還原原始遮罩。
不再經過 HT32 的串口 flush。桌面測試新增此套件行為模型，驗證無 flush/等待並保留原本中斷狀態。
此修正須重新燒錄控制板；Python 的開埠 2 秒等待仍保留，ACK 遺失停止條件維持不變。
待實車重測確認行車序號是否完整；不宣稱所有開埠亂碼都由此問題造成。

- Orin 透過 Serial1（115200、8N1）控制兩顆驅動馬達與轉向舵機。
- 預設 ORIN 模式，BLE 行車控制與廣播預設停用；保留原手機 `M + throttle + steer` 三位元組介面。
- 本地實體急停優先；啟動按鈕必須經過放開及新的按下動作，不能持續按住自動重啟。
- 行車指令逾時 500 ms，立即把 D8/D9 PWM 設定歸零並鎖定；查詢、錯誤封包、重複序號不續命。
- 故障、停止、模式切換後需要重新實體啟動；第一筆行車指令必須是動力 0、轉向 0。
- 封包有序號及 CRC16；每次停止/啟動更新 generation，拒絕舊授權的行車指令。
- JSON Lines 狀態約 5 Hz，包含按鈕邏輯、馬達輸出設定、舵機設定、通訊與 I²C 診斷、編碼器脈衝。
- 固定大小接收器、每輪接收配額及輸出佇列；UART/BLE 行車接收不使用 `readString()` 或等待封包補齊。
- 初次驗證的動力設定限幅 ±25%，兩個控制來源都適用。這是 PWM 比例上限，不是車速上限。

## 硬體與限制

**實車尚未實作動力斷電；這份韌體的急停是軟體停止，不會切斷電池。**
控制板當機、電源異常、驅動器故障等情況不能靠這份程式保證停止。
零輸出的實際結果（滑行或煞車）須依 XY160D 實車驗證，不宣稱已實現主動煞車。

實車驅動器已由使用者確認為 XY160D；圖面標示 `DRV1011`，且 PCA 馬達通道與原程式存在差異。
本版以已確認可用的原程式馬達/舵機映射為準，未按舊圖更換輸出腳位。
原圖還有 74LS14 的 EN 訊號反相路徑，不能僅憑圖面把 PWM 設定當成驅動器端的實測波形。
上架測試時應確認 PWM=0 的確不產生動力；若不符，先停止測試並核對實車接線。

| 功能 | 採用設定 |
|---|---|
| 實體啟動 | D10、INPUT_PULLUP，LOW 為按下 |
| 實體急停 | D11、INPUT_PULLUP，LOW 為按下 |
| 啟動燈 | D12，ARMED 亮，其他狀態滅 |
| 馬達 A PWM | D8；PCA9685 CH4/CH5，極性反轉 |
| 馬達 B PWM | D9；PCA9685 CH6/CH7，極性正常 |
| 轉向 | PCA9685 CH0、50 Hz，中心 1340 μs，範圍 ±700 μs |
| Orin UART | Serial1，原版 D20/RX1、D21/TX1 |
| BLE | Serial3，掃描名稱 BMC_CAR |
| 編碼器 A | 暫定 D4=A、D5=B |
| 編碼器 B | 暫定 D6=A、D7=B |

BMduino 已安裝的 BM53A367A core 支援 D4～D7 中斷；程式採 AB 雙邊緣四倍頻解碼。
編碼器腳位由圖面推定，尚未確認實車接線、車輪身份與正負方向。
`Config.h` 可修改配對及 `ENC_A_SIGN`/`ENC_B_SIGN`。`ENCODERS_ENABLED=false` 可停用。
這些 D4～D7 是 BMduino GPIO，與 PCA9685 的 CH4～CH7 是不同硬體，不會因數字相同而互占。

目前編碼器是診斷用途，不參與速度閉迴路，也不會因無脈衝自動判定堵轉。
輸入使用內部上拉；未接線通常維持 HIGH，但雜訊也可能產生假脈衝。
不能只憑 `seen=true` 宣稱已接妥，要確認手轉輪子的連續 AB 訊號與兩個轉動方向。

電池電壓、電流、真實車速尚無已驗證量測/校正資料，固定回報 `null`。
PWM 和舵機脈寬都是**命令設定**，不是實測電壓、轉速或輪角。

## Arduino 開啟及燒錄

1. 開啟本資料夾的 `Orin_Interface_v1.ino`；其餘 `.h/.cpp` 一起參與編譯。
2. 選擇 BMduino-UNO 的 BM53A367A（HT32F52367），並沿用原本能成功燒錄的上傳方式與連接埠。
3. 函式庫沿用原專案：Adafruit PWM Servo Driver、Adafruit BusIO、BM7701-00-1。
4. 第一次燒錄後先保持驅動輪離地，不按啟動；確認開機 LOCKED、BLE 停用、兩輪無動力。

桌面編譯已針對 `HT32:HT32:BM53A367A` 檢查；並未自動燒錄到車上。
如果你實際選用的是 IAP Mode，請維持既有 IAP 燒錄流程，不能直接拿一般模式產物當 IAP 產物上傳。

## Orin 操作

把 `tools` 複製到 Orin，例如 `~/orin_interface_v1/tools`。在 Orin 終端執行：

```bash
cd ~/orin_interface_v1/tools
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python orin_client.py status
python orin_client.py --log session.jsonl monitor
```

預設埠 `/dev/ttyTHS1`。若權限不足，將使用者加入 `dialout` 後重新登入：

```bash
sudo usermod -aG dialout "$USER"
```

**一個 UART 只能有一個讀取程式。** 先結束先前的 `sudo cat /dev/ttyTHS1`，也不要同時開 monitor 和 drive；
它們會搶走彼此的 ACK/狀態。測試工具以 115200、8N1、無軟硬體流量控制開啟串口。

2026-10-09 工具更新：每次開啟串口後，第一次讀狀態或送指令前，會先清除當時殘留的接收資料，
持續接收 2 秒再進行操作。這是依本車「先收 2 秒，再送 STOP 可收到 ACK」的實測加入的處理，
尚未證明開埠亂碼的根本原因。同一連線後續指令不再等待，也不再清空接收資料，以免丟掉 ACK。
期間若仍有亂碼，工具繼續顯示 `Unparsed UART`；若持續發生或 STOP 沒有 ACK，先停止動力測試。
新開串口的 `stop` 命令也包含這 2 秒等待，不作為即時急停使用；本地實體急停仍由控制板獨立檢查。

### 第一輪：不驅動，確認編碼器

維持 LOCKED，執行 monitor，手動轉動輪子，觀察對應的 `enc_a` 或 `enc_b`：

- `seen=false`：本次開機尚未觀察到任何邊緣，不能據此分辨未接線或輪子未動。
- `forward/reverse`：兩個解碼方向的累積有效計數，名稱尚未等同實車前進/後退。
- `delta`：約 200 ms 取樣區間內的帶正負計數，不是 rpm 或 m/s。
- `edges`：所有觀察到的 AB 邏輯變化。
- `invalid`：AB 同時翻轉的非法轉換累積量；可能是雜訊、漏取樣或接線問題。
- `age_ms=-1`：尚未觀察到邊緣；其他值是距離最後邊緣的時間。
- `ab`：AB 二位元邏輯狀態，0～3。

### 第二輪：Orin 小動力測試

先結束 monitor，保持輪子離地，放開實體急停。

```bash
python orin_client.py --log drive_test.jsonl drive --throttle 10 --steer 0 --seconds 2
```

工具會等待最多 30 秒的新實體啟動授權。按照畫面提示按下再放開 D10 啟動按鈕。
ORIN 模式下啟動鈕使狀態變 READY；工具傳送 ARM 與中立 MOVE 後，才送出 20 Hz 行車指令。
測試結束或 Ctrl+C 會送出 STOP 並鎖定，下一次必須重新實體啟動。
如果程式被強制終止或 UART 斷線，韌體會在最後有效 MOVE 後逾時停止。

先驗證 10% 前進，再驗證 `--throttle -10` 後退；舵機可先用 `--throttle 0 --steer 10` 小幅測試。
轉向設定仍沿用原版範圍，尚未校正實車輪角及機械端點，不宜一開始直接測 ±100。

### 第三輪：急停及逾時

- 行車測試中按下急停：狀態鎖定，PWM 命令歸零，工具回報停止/授權變更。
- 放開急停或串口恢復：不能自動恢復行駛；需重新按啟動並執行新測試。
- 驗證逾時可暫停行車程序（例如另一終端 `kill -STOP <PID>`），觀察輪子失去動力及稍後 log 的 COMMAND_TIMEOUT；
  程序恢復後舊 generation 指令仍被拒絕。

這些是待實車驗收步驟；桌面模擬通過不等於已驗證實際停止時間。

### 手機備援

先停止並鎖定，再切換：

```bash
python orin_client.py stop
python orin_client.py mode BLE
```

用原手機 App 連線 BMC_CAR，保持搖桿中立，**連線後**再按實體啟動。
BLE 模式由實體啟動直接進 ARMED，500 ms 內須收到新的中立移動封包，之後才能行車。
手機必須持續送封包，即使搖桿位置不變；若 App 只在搖桿變化時發送，會觸發逾時，需修改 App 發送策略。
BLE 斷線只在 BLE 控制模式下觸發停止；不會因背景手機事件中斷 Orin 控制。

切回 Orin：

```bash
python orin_client.py stop
python orin_client.py mode ORIN
```

不會自動接管或自動啟動。BLE 初始化/廣播設定使用既有函式庫，可能等待回應，僅在開機或已鎖定時執行。
模組未回覆停用廣播時會記錄 `DISABLE_UNCONFIRMED_CONTROL_BLOCKED`；即使仍可被掃描，BLE 行車控制也被模式閘門禁止。

## 訊息及故障排查

Serial1 和 USB 都輸出 JSON Lines。USB 可用 `?` 查狀態、`!` 立即停止；移除原本無保護的解鎖與極性切換字串。
詳細封包規格見 `PROTOCOL.md`。Orin 端 JSONL 每筆加入本機日期時間，韌體時間是開機後毫秒數。

| reason | 說明 |
|---|---|
| PHYSICAL_START_REQUIRED / NOT_ARMED | 先重新按實體啟動，再 ARM |
| NEUTRAL_REQUIRED | 第一筆 MOVE 必須是 0,0 |
| STALE_GENERATION | 停止/啟動後授權已變；不能沿用先前命令 |
| STALE_SEQUENCE | 重複或較舊封包；不更新逾時計時 |
| COMMAND_TIMEOUT | 500 ms 沒收到接受的行車指令，已鎖定 |
| START_AUTH_EXPIRED | READY 等待 30 秒未啟用，重新按啟動 |
| ESTOP_ACTIVE | D11 為 LOW，禁止啟用 |
| STOP_FIRST | 先 STOP，再切模式/同步會話 |
| PCA_INIT_FAILED / PCA_IO_FAILED / PCA_WRITE_FAILED | PCA9685 初始化/通訊故障，鎖定 FAULT，檢查後重開控制板 |

故障停止維持最後轉向設定，不突然自動回正；重新啟用的中立 MOVE 會回中心。
`max_loop_us` 是觀察到的最大迴圈間隔（包括開機後第一輪/模式設定時間），不是硬即時反應保證。
`tx_dropped` 記錄 Orin 輸出佇列滿時丟棄的訊息數；大量查詢/指令可能使 log 丟失。
USB log 可以獨立丟棄，不會阻塞 Orin。
I²C 交易 timeout 設 3 ms，PCA 位址約每秒探測一次；不代表所有硬體故障都可被辨識。

## 程式分工與測試

- `Config.h`：接線、校正、限幅、逾時。
- `Safety`：啟動/急停、控制權、generation 與行車租期。
- `Drive`：馬達斜坡、先停再換向、舵機及 PCA 故障。
- `OrinProtocol`：CRC、序號、非阻塞封包解析。
- `BleRemote`：手機備援；行車期直接非阻塞解析既有模組事件格式。
- `Encoder`：四倍頻計數及未觀察訊號判定。
- `Telemetry`：事件、ACK、狀態及非阻塞輸出。
- `tools/orin_client.py`：Orin 測試、監看、記錄工具。
- `tests`：使用實際韌體模組及 GPIO/UART/I²C 替身的桌面測試。

桌面測試涵蓋鎖定、實體啟動、中立首包、25% 限幅、逾時、急停不能被持續啟動解除、舊授權/舊序號、
反轉斜坡、CRC/超長/不完整封包、BLE 控制權與斷線、編碼器有效及非法轉換、輸出壅塞、I²C 故障停止與 JSON 完整性。
沒有測試實際馬達、手機 App、編碼器接線或硬體斷電。

Windows 工作區重跑桌面測試（需 g++、Python）：

```powershell
g++ -std=c++11 -Wall -Wextra -Wformat=2 -I Orin_Interface_v1/tests/stubs -I Orin_Interface_v1 Orin_Interface_v1/tests/firmware_test.cpp Orin_Interface_v1/Drive.cpp Orin_Interface_v1/MotorTest.cpp Orin_Interface_v1/Safety.cpp Orin_Interface_v1/Encoder.cpp Orin_Interface_v1/BleRemote.cpp Orin_Interface_v1/OrinProtocol.cpp Orin_Interface_v1/Telemetry.cpp -o Orin_Interface_v1/tests/firmware_test.exe
python -m unittest discover -s Orin_Interface_v1/tests -v
```
