# 視覺循線自走車：程式、模型與環境

這個版本庫保存一台視覺循線自走車在比賽結束時（2026-11-24）的完整狀態，目的是日後能把車復原。
開發過程的說明與簡報在另一個版本庫：<https://github.com/ShaFuIng/autocar-devlog>。

車子的組成：Jetson Orin Nano 負責相機、白線辨識、路徑計算與控制迴圈；BMduino 控制板負責馬達與舵機；筆電工作台與手機網頁負責下指令和看結果。

## 內容

| 資料夾 | 內容 |
|---|---|
| `car/orin_interface_v1/web/` | 車控程式與所有行駛設定（比賽結束時車上的最終狀態） |
| `car/orin_interface_v1/mobile_service/` | 手機操作頁（起跑、暫停接續、起點位置、直線衝刺、燈號起跑）與測試 |
| `car/orin_interface_v1/lab/` | 辨識容器的 Dockerfile 與已安裝套件清單 |
| `car/orin_interface_v1/*.ino *.cpp *.h` | 控制板韌體原始碼 |
| `car/orin_interface_v1/camera imu tools` | 相機曝光設定、IMU 紀錄服務、工具腳本 |
| `car/config/` | 舵機保存設定、手動預設、中心估計 |
| `car/systemd/` | 開機自動啟動的使用者服務檔 |
| `model/` | 車上使用的白線辨識模型 `runs/trial01/weights/best.pt`、訓練參數與結果、訓練腳本 |
| `env/orin_environment.txt` | 系統版本、映像檔版本碼、容器內與主機的套件清單 |
| `env/orin_sha256_web_20261124.txt` | 車上檔案的雜湊，用來核對 `car/` 的內容 |
| `firmware/pwm100_v1` | 比賽時使用的韌體（原始碼與燒錄檔）；`pwm70_v1` 是前一版 |
| `laptop_workbench/` | 筆電工作台程式（不含資料與執行環境） |
| `laptop_src/` | 筆電端的測試與工具腳本 |
| `docs/` | 協作規則 `AGENTS.md`、逐日紀錄、跑道地圖、各階段的交接與需求文件 |
| `experiments/orin-web-snapshot-20261123d/` | 定版時的車上快照 |
| `experiments/field-*/` | 現場各日的分析腳本、設定候選與摘要 |

## 沒有放進來的

- 密碼與金鑰：車上的登入權杖、Wi-Fi 密碼、SSH 金鑰都不在這裡，復原時要重新設定。
- 辨識映像檔本體（約 15 GB）：只留定義與版本碼。基底映像當時用的是 `latest` 標籤，重建時請用 `env/orin_environment.txt` 記錄的版本碼。
- 原始行駛紀錄、影片與訓練資料集。
- 開發過程中的其他候選版本與歷史快照。

## 復原步驟

1. Orin 依 `env/orin_environment.txt` 的系統版本安裝，並依 `car/orin_interface_v1/lab/` 重建辨識映像。
2. 把 `car/orin_interface_v1/` 放到 `~/orin_interface_v1/`，把 `model/runs` 放到 `~/orin_interface_v1/lab/notebooks/yolo26_sem_v4_trial/runs`。
3. 把 `car/config/` 放到 `~/.config/orin_car/`，`car/systemd/` 放到 `~/.config/systemd/user/` 並啟用各服務。
4. 控制板燒錄 `firmware/pwm100_v1/build/` 的燒錄檔。
5. 重新設定登入權杖與 Wi-Fi。
6. 舵機或轉向連桿更換過的話，轉向中心與間隙補償要重新量，做法見 `docs/AGENTS.md`。

## 注意

- 這些設定是針對特定一台車、特定跑道與當時的舵機調出來的，換車或換場地不能直接套用。
- `docs/` 內的紀錄是開發當下寫的，保留了場地的網路名稱、區域網路位址與裝置主機名稱。
- 實車動作有風險：依 `docs/AGENTS.md` 的規則，馬達、舵機、燒錄與重啟車控都要在人在場時進行。
