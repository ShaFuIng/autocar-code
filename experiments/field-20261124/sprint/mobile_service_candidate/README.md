# 手機操作服務 mobile-service-r6（2026-11-23，離線完成、未上車；Orin 上為 r4／r5，見 AGENTS.md）

**r6 新增：起點位置**（車比旗子位置往後幾公尺；邏輯在 `start_cfg.py`，純標準庫）。頁面「起點位置」填實際量到的距離（0～7 m、0.1 m 一格）→「套用」→ 只改車控每輪開跑時讀的四個檔的規則鍵（免重啟）：`lap_pos.json` start_s＝−D、skip_after_by.C1s＝9.0；`brake_zones.json` 所有 anchor start 且 from_m<900 的區 from_m／to_m＝旗子值＋Δ；`lap_ff.json` curves.c1.start.gap_m＝旗子值＋Δ；`start_bias.json` enabled＝（往後 ≤2.5 m）。D＝3.44＋往後距離（>0 時再少報 0.25），Δ＝D−3.44。其餘鍵與各檔原排版（縮排、換行 CRLF/LF、單行）逐位元保留，來回切換回到原值＝逐位元相同。
- 目前往後幾公尺：由 `lap_pos.json` start_s 反推（0.1 m 格點上一對一）。旗子參考值（各起跑區 from/to 依區名、c1 gap_m）存在本服務目錄 `start_ref.json`，**不寫進車控的檔**。四檔與參考對不起來＝「不一致」：套用被拒（START_INCONSISTENT／START_NO_REF／START_S_UNKNOWN），頁面列出每一處差異，並提供「以目前車上檔重建參考」（＝認定車上檔正確，明確按鈕，不自動）。只有 C1s 跳過距離、起步左舵不合規則時＝START_NEEDS_APPLY，再按一次套用即補正。不一致**不擋起跑**（只在起跑鍵上方紅字警示）。
- 安全：行駛／暫停中拒絕（RUN_ACTIVE）；問不到車控時拒絕（CAR_UNREACHABLE）；比賽起跑模式開啟中拒絕（RACE_ARMED）；四檔先算好並自我檢查（重新解析、只改規則鍵、套用後讀得回同一距離）才寫；寫前四原檔＋參考檔備份到 `~/orin_interface_v1/backups/startcfg_<時間>/`；全部先寫 .tmp（fsync）再逐一 os.replace，任一步失敗或讀回不符就放回原檔（START_WRITE_FAILED）。值不變時不寫、不備份。
- 介面：GET `/api/start`（只讀本機檔）；`/api/pid/status` 多 `start` 欄位 {back_m, D, delta_m, left_bias_on, consistent, reason, problems, fixable}；POST `/api/start/set {"back_m": 2.5}` 回改了哪些鍵與備份資料夾；POST `/api/start/rebuild-ref {}`。
- Orin 終端機：`python3 start_cfg.py status`、`python3 start_cfg.py plan 2.5`（只顯示、不寫）、`python3 start_cfg.py rebuild-ref`（套用只走頁面，因為要問車控有沒有輪次）。
- 上車＝複製 server.py、start_cfg.py、web/index.html、web/model.mjs、web/style.css、web/start.js，重啟 orin-mobile（不動車控），再按一次「以目前車上檔重建參考」（或終端機 rebuild-ref）。測試：`python -m unittest test_start test_server test_level`（test_start 需要筆電上的 experiments 資料夾，否則略過）。

# 手機操作服務 mobile-service-r3（2026-11-21）

**r3 新增：圈數賽起跑模式**（依根目錄 `CAR-INTEGRATION.md`）。頁面「開啟起跑模式（等綠燈）」→ 手機服務在 Orin 上每 50 ms 讀中控台 `http://192.168.0.149:8080/go`（timeout 0.2 s；只在開啟時讀）→ 旗標先 0 後 1 才起跑（開啟時已是 1 要先歸零）→ 走與「從起點開始」相同的起跑內容 → 起跑模式自動關閉、不再讀旗標。旗標回 0 **不停車**（使用者決定，用暫停／停止本輪）。開啟條件：模型已載入且看得到路徑、無故障（含 IMU、急停）、無輪次。綠燈時再檢查一次故障；「沒有連續 3 張有效結果」在 1 秒內重試，其他拒絕不重試；起跑回覆遺失不重送（START_UNKNOWN）。開啟中拒絕兩個修復鍵；按停止本輪或手動起跑會取消起跑模式。服務重啟＝起跑模式關閉。介面 POST `/api/race/arm`、`/api/race/disarm`，狀態多 `race` 欄位（旗標、連線、讀取 ms、起跑結果、綠燈→車控接受 ms）。`service.json` 可選鍵：`race_go_url`（Pi 換 IP 時改）、`race_practice`（true＝加 ?practice=1，燈號練習也會起跑，正式比賽不可開）。上車＝換 server.py、web/index.html、web/model.mjs、web/race.js、web/style.css，重啟 orin-mobile（不動車控）。本機示範：`py -3.12 race_demo.py`（假車控＋假旗標，頁面 127.0.0.1:18092；不上 Orin）。測試 20 項。



手機頁（Codex 的 `mobile_console` 前端，去掉模擬內容）＋轉接到真車車控（Orin 8000）。跑在 Orin 上，與筆電工作台分開。

- 網址：`http://<Orin 位址>:8092`（2026-11-21 場地：192.168.0.244）。**目前免登入**（使用者 2026-11-21 決定；`service.json` 為 `{"login": false}`）：同一網路上知道網址的人都能操作。要恢復登入＝刪掉 Orin 上的 `service.json` 或改成 `{"login": true}`，再 `systemctl --user restart orin-mobile`；登入與原車控網站相同（jetson＋同一組登入碼）。連線紀錄在 `access.log`（不含密碼）。
- Orin 位置：`~/orin_interface_v1/mobile_service/`；使用者服務 `orin-mobile.service`（目前**未設開機自啟**）。
  - 啟動／停止：`systemctl --user start|stop orin-mobile`；開機自啟：`systemctl --user enable orin-mobile`。
- 頁面功能：載入跑車版本、從起點開始（無倒數）、暫停／繼續、停止本輪、黑白遮罩＋路徑、行駛時間與圈時。
- 只轉接 7 個介面：status、mask、prepare、start、stop、pause、resume；舵機、馬達測試、手動駕駛、卸載模型都碰不到。
- 起跑內容固定在 `start_params.json`（PID 文件 rev154 的 31 個參數、orin_timed、seconds null）；中心與左右極限在起跑時改用車上舵機保存設定。**改了 PID 參數要重新產生這個檔**（工作台 PID 文件 → `start.parameters`、`parameter_revision`）。
- 狀態與修復（r2）：服務每 0.5 秒讀 IMU 最新值；時間戳在走但陀螺儀／俯仰／側傾 2 秒不變＝凍結、1.5 秒沒有新資料＝沒有資料。IMU 異常超過 1 秒時狀態 fault 為 IMU_FAULT（頁面不能起跑）。頁面按鈕「重啟 IMU」（POST /api/repair/imu＝重啟 orin-imu-logger）、「重新載入模型」（POST /api/repair/model＝卸載再載入）；有輪次在跑（含暫停）時兩者都拒絕。自動：異常持續 3 秒、沒有輪次在跑、距上次自動重啟 30 秒以上時自動重啟 IMU 紀錄服務（`service.json` 的 `imu_auto_restart` 設 false 可關）。車控本身不能從頁面重啟。
- 測試：`python -m unittest test_server`（假車控，r3 共 20 項）。

未驗證：Android 實機畫面與操作、seconds null（無期限）起跑、手機起跑後的整圈。車控端暫停接續的驗證狀況見專案根目錄 AGENTS.md。
