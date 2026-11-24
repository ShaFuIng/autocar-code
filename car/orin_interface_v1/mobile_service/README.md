# 手機操作服務 mobile-service-r2（2026-11-21）

手機頁（Codex 的 `mobile_console` 前端，去掉模擬內容）＋轉接到真車車控（Orin 8000）。跑在 Orin 上，與筆電工作台分開。

- 網址：`http://<Orin 位址>:8092`（2026-11-21 場地：192.168.0.244）。**目前免登入**（使用者 2026-11-21 決定；`service.json` 為 `{"login": false}`）：同一網路上知道網址的人都能操作。要恢復登入＝刪掉 Orin 上的 `service.json` 或改成 `{"login": true}`，再 `systemctl --user restart orin-mobile`；登入與原車控網站相同（jetson＋同一組登入碼）。連線紀錄在 `access.log`（不含密碼）。
- Orin 位置：`~/orin_interface_v1/mobile_service/`；使用者服務 `orin-mobile.service`（目前**未設開機自啟**）。
  - 啟動／停止：`systemctl --user start|stop orin-mobile`；開機自啟：`systemctl --user enable orin-mobile`。
- 頁面功能：載入跑車版本、從起點開始（無倒數）、暫停／繼續、停止本輪、黑白遮罩＋路徑、行駛時間與圈時。
- 只轉接 7 個介面：status、mask、prepare、start、stop、pause、resume；舵機、馬達測試、手動駕駛、卸載模型都碰不到。
- 起跑內容固定在 `start_params.json`（PID 文件 rev154 的 31 個參數、orin_timed、seconds null）；中心與左右極限在起跑時改用車上舵機保存設定。**改了 PID 參數要重新產生這個檔**（工作台 PID 文件 → `start.parameters`、`parameter_revision`）。
- 狀態與修復（r2）：服務每 0.5 秒讀 IMU 最新值；時間戳在走但陀螺儀／俯仰／側傾 2 秒不變＝凍結、1.5 秒沒有新資料＝沒有資料。IMU 異常超過 1 秒時狀態 fault 為 IMU_FAULT（頁面不能起跑）。頁面按鈕「重啟 IMU」（POST /api/repair/imu＝重啟 orin-imu-logger）、「重新載入模型」（POST /api/repair/model＝卸載再載入）；有輪次在跑（含暫停）時兩者都拒絕。自動：異常持續 3 秒、沒有輪次在跑、距上次自動重啟 30 秒以上時自動重啟 IMU 紀錄服務（`service.json` 的 `imu_auto_restart` 設 false 可關）。車控本身不能從頁面重啟。
- 測試：`python -m unittest test_server`（假車控，8 項）。

未驗證：Android 實機畫面與操作、seconds null（無期限）起跑、手機起跑後的整圈。車控端暫停接續的驗證狀況見專案根目錄 AGENTS.md。
