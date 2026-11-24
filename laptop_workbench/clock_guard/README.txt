Orin 開機校時修正（2026-10-18）

根因：本次開機 timesyncd 在 10.894 秒恢復保存時間；RTC 驅動在
10.909 秒將系統設回 1970，導致校園 EAP 憑證尚未生效。

已完成：筆電與 Orin 各 6 項測試通過；bash -n、systemd-analyze verify
通過（系統既有 snapd/nvs/nvargus 單元警告不是本次新增）。
部署包：/home/jetson/clock-guard-20261018
目前待 sudo 安裝，不宣稱已完成冷開機驗收。

在 Orin 終端機執行：
sudo bash /home/jetson/clock-guard-20261018/install.sh

安裝以目前 NTP 已同步為前提，保存可信時間；啟用獨立開機恢復服務和
每分鐘檢查計時器。不重啟 NetworkManager、原車控、影子服務或整機。
正常日期不倒退；RTC 初始化等待有 25 秒上限，遲到由後續計時器補救。
Wi-Fi 僅於修復後未同步、既有介面 disconnected 且自動連線已啟用時，
最多重試既有校園設定檔三次，每次間隔至少兩分鐘。不變更憑證驗證。

安裝後檢查：
systemctl status orin-clock-maintain.timer --no-pager
cat /var/lib/orin-clock-guard/status.json
timedatectl status

停用修正（保留資料及備份）：
sudo bash /home/jetson/clock-guard-20261018/disable.sh

限制：保存時間只是可信下限，不是離線精準時鐘。長期斷電或憑證更新後，
舊下限仍可能不足；屆時可經 USB 從可信筆電校時，或驗證 RTC 備援供電。
此次不重開機，實際冷開機自動校時和 Wi-Fi 恢復須在日後授權時驗收。
全程沒有 UART、馬達、舵機、韌體、模型部署或原車控重啟操作。
