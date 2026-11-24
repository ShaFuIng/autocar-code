# Orin GPU Notebook 與操作台

這份環境搭配本專案的只讀網頁操作台。Notebook 獨立在 JetPack 6 GPU 容器內，車輛服務在 Orin 主機上執行，兩者透過 HTTP API 交換快照與狀態。

## 已完成驗證

- PyTorch 2.10.0、CUDA 12.6、torchvision 0.25.0、Ultralytics 8.4.174、TensorRT 10.7.0、JupyterLab 4.4.9（容器內版本）。主機既有 Python／TensorRT 環境沒有因此更新。
- CUDA 矩陣計算與 torchvision GPU NMS 通過。
- Notebook 已執行兩個程式碼儲存格，沒有 error 輸出，成功顯示 GPU 資訊、車況與相機快照。
- 操作台未登入回傳 401；Jupyter 未登入導向 login，有 token 可讀 API。Jupyter CSP 僅允許指定操作台來源嵌入；筆電至 8000 與 8888 連線均通過。
- YOLO26n 對相機預覽快照（960×540）以 imgsz=640 做兩次暖機、三次 GPU 預測，平均 34.98 ms。這是 PyTorch 預測耗時，排除取像與網路傳輸，不是 TensorRT 測試、跑道辨識準確率或完整行車 FPS。
- 容器以非 root 使用者執行，非 privileged，設定 unless-stopped；操作台使用者服務已啟用 linger。尚未以重新開機驗證開機流程。
- 本地七項測試通過，包括登入拒絕、只讀端點、Notebook 程式碼格式與原有 client／韌體測試。

## 登入

- 操作台：`http://10.130.227.240:8000`，帳號 `jetson`。
- JupyterLab：操作台的 Notebook 頁面，或 `http://10.130.227.240:8888/lab`。
- 在你自己的 Orin 終端執行 `cat ~/.config/orin_lab/token` 取得私人登入憑證。操作台把它當密碼；Jupyter 登入表單可輸入同一 token。
- token 位於 Orin 使用者的私人設定資料夾，不複製到 Windows／OneDrive，不加入版本管理。HTTP 未加密，僅按使用者授權開放指定介面，沒有設定外網轉發。換網路後需重新確認 IP 與可存取範圍。

## 功能與限制

操作台是第一版只讀監看，沒有 ARM、MOVE 或其他行車指令端點。相機由主機單一服務以 720p MJPEG 取像，網頁預覽是 960×540、約 5 FPS。UART 保持單次連線，開啟異常與穩定期異常分開計數，異常原始 hex 寫入 `~/orin_interface_v1/logs/web_uart.jsonl`，限制每檔 2MB、保留三份輪替。

Notebook 透過已驗證的 API 讀影像與狀態，不映射 `/dev/ttyTHS1`、相機裝置、Docker socket 或 SSH 私鑰。它可以使用 GPU，但 Notebook 大型訓練仍會和推論競爭 GPU 資源，因此不與正式行車並行做大量訓練。

## 檔案與版本

- Orin：`~/orin_interface_v1/lab/notebooks` 持久保存 Notebook，即使重建容器也保留。
- Windows 原始範例：`D:\OneDrive\Desktop\自走車\Orin_Interface_v1\lab\notebooks`。
- `provision.py` 用 Docker 拉取後的 RepoDigest 固定基底映像，寫入 `.env`，不以浮動 latest 作為後续重建依據。
- `requirements-installed.txt` 保存實際套件版本；`gpu_validation.txt` 保存 GPU 驗證結果。
- `01_GPU與相機驗證.ipynb` 驗證 CUDA 矩陣計算、讀取車況與顯示相機快照。執行成功會生成 `01_GPU與相機驗證_已執行.ipynb`。

## 管理（在 Orin 終端）

```sh
systemctl --user status orin-dashboard
systemctl --user restart orin-dashboard
systemctl --user stop orin-dashboard
cd ~/orin_interface_v1/lab
docker compose ps
docker compose stop
docker compose up -d
```

UART 被操作台獨占；要執行舊的 orin_client.py 工具，先停止操作台，結束後再啟動。操作台停止／重啟本身不發行車命令。

已為使用者服務啟用 linger，登入工作階段結束後仍可執行。Notebook 容器使用 `restart: unless-stopped`。目前服務綁定固定 IP；IP 改變時要更新 web 的 systemd Environment、lab `.env` 的 ORIN_HOST／BIND_ADDR，以及 Jupyter iframe 允許來源後重啟。

## 重建

在 Orin 的 lab 目錄以 `docker compose build`、`docker compose up -d` 重建。初次基底下載需要數 GB，空間與時間需求明顯；不需要重裝主機 JetPack。容器 TensorRT／PyTorch 版本與主機可能不同，TensorRT engine 應在目標環境建置並驗證。
