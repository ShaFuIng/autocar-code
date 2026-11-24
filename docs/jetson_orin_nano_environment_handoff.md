# Jetson Orin Nano 環境與連線資訊

## 1. 設備資訊

- 裝置：NVIDIA Jetson Orin Nano Developer Kit
- 記憶體：8 GB LPDDR5
- 架構：ARM64 / aarch64
- 儲存裝置：Crucial P310 PCIe Gen4 NVMe M.2 SSD
- 無線網路：Wi-Fi / Bluetooth 模組已安裝
- 顯示輸出：DisplayPort
- 主要用途：自走車上的影像辨識 / Edge AI 開發
- 攝影機：Full HD USB Webcam

## 2. 作業系統與 NVIDIA 環境

- Ubuntu：22.04
- Jetson Linux / L4T：R36.4.3
- 對應 JetPack 世代：6.2 系列
- CUDA：12.6
- cuDNN：9.3.0
- TensorRT：10.3.0
- OpenCV：4.8.0
- PyTorch：尚未安裝
- GPU 狀態：`tegrastats` 可正常使用，GPU 可正常運作

### 已確認的版本資訊

```text
Jetson Linux / L4T:
# R36 (release), REVISION: 4.3

CUDA:
Cuda compilation tools, release 12.6, V12.6.68

TensorRT:
10.3.0

OpenCV:
4.8.0
```

### JetPack 套件狀態

`nvidia-jetpack` meta package 本身顯示未安裝：

```text
Installed: (none)
Candidate: 6.2.1+b38
```

但 CUDA、cuDNN、TensorRT 等 JetPack 主要元件已個別存在，因此目前不需因為 meta package 顯示 `(none)` 而重灌系統。

## 3. 主機名稱與網路

- Linux 使用者：`jetson`
- Hostname：`orinnano-super`
- mDNS 名稱：`orinnano-super.local`

Jetson 使用 DHCP，因此實際 IPv4 會隨 Wi-Fi 網路改變，例如曾出現：

```text
192.168.0.245
10.39.224.240
```

同一區網下建議直接使用：

```text
orinnano-super.local
```

避免依賴會變動的 DHCP IP。

## 4. SSH 遠端連線

SSH Server 已安裝並啟用，開機會自動啟動。

Windows / Linux 端連線方式：

```bash
ssh jetson@orinnano-super.local
```

SSH 預設 Port：

```text
22
```

若 `.local` 無法解析，可先在 Jetson 查 IP：

```bash
hostname -I
```

再改用：

```bash
ssh jetson@<Jetson-IP>
```

## 5. TigerVNC 遠端 GUI

目前使用 TigerVNC 建立獨立的虛擬桌面，而不是鏡像實體 DisplayPort 畫面。

- VNC Display：`:2`
- VNC Port：`5902`
- Systemd Service：`tigervncserver@:2.service`
- 使用者對應：`:2=jetson`

Windows TigerVNC Viewer 連線：

```text
orinnano-super.local:2
```

或：

```text
orinnano-super.local:5902
```

### 檢查 VNC 狀態

```bash
systemctl status tigervncserver@:2.service --no-pager
```

確認 Port：

```bash
ss -ltnp | grep 5902
```

若 VNC 未啟動：

```bash
sudo systemctl start tigervncserver@:2.service
```

確認是否設為開機自啟：

```bash
systemctl is-enabled tigervncserver@:2.service
```

## 6. Headless 使用方式

目前目標是讓 Jetson 作為 Headless 主機使用：

```text
Jetson Orin Nano
├─ 電源
├─ Wi-Fi
├─ Full HD Webcam
└─ 不需接實體螢幕 / 鍵盤 / 滑鼠
```

主要遠端操作方式：

```text
SSH
→ CLI、安裝套件、執行程式、系統管理

VS Code Remote SSH
→ 程式開發

TigerVNC :2
→ 需要 Ubuntu GUI 時使用
```

TigerVNC 的 `:2` 是獨立虛擬桌面，因此與實體 DP 螢幕上的桌面不是同一個 session。

## 7. 瀏覽器

- Chromium：Ubuntu Snap 版本
- Snap Chromium 在 TigerVNC 虛擬桌面中曾出現 cgroup / snap session 問題
- 目前改用 Brave ARM64 版本
- Brave 已可在 TigerVNC 虛擬桌面中正常開啟

## 8. 時間設定

- Time Zone：`Asia/Taipei`
- NTP：啟用
- 系統時間同步：正常

可用以下指令確認：

```bash
timedatectl
```

正常狀態：

```text
System clock synchronized: yes
NTP service: active
Time zone: Asia/Taipei
```

## 9. 常用檢查指令

```bash
# Jetson Linux / L4T
cat /etc/nv_tegra_release

# CPU 架構
uname -m

# CUDA
nvcc --version

# OpenCV
python3 -c "import cv2; print(cv2.__version__)"

# TensorRT
python3 -c "import tensorrt as trt; print(trt.__version__)"

# GPU / RAM / 功耗 / 溫度
tegrastats

# IP
hostname -I

# SSH
systemctl status ssh --no-pager

# VNC
systemctl status tigervncserver@:2.service --no-pager
ss -ltnp | grep 5902

# 時間同步
timedatectl
```

## 10. 目前不需處理的項目

- 不需重灌 Ubuntu
- 不需重灌 Jetson Linux
- 不需重新安裝 CUDA / cuDNN / TensorRT
- PyTorch 尚未安裝，等老師指定模型或框架後再決定版本
- 影像辨識模型尚未安裝，待確認老師指定的模型與框架
