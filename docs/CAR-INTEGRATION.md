# 圈數賽起跑旗標：車端接入說明（給各隊與各隊的 AI agent）

這份文件說明怎麼讓你的自走車在圈數賽綠燈亮起的那一刻自動起跑。整套只需要在你原本的程式前面加「等旗標變 1」這一段，不用裝任何套件，Python、C++、Node 或 shell 都可以。文件末尾有給 agent 的任務描述與驗收標準。

## 1. 訊號是什麼

中控台是一台 Raspberry Pi 5（主機名 `pi5`，Wi-Fi `R18-AIRACE`，IP `192.168.0.149`，HTTP 埠 8080）。裁判在控制台按「開始比賽」後：

| 時間 | 中控台 | 旗標 `/go` 的值 |
| --- | --- | --- |
| 按下開始 | 拱門燈板亮第 1 顆紅燈 | `0` |
| 第 1、2 秒 | 第 2、3 顆紅燈 | `0` |
| 第 3 秒 | 三盞燈熄滅＝綠燈，比賽開始 | **`1`** |
| 比賽中 | 計圈、排名 | `1` |
| 裁判按停止或重置 | 燈板停住或回 READY | `0` |

旗標是一個純文字網址，任何裝置都能讀：

```
GET http://192.168.0.149:8080/go        → 回 "1" 或 "0"
GET http://192.168.0.149:8080/go?practice=1   → 燈號練習的綠燈也算 1（測試用）
GET http://192.168.0.149:8080/sign      → 三盞燈的網頁動畫，可用手機開著看
GET http://192.168.0.149:8080/api/state → JSON，含 status（idle/countdown/running/finished）、go_ms（綠燈時間，毫秒）
```

只有圈數賽會變 1，計時賽沒有起跑燈。舊的 UDP 廣播（埠 37020）已經移除，不要再用。

## 2. 車端要做的事：一個 if

```python
from urllib.request import urlopen
if urlopen("http://192.168.0.149:8080/go", timeout=0.2).read() == b"1":
    開始()
```

```sh
[ "$(curl -s -m 0.2 http://192.168.0.149:8080/go)" = "1" ] && 開始
```

實際使用時把它包成「起跑模式」，規則如下，請照做，這些是實測踩過的坑：

1. **先把一切準備好再等旗標。** 鏡頭、模型、馬達、UART 都載入完成後才開始問旗標，綠燈一亮就能出發。不要綠燈後才載入模型。
2. **只在起跑線上等待時輪詢，每 50 毫秒一次，timeout 0.2 秒以內。** 起跑後就不要再問，網路卡住才不會卡住控制迴圈。
3. **只在旗標由 0 變 1 時起跑。** 開啟起跑模式時若旗標已經是 1（上一場還沒重置、或 Pi 重開機後殘留），不能起跑，要等它先歸零。
4. **一次性。** 起跑後自動關閉起跑模式，下一場再手動開。否則車停在場邊時，下一場綠燈會自己衝出去。
5. **旗標變回 0 要不要停車，自己決定。** 若要，起跑後繼續讀旗標，讀到 0 就停車。無論如何都要保留自己的手動與實體停止，旗標只是通知，不是保證。
6. **正式比賽不要用 `?practice=1`。** 它只是讓「燈號練習」也能觸發，方便測試；比賽時帶著它，練習燈號車就會跑出去。

## 3. 可直接使用的 Python 參考實作

存成 `race_go.py` 放在你的程式旁邊，只用標準函式庫，Python 3.6 以上都能跑。

```python
"""圈數賽起跑旗標：等中控台 /go 由 0 變 1。"""
import time
from urllib.request import urlopen

PI_GO_URL = "http://192.168.0.149:8080/go"


def read_go(url=PI_GO_URL, timeout=0.2):
    """回傳 1 或 0；讀不到（斷線、逾時）回 None。"""
    try:
        with urlopen(url, timeout=timeout) as r:
            return 1 if r.read(8).strip() == b"1" else 0
    except OSError:
        return None


def wait_for_go(url=PI_GO_URL, interval=0.05, on_status=None):
    """停在這裡直到旗標由 0 變 1（綠燈）。

    開始等待時旗標若已經是 1（上一場沒重置）不會回傳，等它歸零後的下一次綠燈。
    on_status(value, saw_zero) 每次讀取後呼叫，可拿來更新畫面；value 為 1、0 或 None。
    """
    saw_zero = False
    while True:
        value = read_go(url)
        if on_status:
            on_status(value, saw_zero)
        if value == 0:
            saw_zero = True
        elif value == 1 and saw_zero:
            return
        time.sleep(interval)


def stopped(url=PI_GO_URL):
    """起跑後想跟著裁判的停止鍵停車時用：旗標回 0 回傳 True。讀不到回 False（不當作停止）。"""
    return read_go(url) == 0
```

用法：

```python
from race_go import wait_for_go, stopped

載入鏡頭與模型()
連接馬達()
print("起跑模式：等待綠燈")
wait_for_go()                 # 綠燈那一刻回傳
開始行駛()
while 行駛中:
    跑一個控制週期()
    # 可選：每 0.2 秒檢查一次，裁判按停止就停車
    # if stopped(): 停車(); break
```

若你的車有網頁或按鈕介面，建議做成一個「起跑模式」開關，開啟時才執行 `wait_for_go()`（放在背景執行緒），並顯示：目前旗標值、中控台連不連得到、起跑時間。

## 4. 網路設定（Jetson 上）

- Jetson 連 Wi-Fi `R18-AIRACE`，和 Pi 同網段。確認：`curl -s -m 2 http://192.168.0.149:8080/go` 要回 `0` 或 `1`。
- 關掉 Wi-Fi 省電，否則回應偶爾會慢幾百毫秒（需要 sudo，重開機後要再做一次）：

  ```sh
  iw dev                                   # 查介面名稱，Orin Nano 常是 wlP1p1s0 或 wlan0
  sudo iw dev wlP1p1s0 set power_save off
  ```

- Pi 的 IP 目前由 DHCP 配發，比賽前裁判會固定；若找不到 Pi，先 `ping 192.168.0.149`。

## 5. 怎麼測試

**有中控台時**（最準）：手機開 `http://192.168.0.149:8080/control`，模式選圈數賽，車子開起跑模式後按「開始比賽」，3 秒後車應出發；按「停止」旗標回 0。第一次先把車架起來讓輪子懸空。可另開 `/sign` 看燈。計時器（BMduino）有連線時也可以用「燈號練習」的「播放起跑流程」，此時車端要暫時用 `?practice=1`。

**沒有中控台時**：在自己電腦上跑下面這個假旗標伺服器，車端網址改成 `http://<電腦IP>:8080/go`。

```python
# fake_go.py：python3 fake_go.py，然後 curl -X POST localhost:8080/start（3 秒後變 1）、curl -X POST localhost:8080/stop
import threading, time
from http.server import BaseHTTPRequestHandler, HTTPServer
go_at = [None]
class H(BaseHTTPRequestHandler):
    def _send(self, body):
        self.send_response(200); self.send_header("Content-Type", "text/plain"); self.end_headers(); self.wfile.write(body)
    def do_GET(self):
        self._send(b"1" if go_at[0] and time.time() >= go_at[0] else b"0")
    def do_POST(self):
        go_at[0] = time.time() + 3 if self.path == "/start" else None
        self._send(b"ok")
    def log_message(self, *a): pass
HTTPServer(("", 8080), H).serve_forever()
```

驗收時要看到：待命讀到 0；按 start 後 3 秒內讀到 1 並起跑；起跑後不再輪詢（或依你的設計在 stop 後停車）；假伺服器一開始就是 1 的情況下，開起跑模式不會起跑，stop 後再 start 才會。

## 6. 給 agent 的任務描述

> 在這台車的自走程式加入「起跑模式」：開啟後每 50 毫秒讀 `http://192.168.0.149:8080/go`（timeout 0.2 秒），旗標先為 0、之後變 1 時呼叫現有的起跑函式，然後自動關閉起跑模式；沒開啟時不碰網路。旗標讀不到時顯示離線但不影響其他功能。使用本文件第 3 節的 `race_go.py`，或等價實作。完成後用第 5 節的假伺服器跑過四個驗收情境，再連真的中控台測一次。不要移除或弱化車上原本的手動停止與緊急停止。

常見錯誤：綠燈後才載入模型（晚好幾秒才動）、起跑模式沒有自動關閉（下一場自己衝出去）、timeout 太長或在控制迴圈裡同步讀網路（Wi-Fi 一卡車就停頓）、比賽時留著 `?practice=1`。
