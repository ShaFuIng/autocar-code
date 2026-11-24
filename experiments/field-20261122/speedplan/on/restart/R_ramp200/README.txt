R_ramp200：(restart, touches the climb) ramp 100 zone own rate 200 PWM/s
底＝step14 累積結果的 lap_speed.json（c60cc821）。只在 lap_speed.py 7bbeb350（straightfast lap-speed-r3）上有效；舊程式 3ad2272b 會忽略新鍵（行為＝底）。
實際上車時請用 mkcfg.py --base <車上檔> --steps <已採用的步驟>,R_ramp200 重產。

  lap_speed.json：改變 2 個鍵；不在清單內 0 個
      /note_speedplan_R_ramp200 : (無) -> "speedplan-20261122 step R_ramp200: (restart, touches the climb) ramp 
      /zones[A ramp 100 (RAMP+0.3..1.9, a]/rate_pwm_s : (無) -> 200.0
