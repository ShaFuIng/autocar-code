step11 rate120（同輪群組 E）：lap_speed global rate_pwm_s 60 -> 120 (whole lap; also the ramp 100 zone)
累積：以 step11 之前的結果為底（left34 + c1in38 + crest33 + c4_64 + post46 + left38 + c1in42 + crest36 + c4_70 + min28 + rate120）

lap_speed.json  新 14b63be6  退回（前一步）c5633601  base 97ae42e1

改動：
  lap_speed.json rate_pwm_s: 60.0 -> 120.0

驗證（相對前一步，只動列出的鍵）：
  lap_speed.json：改變 2 個鍵；不在清單內 0 個
      /note_speedplan_rate120 : (無) -> "speedplan-20261122 step rate120: lap_speed global rate_pwm_s 60 -> 12
      /rate_pwm_s : 60.0 -> 120.0
