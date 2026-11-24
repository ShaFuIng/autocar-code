step03 crest33（同輪群組 A）：crest zone RAMP+2.3..4.4 PWM 27 -> 33
累積：以 step03 之前的結果為底（left34 + c1in38 + crest33）

lap_speed.json  新 05a06e7b  退回（前一步）43c3c216  base 97ae42e1

改動：
  lap_speed.json zones[crest (RAMP].pwm: 27 -> 33

驗證（相對前一步，只動列出的鍵）：
  lap_speed.json：改變 2 個鍵；不在清單內 0 個
      /note_speedplan_crest33 : (無) -> "speedplan-20261122 step crest33: crest zone RAMP+2.3..4.4 PWM 27 -> 3
      /zones[crest (RAMP+2.3 .. +4.4; cre]/pwm : 27 -> 33
