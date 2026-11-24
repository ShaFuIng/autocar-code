step10 min28（同輪群組 D）：speed_adapt min_pwm 24 -> 28 (whole lap: LOW frames, hold frames)
累積：以 step10 之前的結果為底（left34 + c1in38 + crest33 + c4_64 + post46 + left38 + c1in42 + crest36 + c4_70 + min28）

speed_adapt.json  新 40af85cc  退回（前一步）578ac6d9  base 578ac6d9

改動：
  speed_adapt.json min_pwm: 24 -> 28

驗證（相對前一步，只動列出的鍵）：
  speed_adapt.json：改變 2 個鍵；不在清單內 0 個
      /min_pwm : 24 -> 28
      /note_speedplan_min28 : (無) -> "speedplan-20261122 step min28: speed_adapt min_pwm 24 -> 28 (whole la
