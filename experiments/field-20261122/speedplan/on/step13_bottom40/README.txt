step13 bottom40（同輪群組 F）：slope bottom -> curve 4 brake cap 36 -> 40
累積：以 step13 之前的結果為底（left34 + c1in38 + crest33 + c4_64 + post46 + left38 + c1in42 + crest36 + c4_70 + min28 + rate120 + c2br33 + bottom40）

brake_zones.json  新 e44626e7  退回（前一步）1b638bde  base 8ff9c0ea

改動：
  brake_zones.json zones[slope bottom].cap_pwm: 36 -> 40

驗證（相對前一步，只動列出的鍵）：
  brake_zones.json：改變 2 個鍵；不在清單內 0 個
      /note_speedplan_bottom40 : (無) -> "speedplan-20261122 step bottom40: slope bottom -> curve 4 brake cap 3
      /zones[slope bottom -> curve 4: kee]/cap_pwm : 36 -> 40
