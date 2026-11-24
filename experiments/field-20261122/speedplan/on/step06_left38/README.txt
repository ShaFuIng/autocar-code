step06 left38（同輪群組 C）：left bend second step: zone 38, curve-3 entry cap 38
累積：以 step06 之前的結果為底（left34 + c1in38 + crest33 + c4_64 + post46 + left38）

brake_zones.json  新 616615fb  退回（前一步）79bd0e2f  base 8ff9c0ea
lap_speed.json  新 9f338686  退回（前一步）dc11718c  base 97ae42e1

改動：
  lap_speed.json zones[+left bend -> curve 3 entry]: None -> C2e+6.6..10.5 pwm 38 allow_low err 0.8 steer 350
  brake_zones.json zones[curve 3 entry].cap_pwm: 34 -> 38

驗證（相對前一步，只動列出的鍵）：
  lap_speed.json：改變 3 個鍵；不在清單內 0 個
      /note_speedplan_left38 : (無) -> "speedplan-20261122 step left38: left bend second step: zone 38, curve
      /zones[left bend -> curve 3 entry 3]/name : "left bend -> curve 3 entry 34 (speedplan-20261122 left34: the left be -> "left bend -> curve 3 entry 38 (speedplan-20261122 left38: the left be
      /zones[left bend -> curve 3 entry 3]/pwm : 34 -> 38
  brake_zones.json：改變 2 個鍵；不在清單內 0 個
      /note_speedplan_left38 : (無) -> "speedplan-20261122 step left38: left bend second step: zone 38, curve
      /zones[curve 3 entry (11-19 field, ]/cap_pwm : 34 -> 38
