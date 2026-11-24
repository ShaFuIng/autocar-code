step12 c2br33（同輪群組 F）：curve 2 entry: both curve-2 brake caps 30 -> 33 + bridge zone C1e+10.6..12.8 PWM 33
累積：以 step12 之前的結果為底（left34 + c1in38 + crest33 + c4_64 + post46 + left38 + c1in42 + crest36 + c4_70 + min28 + rate120 + c2br33）

brake_zones.json  新 1b638bde  退回（前一步）4db1d51e  base 8ff9c0ea
lap_speed.json  新 c60cc821  退回（前一步）14b63be6  base 97ae42e1

改動：
  lap_speed.json zones[+curve 2 entry bridge]: None -> C1e+10.6..12.8 pwm 33 allow_low err 0.75 steer 400
  brake_zones.json zones[curve 2 entry].cap_pwm: 30 -> 33
  brake_zones.json zones[curve 2 on lap 1].cap_pwm: 30 -> 33

驗證（相對前一步，只動列出的鍵）：
  lap_speed.json：改變 9 個鍵；不在清單內 0 個
      /note_speedplan_c2br33 : (無) -> "speedplan-20261122 step c2br33: curve 2 entry: both curve-2 brake cap
      /zones[curve 2 entry bridge 33 (spe]/allow_low : (無) -> true
      /zones[curve 2 entry bridge 33 (spe]/anchor : (無) -> "C1e"
      /zones[curve 2 entry bridge 33 (spe]/err_guard : (無) -> 0.75
      /zones[curve 2 entry bridge 33 (spe]/from_m : (無) -> 10.6
      /zones[curve 2 entry bridge 33 (spe]/name : (無) -> "curve 2 entry bridge 33 (speedplan-20261122 c2br33: C2s-0.4 ran speed
      /zones[curve 2 entry bridge 33 (spe]/pwm : (無) -> 33
      /zones[curve 2 entry bridge 33 (spe]/steer_guard_us : (無) -> 400.0
      /zones[curve 2 entry bridge 33 (spe]/to_m : (無) -> 12.8
  brake_zones.json：改變 3 個鍵；不在清單內 0 個
      /note_speedplan_c2br33 : (無) -> "speedplan-20261122 step c2br33: curve 2 entry: both curve-2 brake cap
      /zones[curve 2 entry (11-16 field):]/cap_pwm : 30 -> 33
      /zones[curve 2 on lap 1 (FLOOR 17.7]/cap_pwm : 30 -> 33
