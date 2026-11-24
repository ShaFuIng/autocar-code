step07 c1in42（同輪群組 C）：curve 1 entry second step: cap 42, zone start 0.4 m later, bridge 42
累積：以 step07 之前的結果為底（left34 + c1in38 + crest33 + c4_64 + post46 + left38 + c1in42）

brake_zones.json  新 4db1d51e  退回（前一步）616615fb  base 8ff9c0ea
lap_speed.json  新 0a77e5fd  退回（前一步）9f338686  base 97ae42e1

改動：
  lap_speed.json zones[+curve 1 entry bridge]: None -> C4e+9.4..12.0 pwm 42 allow_low err 0.75 steer 400
  brake_zones.json zones[curve 1 after the slope].cap_pwm: 38 -> 42
  brake_zones.json zones[curve 1 after the slope].lap_from_m: 19.2 -> 19.6
  brake_zones.json zones[curve 1 after the slope].from_m: 16.8 -> 17.2

驗證（相對前一步，只動列出的鍵）：
  lap_speed.json：改變 17 個鍵；不在清單內 0 個
      /note_speedplan_c1in42 : (無) -> "speedplan-20261122 step c1in42: curve 1 entry second step: cap 42, zo
      /zones[curve 1 entry bridge 38 (spe]/allow_low : true -> (無)
      /zones[curve 1 entry bridge 38 (spe]/anchor : "C4e" -> (無)
      /zones[curve 1 entry bridge 38 (spe]/err_guard : 0.75 -> (無)
      /zones[curve 1 entry bridge 38 (spe]/from_m : 9.4 -> (無)
      /zones[curve 1 entry bridge 38 (spe]/name : "curve 1 entry bridge 38 (speedplan-20261122 c1in38: between the curve -> (無)
      /zones[curve 1 entry bridge 38 (spe]/pwm : 38 -> (無)
      /zones[curve 1 entry bridge 38 (spe]/steer_guard_us : 400.0 -> (無)
      /zones[curve 1 entry bridge 38 (spe]/to_m : 12.0 -> (無)
      /zones[curve 1 entry bridge 42 (spe]/allow_low : (無) -> true
      /zones[curve 1 entry bridge 42 (spe]/anchor : (無) -> "C4e"
      /zones[curve 1 entry bridge 42 (spe]/err_guard : (無) -> 0.75
      /zones[curve 1 entry bridge 42 (spe]/from_m : (無) -> 9.4
      /zones[curve 1 entry bridge 42 (spe]/name : (無) -> "curve 1 entry bridge 42 (speedplan-20261122 c1in42: between the curve
      /zones[curve 1 entry bridge 42 (spe]/pwm : (無) -> 42
      /zones[curve 1 entry bridge 42 (spe]/steer_guard_us : (無) -> 400.0
      /zones[curve 1 entry bridge 42 (spe]/to_m : (無) -> 12.0
  brake_zones.json：改變 4 個鍵；不在清單內 0 個
      /note_speedplan_c1in42 : (無) -> "speedplan-20261122 step c1in42: curve 1 entry second step: cap 42, zo
      /zones[curve 1 after the slope (FLO]/cap_pwm : 38 -> 42
      /zones[curve 1 after the slope (FLO]/from_m : 16.8 -> 17.2
      /zones[curve 1 after the slope (FLO]/lap_from_m : 19.2 -> 19.6
