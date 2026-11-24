step02 c1in38（同輪群組 A）：curve 1 entry: brake cap 32 -> 38 + bridge zone C4e+9.4..12.0 PWM 38
累積：以 step02 之前的結果為底（left34 + c1in38）

brake_zones.json  新 79bd0e2f  退回（前一步）3b857976  base 8ff9c0ea
lap_speed.json  新 43c3c216  退回（前一步）44872cbf  base 97ae42e1

改動：
  lap_speed.json zones[+curve 1 entry bridge]: None -> C4e+9.4..12.0 pwm 38 allow_low err 0.75 steer 400
  brake_zones.json zones[curve 1 after the slope].cap_pwm: 32 -> 38

驗證（相對前一步，只動列出的鍵）：
  lap_speed.json：改變 9 個鍵；不在清單內 0 個
      /note_speedplan_c1in38 : (無) -> "speedplan-20261122 step c1in38: curve 1 entry: brake cap 32 -> 38 + b
      /zones[curve 1 entry bridge 38 (spe]/allow_low : (無) -> true
      /zones[curve 1 entry bridge 38 (spe]/anchor : (無) -> "C4e"
      /zones[curve 1 entry bridge 38 (spe]/err_guard : (無) -> 0.75
      /zones[curve 1 entry bridge 38 (spe]/from_m : (無) -> 9.4
      /zones[curve 1 entry bridge 38 (spe]/name : (無) -> "curve 1 entry bridge 38 (speedplan-20261122 c1in38: between the curve
      /zones[curve 1 entry bridge 38 (spe]/pwm : (無) -> 38
      /zones[curve 1 entry bridge 38 (spe]/steer_guard_us : (無) -> 400.0
      /zones[curve 1 entry bridge 38 (spe]/to_m : (無) -> 12.0
  brake_zones.json：改變 2 個鍵；不在清單內 0 個
      /note_speedplan_c1in38 : (無) -> "speedplan-20261122 step c1in38: curve 1 entry: brake cap 32 -> 38 + b
      /zones[curve 1 after the slope (FLO]/cap_pwm : 32 -> 38
