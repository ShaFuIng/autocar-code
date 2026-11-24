step01 left34（同輪群組 A）：left bend -> curve 3 entry: new lap_speed zone C2e+6.6..10.5 PWM 34 + curve-3 entry brake cap 30 -> 34
累積：以 step01 之前的結果為底（left34）

brake_zones.json  新 3b857976  退回（前一步）8ff9c0ea  base 8ff9c0ea
lap_speed.json  新 44872cbf  退回（前一步）97ae42e1  base 97ae42e1

改動：
  lap_speed.json zones[+left bend -> curve 3 entry]: None -> C2e+6.6..10.5 pwm 34 allow_low err 0.8 steer 350
  brake_zones.json zones[curve 3 entry].cap_pwm: 30 -> 34

驗證（相對前一步，只動列出的鍵）：
  lap_speed.json：改變 9 個鍵；不在清單內 0 個
      /note_speedplan_left34 : (無) -> "speedplan-20261122 step left34: left bend -> curve 3 entry: new lap_s
      /zones[left bend -> curve 3 entry 3]/allow_low : (無) -> true
      /zones[left bend -> curve 3 entry 3]/anchor : (無) -> "C2e"
      /zones[left bend -> curve 3 entry 3]/err_guard : (無) -> 0.8
      /zones[left bend -> curve 3 entry 3]/from_m : (無) -> 6.6
      /zones[left bend -> curve 3 entry 3]/name : (無) -> "left bend -> curve 3 entry 34 (speedplan-20261122 left34: the left be
      /zones[left bend -> curve 3 entry 3]/pwm : (無) -> 34
      /zones[left bend -> curve 3 entry 3]/steer_guard_us : (無) -> 350.0
      /zones[left bend -> curve 3 entry 3]/to_m : (無) -> 10.5
  brake_zones.json：改變 2 個鍵；不在清單內 0 個
      /note_speedplan_left34 : (無) -> "speedplan-20261122 step left34: left bend -> curve 3 entry: new lap_s
      /zones[curve 3 entry (11-19 field, ]/cap_pwm : 30 -> 34
