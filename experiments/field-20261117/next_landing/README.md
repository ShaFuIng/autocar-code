# 落地後油門（已備妥，尚未上車）
依據 run-4a76：落地後 0～1.4 s 油門 24（landing_pwm），之後在 curve_map 還沒認出彎4 之前（LAND8+約1.9～2.9 m）speed_adapt 走 LOW→最低油門 21，車速谷底 0.73～0.75 m/s；約 3.0 m 後才 38。
改法（只換兩個設定檔，每輪開跑讀，不用重啟）：
- speed_adapt.json：landing_pwm 24→30、landing_s 1.5→3.0（落地後 3 s 內路徑類低檔都用 30，蓋掉那段 21）。
- brake_zones.json：坡底上限區（上限 28）結束點 DOWN+4.8→3.8 m（約 LAND8+1.6 m）。
結果：落地後約 1.6 m 內最多 28，之後 30，curve_map 認出彎4 後 38。
上車前：車上兩檔須等於 orin-web-snapshot-20261117d 的版本（brake_zones ac3f12cf、speed_adapt 91f4b01c），否則只套用上述鍵。
看：落地後 1.5～3 m 車速不再掉到 0.75 以下；扭動有沒有超出白線；彎4 中段有無丟線。退回＝放回原兩檔。
