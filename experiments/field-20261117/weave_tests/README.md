# 直線晃動：分辨原因用的三個單一變更（已備妥，未上車）
依據 straight-weave-20261117。以快照 orin-web-snapshot-20261117d 為底，每組只改一個鍵；每輪開跑讀、不用重啟。一次只換一組，換下一組前先放回原檔。
先以現況跑 3～5 圈當基準（11-17 的 18 圈中 10 圈晃）。看主直線 C4e+3～8 m 的車頭轉速標準差：平穩 <10、晃 >15。
1. 1_yaw_k4/yaw_inner.json：k_us_per_dps 6→4。驗證內迴圈是不是主迴路。預期晃的圈明顯變少；注意彎4 出彎與坡上阻尼變弱。
2. 2_play40/play_comp.json：±55→±40。驗證「補償只比間隙小一點」是不是點火源。看擺幅，不只看換向次數；直線固定偏差可能回來（11-13 補償不足時擺幅變大）。
3. 3_oppose_off/yaw_inner.json：oppose_cap_us 30→null。影響可能小；風險是 11-10 那種內迴圈項把車推向隧道。排最後、只看直線。
退回＝放回快照 d 的 yaw_inner.json／play_comp.json。

4. 4_no_weave_mode/yaw_inner.json：map_done_unhold_margin_deg 0.0→null、weave_unhold_dps 20→null ＝關掉「彎道已放手／偵測到擺動時改用全額陀螺儀阻尼（no_hold）」模式。
   依據（11-18）：主直線 18 圈中，平穩 6 圈整段是 straight／IDLE 且 no_hold 幾乎為 0；晃的 10 圈有 7 圈整段仍是 curve4／RELEASED（彎4 未結束）且 no_hold 100%。
   因果未定（晃動本身會讓彎4 不結束、也會觸發 no_hold）；這一組就是用來分辨。這個模式是 11-10 在每秒 10 張時加的。
   預期：若它是維持晃動的原因，直線上晃的圈會明顯變少、且會自己衰減；風險：出彎的大擺失去這個阻尼而變大。
