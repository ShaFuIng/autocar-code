c3c4 工具（全部只在筆電上跑；只讀 experiments/ 下的 run-*-20261122/pid.jsonl 與 orin-web-snapshot-20261122；不連車）
環境變數：C34_EXP＝experiments 資料夾（有預設）；C34_CACHE＝每輪一個小 pickle 的快取（預設 系統暫存/c3c4，跑完可整個刪）；
          重播另用 L4_SCRATCH（工作資料夾與輸出，預設 系統暫存/c3c4）。

順序（在 tools/ 下執行）：
  python mk.py                                  讀五輪 pid.jsonl 一次，寫快取
  python c4tab.py    > ../proof/A1_c4_profiles.txt        彎 4 沿距離／已轉角度的弧度、誤差、帶狀各項
  python c4track.py  > ../proof/A2_c4_track_shape.txt     彎 4 車道航向（陀螺儀＋arc_lat 相對航向）
  python c4err.py    > ../proof/A3_c4_error_model.txt     彎 4 相機誤差＝f(已轉角度)＋b×橫向＋c×相對航向 → c4err.json
  python c4hand.py   > ../proof/A4_c4_handover.txt        land4→帶狀交接時的狀態；最後寫入者比例
  python ident.py    > ../proof/A5_steering_gain.txt      轉向增益（模型用）
  python c3tab.py base > ../proof/B1_c3_profiles.txt （另 c3tab.py trim）；c3track.py → B2；c3err.py → B4（寫 c3err.json）
  python mkcfg.py                               由快照的 json 產生 ../on/ 全部設定檔（印出雜湊與相對基底差異的鍵）
  python plan_c3.py  > ../proof/D2_c3_own_plan_and_model.txt   彎 3 距離帶狀的路徑規劃、表、簡化閉迴路模型
  python sim_c4.py <lap_ff.json>…               彎 4 帶狀簡化閉迴路模型（D1）
  python sim_c3.py <curve_trim.json> <arc2.json> 彎 3「只換設定」版的擾動模型（D3）
重播（harness/＝landing4 的重播工具複本；基底改為快照 20261122，每輪設定見 harness/run_replay.py 的 RUN_CFG：d60e／ccf1／4bbe）：
  新端資料夾＝candidate/ 的 live_pid.py＋lap_ff.py（放在同一資料夾，重播工具會把同資料夾的 .py 蓋到基底上）
  python harness/run_replay.py --tag off --old base --new <新端>/live_pid.py --runs d60e,ccf1,4bbe
  python harness/cmp_off.py <輸出>/off fidelity > ../proof/C1_key_absent.txt
  python harness/run_replay.py --tag <步驟> --new <新端>/live_pid.py --cfg ../on/<步驟> --runs d60e,ccf1      （每個步驟一次）
  python eval_open.py <輸出> <步驟>… > ../proof/C2_open_loop.txt
測試：python -B ../tests/test_c3c4.py
重播限制同 landing4：控制板狀態與 IMU 由逐格紀錄重建，「重播對車上紀錄」不是逐格相同（C1 的 fidelity 行）；鍵缺證明是「舊程式對新程式、同一份輸入」的差分比較。
