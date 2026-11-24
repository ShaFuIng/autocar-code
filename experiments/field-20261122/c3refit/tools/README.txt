c3refit 工具（全部只在筆電上跑；只讀 experiments/ 下的 run-*-20261122/pid.jsonl、orin-web-snapshot-20261122、field-20261122/c3c4；不連車）
環境變數：C3R_SCRATCH＝快取資料夾（每輪一個小 pickle；預設 系統暫存/c3refit，跑完可整個刪）；重播另用 L4_SCRATCH（工作資料夾與輸出）。
只用這五輪：6f61（x-band r1 只記錄）、c6f3（x-band r1 全權）、d60e／84f2（無 x-band、無減 30）、ccf1（減 30）。

在 tools/ 下執行（順序）：
  python p3.py                      讀五輪一次、寫快取（p3.passes 提供每趟彎 3 的陣列；xc＝連續距離、x＝控制器看到的階梯距離、h＝自 C3s 那一步起的陀螺儀航向）
  python d0.py 6f61 c6f3 d60e     > ../proof/A0_profiles_by_distance.txt     依距離的剖面（含 x-band 各項）
  python d1.py                    > ../proof/A1_alignment_handback_tail.txt  逐趟：車道特徵在控制器距離上的位置、交還與尾段狀態
  python ident.py ; python ident2.py   > ../proof/A3_model_identification.txt   車道航向 T(x)、幾何一致性、轉向增益、相機模型 → model.json（ident2 要接在 ident 後面）
  python d2.py                    > ../proof/A2_staircase_timing_spike_tail.txt   編碼器階梯、轉彎重心、3.0～3.6 m 尖峰組成、arc_lat 量程、尾段
  python chk_turned.py            > ../proof/A4_heading_reference.txt        三種航向基準的散布
  python chk_al.py                > ../proof/A5_arc_lat_position.txt         arc_lat 橫向位置的可用比例與雜訊
  python calib.py <車上 arc2.json> ; python sweep.py <同>   > ../proof/D1_model_calibration.txt   簡化模型對 6f61／c6f3 的校正
  python plan.py                  > ../proof/D2_plan_and_tables.txt          平順路徑與各表
  python mkcfg.py                                                           由車上 arc2.json 2b81e9b2 產生 ../on/ 各步驟
  python evalm.py <車上 arc2.json> 名稱=<arc2.json>…   > ../proof/D3_model_runs.txt   模型中的擾動比較（鏈、車上現行表、新表）
  python chk_main.py              > ../proof/A6_main_session_numbers_recomputed.txt   以主對話的切法（階梯距離、0.6 m 一格）重算
  python open_offline.py ../on/step1_own_conservative/arc2.json ../on/step2_own_standard/arc2.json ../on/step3_own_prerotated/arc2.json   > ../proof/C2b_open_loop_on_car_logs.txt
                                                                            以 candidate/lap_ff.py 的 _xb_tick／_xb2 直接跑車上紀錄（彎 3 的開迴路數字以這份為準）
  （sweep.py＝定模型轉向延遲時的掃描，結果附在 D1 末尾；gains.py／dbg.py＝選增益與查大舵量來源時用的小工具）
重播（harness/＝c3c4 的重播工具複本；多註冊 6f61、c6f3，基底＝快照 20261122＋當晚車上的程式與設定，見 harness/run_replay.py 的 RUN_CFG；base/lap_ff.car_052be3dc.json＝當晚車上的 lap_ff.json）：
  python harness/run_replay.py --tag off --old base --new ../candidate/live_pid.py --runs 6f61,c6f3,d60e_new
  python harness/cmp_off.py <輸出>/off fidelity                              > ../proof/C1_key_absent.txt
  python harness/run_replay.py --tag <步驟> --new ../candidate/live_pid.py --cfg ../on/<步驟> --runs 6f61,c6f3,d60e_new      （每個步驟一次）
  python eval_open.py <輸出> step0_shadow step1_own_conservative step2_own_standard   > ../proof/C2_open_loop.txt
測試：python -B ../tests/test_c3refit.py                                    > ../proof/C3_tests.txt
重播限制：重播工具重建的 IMU 航向在部分彎 3 通過是錯的（見 proof/C2_open_loop.txt 末尾），彎 3 帶狀的數值請看 C2b；
其餘同前：控制板狀態與 IMU 由逐格紀錄重建，「重播對車上紀錄」不是逐格相同（C1 的 fidelity 行）；鍵缺證明是「車上程式對新程式、同一份輸入、同一份設定」的差分比較。
