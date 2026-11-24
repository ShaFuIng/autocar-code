landing4 工具（全部只在筆電上跑，只讀 experiments/ 下的五輪 pid.jsonl 與 orin-web-snapshot-20261121c；不連車）
======================================================================================================
環境變數（都有預設值）：
  L4_EXP      experiments 資料夾（預設 D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments）
  L4_CACHE    小型快取（每輪一個 pickle，去掉遮罩；預設 <系統暫存>\l4）。跑完可整個刪掉。
  L4_SCRATCH  重播用的工作資料夾與輸出（預設 <系統暫存>\l4）；L4_OUT 可另指定輸出資料夾。

順序：
  1. python mk.py                      讀五輪 pid.jsonl 一次，寫快取。
  2. python inv.py     > ../proof/A_inventory.txt        A. 各路段由哪一層決定舵量／油門、交接位置、交接處舵量不連續
     python mouth.py   > ../proof/A_mouth.txt            坡口單格誤判、坡長、落地是否頂右限
     python tab1.py; python tab2.py  >> ../proof/A_heading.txt   23 次落地的絕對航向表（對主對話數字的重算）、走線分組
     python ident.py   > ../proof/I_steering_ident.txt   落地區／坡末／彎 4 的轉向增益與延遲（線性擬合）
  3. python mktab.py   > ../proof/T_tables.txt           由走線好的落地統計 theta*(u)、kappa*(u)、e_ref(u) → tables.json
     python mkcfg.py                                     由 tables.json＋快照的 arc2.json／lap_ff.json 產生 ../on/ 各步驟設定檔
  4. 重播（harness/；來源＝ramp-bias-20261119/harness，改動見 harness/common.py 與 replay.py 末段註解「landing4」）：
     python harness/run_replay.py --tag off --old base --new ../candidate/live_pid.py
     python harness/run_replay.py --tag step1_slope --new ../candidate/live_pid.py --cfg ../on/step1_slope      （其餘步驟同理）
     python harness/cmp_off.py <輸出>/off fidelity   > ../proof/C1_key_absent.txt
     python eval_open.py <輸出>                      > ../proof/C2_open_loop.txt
  5. python sim.py ../on/step2_slope_land/land4.json 200  > ../proof/C3_sim.txt      簡化閉迴路模型（限制寫在檔頭）
  6. python -B ../tests/test_land4.py                     單元測試

重播的輸入與限制：
  - 每輪的車上設定＝快照 20261121c＋當輪不同的兩檔（imu_boost.json 的 flat_pitch_deg、lap_speed.json；見 run_replay.py RUN_CFG）。
  - 五輪中只有 65ea 有 controls.jsonl，所以控制板狀態一律由 pid.jsonl 的 proposal.inputs（veh_ms、enc_b、steer_cmd）合成（SynthVehicle；
    右輪編碼器沒有紀錄，以左輪鏡像代替）。IMU 沒有 50 Hz 紀錄檔，用逐格紀錄重建（RecImu：上層用 heading_hold.yaw_deg、落地後用
    lap_ff4.band 的 ref−turned、中間以 gz 積分；俯仰＝dash_route.pitch_rel_deg＋該輪的平地基準）。
  - 因此「重播對車上紀錄」不是逐格相同（見 C1 的 fidelity 行）；鍵缺證明是「舊程式對新程式、同一份輸入」的差分比較。
  - 上層段落的重播與車上紀錄差異較大（上層單線保持依賴的邊線量測沒有重算），不影響 land4：它的輸入是 IMU、編碼器與相機誤差。
