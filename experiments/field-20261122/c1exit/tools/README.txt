c1exit 工具（全部只在筆電上跑；只讀 experiments/ 下的 run-*-20261122/pid.jsonl 與 orin-web-snapshot-20261122；不連車）
環境變數：C1X_EXP＝experiments 資料夾（有預設）；C1X_CACHE＝每輪一個小 pickle 的快取（預設 系統暫存/c1exit，跑完可整個刪）；
          重播另用 L4_SCRATCH（工作資料夾與輸出，預設 系統暫存/c1exit）、L4_OUT。

資料（lib.py）：MAIN＝e0cd 6f61 c6f3；MORE＝4bbe 1eab ccf1 d60e（彎 1 設定相同）；SHORT＝14:12／14:16 兩批短起跑 11 輪（只用在第 1 圈）。

順序（在 tools/ 下執行）：
  python mk.py [runs]                               讀 pid.jsonl 一次，寫快取（去掉遮罩）
  python exits.py C1e > ../proof/A1_c1_exits.txt     逐次出彎表（C2e 同理 → A3）
  python hyp.py      > ../proof/A2_c1_groups_hypotheses.txt   分組、相關、四個假設的數字
  python profile.py C1e|C2e > ../proof/A4／A5        依「自直線起轉過的角度」分段的曲率／舵量／誤差／位置
  python trace.py <run> <lap> C1e|C2e [from] [to]    逐格表（B1／B2 由它產生）
  python ident.py; python ident_straight.py          轉向增益／延遲／間隙、相機誤差模型（→ I1；寫 c1err.json）
  python mkcfg.py [--lap-ff <車上 lap_ff.json>] [--i-ramp <車上 i_ramp.json>] [--out <資料夾>]   產生 on/ 全部步驟檔，只改列出的鍵（→ C0）
  python lap1_open.py > ../proof/C5_lap1_open_loop.txt   第 1 圈彎 1：以候選 lap_ff.py 對紀錄做模組層開迴路
  python sim.py base [N] / python sim.py <lap_ff.json> [N]   簡化閉迴路模型——未通過驗證（proof/D1），本次不用來預測
重播（harness/＝c3c4 重播工具複本；基底＝快照 20261122＋各輪當時的程式／設定，見 harness/run_replay.py 的 RUN_CFG）：
  新端資料夾＝ base/live_pid.car_3b74ddc4.py（改名 live_pid.py）＋candidate/lap_ff.py 放同一資料夾
  python harness/run_replay.py --tag off --old base --new <新端>/live_pid.py --runs c6f3,6f61,e0cd,1eab,4bbe
  python harness/cmp_off.py <輸出>/off fidelity > ../proof/C1_key_absent.txt
  python harness/run_replay.py --tag c1s1 --new base --cfg ../on/c1_step1_band --runs ...            （車上現行程式＋步驟檔）
  python harness/run_replay.py --tag c1s2 --new <新端>/live_pid.py --cfg ../on/c1_step2_exit_hold --runs ...
  （c2s1＝c2_step1_tail10、c2s2＝c2_step2_exit_hold、c2i＝i_ramp_c2e，各跑 c6f3,6f61）
  python eval_open.py <輸出> c1 c1s1 …  /  c2 c2s1 …  > ../proof/C2_*、C4_*
  python show_pass.py <輸出> <tag> <run> <lap> C1e|C2e   單趟逐格（車上送的 對 步驟會送的）
測試：python -B ../tests/test_c1exit.py
限制：
  - 重播是開迴路（同一份紀錄的相機／陀螺儀／編碼器輸入；車對新舵量的反應沒有模型）。
  - 重播的 IMU 絕對航向由逐格紀錄重建，第一次上層之前沒有參考：每輪第 1 圈的彎 1／彎 2 帶狀在重播裡角度是錯的（新舊兩邊相同，不影響鍵缺證明），
    第 1 圈彎 1 改用 lap1_open.py。
  - e0cd／1eab／4bbe 當時車上的 lap_ff.json 彎 2／彎 4 部分比現在舊；步驟檔以現行 052be3dc 為底，所以這三輪在彎 1 以外也會不同，「窗外 0 格」只對 c6f3／6f61 有意義。
