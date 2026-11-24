"""Local validation/report/bundle only. No SSH, serial, Docker, or deployment."""
import hashlib
import json
from pathlib import Path
import sys
import unittest
import zipfile
from datetime import datetime,timezone
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'web'),str(ROOT/'tests'),str(ROOT/'laptop_workbench')]
from calibrated_path import CalibratedPath,load_calibration,validate_row
from visual_pid import PID,DEFAULTS
import test_pid_stream_safety as stream

FILES=['web/calibrated_path.py','web/calibration_v7.json','web/pid_paths.py',
       'web/live_pid.py','web/live_model_worker.py','web/visual_pid.py',
       'laptop_workbench/temporal_lane.py','laptop_workbench/curve_replay.py',
       'laptop_workbench/ground_calibration.py']

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    cv2.setNumThreads(2)
    out=ROOT/'laptop_workbench/data/experiments/v7-pid-integration'
    out.mkdir(parents=True,exist_ok=True)
    suite=unittest.TestSuite()
    for module in ('test_calibrated_pid','test_calibrated_stream','test_live_pid',
                   'test_pid_path_worker','test_pid_stream_safety',
                   'test_ground_calibration','test_ground_paths'):
        suite.addTests(unittest.defaultTestLoader.loadTestsFromName(module))
    stream.EVIDENCE.clear()
    result=unittest.TextTestRunner(verbosity=1).run(suite)
    calibration,_,digest=load_calibration()
    snapshot=ROOT/'laptop_workbench/data/experiments/camera-measure'/calibration['source_snapshot']['id']
    assert sha(snapshot/'raw.jpg')==calibration['source_snapshot']['image_sha256']
    image=cv2.imdecode(np.fromfile(str(snapshot/'raw.jpg'),np.uint8),1)
    original_mask=cv2.imdecode(np.fromfile(str(snapshot/'mask.png'),np.uint8),-1)[:,:,3]>0
    observations=[]
    for mode in ('double','left','right'):
        mask=original_mask.copy()
        if mode=='left':mask[:,320:]=False
        if mode=='right':mask[:,:320]=False
        tracker=CalibratedPath()
        for i in range(5):r=tracker.step(image,mask,1+i*.15)
        pid=PID(dict(DEFAULTS,center_us=1360.,row_ratio=.6))
        pid.step_path(1.45,r['path'])
        proposal=pid.step_path(1.6,r['path'])
        overlay=image.copy()
        points=np.asarray(r['path'])/2
        if len(points):
            cv2.polylines(overlay,[points.round().astype(np.int32)],False,(0,255,0),2)
        cv2.line(overlay,(0,int(.6*719/2)),(639,int(.6*719/2)),(0,220,255),1)
        cv2.imencode('.png',overlay)[1].tofile(str(out/(mode+'.png')))
        observations.append(dict(mode=mode,result=r,proposal=proposal,image=mode+'.png'))
    original=ROOT/'laptop_workbench/data/experiments/ground-calibration/measurement-v7/result.json'
    assert sha(original)==digest
    sources={name:sha(ROOT/name) for name in FILES}
    manifest=dict(format=1,algorithm='calibrated-v7',files=sources,
                  calibration_sha256=digest,remote_root='/home/jetson/orin_interface_v1',
                  built_at=datetime.now(timezone.utc).isoformat(),
                  automatic_restart=False,automatic_start=False)
    bundle=out/'v7-pid-candidate.zip'
    # Never deliver a package labelled ready if tests fail.
    if result.wasSuccessful():
        with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED) as z:
            for name in FILES:z.write(ROOT/name,name)
            z.writestr('manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2))
        with zipfile.ZipFile(bundle) as z:
            assert all(hashlib.sha256(z.read(n)).hexdigest()==h for n,h in sources.items())
    report=dict(title='校正 v7 → 路徑算法 → 視覺 PID',built_at=manifest['built_at'],
        passed=result.wasSuccessful(),tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
        calibration_sha256=digest,source_snapshot=calibration['source_snapshot']['id'],
        sources={**sources,'tests/test_calibrated_pid.py':sha(ROOT/'tests/test_calibrated_pid.py'),
                 'tests/test_calibrated_stream.py':sha(ROOT/'tests/test_calibrated_stream.py')},
        snapshot_observations=observations,
        stream_cases=[c for c in stream.EVIDENCE if c['algorithm']=='calibrated-v7'],
        range_cm=[50,120],recommended_rows={'near':.6,'far':.5,
            'near_reference_cm':validate_row(.6),'far_reference_cm':validate_row(.5)},
        deployed=False,hardware_output=False,
        bundle=dict(path=str(bundle.relative_to(ROOT)),sha256=sha(bundle)) if result.wasSuccessful() else None,
        completed=[
            '固定v7校正原檔與SHA256；V4遮罩內緣投影至公分座標。',
            '雙線取中心；單線以46cm淨寬的一半向內法線偏移；僅用本影格證據。',
            '路徑轉回1280×720影像座標，接入既有視覺PID；模式連續3影格確認，缺線與範圍不足即失效。',
            '真實worker協定、LivePID串流與假控制端測試；候選支援start而非僅preview，仍需校正SHA與相機姿態確認。',
            '04依Orin實際能力決定是否可選用控制；未部署的舊Orin不會因筆電UI更新而解鎖。',
            '已產生可核對雜湊的部署ZIP，沒有連Orin、重啟服務或發出車控指令。'],
        limitations=[
            '目前只有v7原始單張快照。重複輸入用於模式確認測試，不是現場連續影片驗收；沒有將舊姿態影片套用v7。',
            '100cm獨立檢查仍約+3.6cm；單線左右路徑差與投影寬度不一致仍存在，未宣稱公分級精確標定。',
            '二次曲線只擬合50～120cm實際觀測區間；擬合殘差RMS超過2cm或偏移折返就拒絕，急彎可能無可用路徑。',
            '相機實際姿態無法由影像尺寸自動證明；部署後須確認姿態、16:9無裁切及新鮮影格。',
            '尚未量測Orin端到端延遲，未完成實車閉環；本次中心1360µs，PID增益與PWM只是軟體測試參數，不是試跑建議。',
            '原始校正檔的未確認欄位保留，未自動改成已確認；實體確認仍依使用者現場判斷。'],
        deployment_steps=[
            '重新連線後先唯讀核對無進行中的任務／錄影、目前服務與版本；不自動停止既有任務。',
            '核對ZIP與manifest雜湊，保存遠端所有待覆寫檔案及不存在檔案清單，再將套件放到暫存位置並編譯檢查。',
            '部署變更需另行安排車控重啟（可能回正）；取得授權後才切換檔案與重啟，失敗使用備份回退。',
            '重啟後唯讀確認車況、校正SHA與算法能力；確認姿態後以v7、近列0.60載入預覽並檢查新鮮結果。',
            '現場另行授權指定時長／PWM後才能從04開始本輪PID；不自動試跑。'])
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'],tests=report['tests'],
        v7_stream_cases=len(report['stream_cases']),
        snapshot_modes=[(x['mode'],x['result']['valid'],x['proposal']['valid']) for x in observations])))
    return 0 if result.wasSuccessful() else 1

if __name__=='__main__':raise SystemExit(main())
