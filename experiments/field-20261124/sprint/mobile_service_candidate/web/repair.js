// mobile-service-r2 (2026-11-21): 狀態與簡易修復 — IMU 健康、重啟 IMU、重新載入模型。與 app.js 分開，互不影響。
import {reasonText} from './model.mjs';
const $ = id => document.getElementById(id), sleep = ms => new Promise(r => setTimeout(r, ms));
const IMU_TEXT = {ok: '正常', frozen: '異常：讀數凍結', stale: '異常：沒有新資料', missing: '異常：讀不到資料', unknown: '檢查中'};
let busy = false, note = '', last = null;

async function call(path, body, timeout = 4000) {
  const c = new AbortController(), t = setTimeout(() => c.abort(), timeout);
  try {
    const r = await fetch(path, {cache: 'no-store', signal: c.signal, ...(body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Orin-Control': '1'}, body: JSON.stringify(body)})});
    const d = await r.json();
    if (!r.ok || d.ok === false) throw Error(reasonText(d.reason, d.reason_text));
    return d;
  } finally { clearTimeout(t); }
}

function render() {
  const s = last, imu = s?.imu, running = s?.run?.state === 'running';
  const ok = imu?.state === 'ok';
  $('imuState').textContent = !s ? '—' : (IMU_TEXT[imu?.state] || '未知');
  $('imuState').dataset.bad = String(!!s && !ok && imu?.state !== 'unknown');
  $('imuDetail').textContent = imu?.pitch_deg == null ? '' : `俯仰 ${imu.pitch_deg.toFixed(1)}°　側傾 ${imu.roll_deg.toFixed(1)}°` + (imu.auto ? `　上次自動重啟 ${imu.auto.at}${imu.auto.ok ? '' : '（未執行：' + (imu.auto.code === 'RUN_ACTIVE' ? '行駛中' : imu.auto.code) + '）'}` : '');
  const lv = imu?.level, still = !running && s?.pause?.state !== 'paused';
  $('imuLevel').textContent = !lv ? '—' : `${lv.calibrated_static_deg.toFixed(2)}° ／ ${imu.pitch_deg == null ? '—' : imu.pitch_deg.toFixed(2) + '°'}` + (lv.diff_deg == null ? '' : `（差 ${lv.diff_deg > 0 ? '+' : ''}${lv.diff_deg.toFixed(2)}°）`);
  const off = !!lv && lv.diff_deg != null && Math.abs(lv.diff_deg) >= 0.5;
  $('imuLevel').dataset.bad = String(off && still);
  $('imuLevelHint').textContent = !lv ? '' : !still ? '行駛中的數值不能拿來比較。' : off ? '車平放在跑道平地上靜止時，差距超過 0.5° 就建議按「平地校準 IMU」。' : '車平放在跑道平地上靜止時才有意義；差距在 0.5° 內不用校準。';
  $('modelState').textContent = !s ? '—' : ({ready: '已載入，看得到路徑', no_reliable_path: '已載入，看不到路徑', loading: '載入中', idle: '未載入', failed: '異常', camera_unavailable: '相機異常'}[s.model_state] || s.model_state || '未知');
  $('fixImu').disabled = busy || !s || running;
  $('fixModel').disabled = busy || !s || running;
  $('fixLevel').disabled = busy || !s || running || !ok;
  $('fixLevelDefault').disabled = busy || !s || running || s?.pause?.state === 'paused';
  $('repairNote').hidden = !note; $('repairNote').textContent = note;
  $('repairHint').textContent = running ? '行駛或暫停中不能修復，請先停止本輪。' : '重啟 IMU 或平地校準前，請讓車靜止不動約 3 秒；平地校準要把車平放在跑道平地上。';
}

async function poll() { try { last = await call('/api/pid/status'); } catch { last = null; } render(); }

async function fix(path, label, timeout, done) {
  if (busy) return;
  busy = true; note = `${label}中…`; render();
  try { const d = await call(path, {}, timeout); note = done ? done(d) : `${label}已送出，請等狀態恢復。`; }
  catch (e) { note = `${label}沒有執行：${e.name === 'AbortError' ? '車端沒有回覆' : e.message}`; }
  busy = false; await poll();
  setTimeout(() => { note = ''; render(); }, 8000);
}

$('fixImu').onclick = () => fix('/api/repair/imu', '重啟 IMU', 25000);
$('fixLevel').onclick = () => fix('/api/repair/imu-level', '平地校準', 10000, d => `平地校準完成：靜止俯仰 ${d.static_pitch_deg}°，平地基準 ${d.old_flat_pitch_deg} → ${d.flat_pitch_deg}。下一輪起跑生效。`);
$('fixLevelDefault').onclick = () => fix('/api/repair/imu-level-default', '平地基準放回', 6000, d => d.old_flat_pitch_deg === d.flat_pitch_deg ? `平地基準本來就是 ${d.flat_pitch_deg}，沒有變更。` : `平地基準 ${d.old_flat_pitch_deg} → ${d.flat_pitch_deg}。下一輪起跑生效。`);
$('fixModel').onclick = () => fix('/api/repair/model', '重新載入模型', 35000);
render();
(async () => { while (true) { await poll(); await sleep(document.hidden ? 3000 : 1000); } })();
