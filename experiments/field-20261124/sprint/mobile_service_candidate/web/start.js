// mobile-service-r6 (2026-11-23; r7 sprint lock 2026-11-24): 起點位置（車比旗子位置往後幾公尺）。改的是 Orin 上車控每輪開跑時讀的四個設定檔，邏輯在 start_cfg.py。
import {reasonText} from './model.mjs';
const $ = id => document.getElementById(id), sleep = ms => new Promise(r => setTimeout(r, ms));
const UNDER = 0.25, REF_D = 3.44, NEAR_MAX = 2.5;
const BLOCKING = ['START_INCONSISTENT', 'START_NO_REF', 'START_S_UNKNOWN', 'START_FILE_ERROR'];
let last = null, busy = false, note = '', noteBad = false, touched = false, extra = [];

async function call(path, body, timeout = 4000) {
  const c = new AbortController(), t = setTimeout(() => c.abort(), timeout);
  try {
    const r = await fetch(path, {cache: 'no-store', signal: c.signal, ...(body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Orin-Control': '1'}, body: JSON.stringify(body)})});
    const d = await r.json();
    if (!r.ok || d.ok === false) { const e = Error(reasonText(d.reason, d.reason_text)); e.problems = d.problems || []; throw e; }
    return d;
  } finally { clearTimeout(t); }
}

const dOf = b => Math.round((REF_D + b - (b > 0 ? UNDER : 0)) * 100) / 100;
const where = b => b === 0 ? '旗子位置 0 m' : `旗子往後 ${b.toFixed(1)} m`;
function parse(text) {   // 同 start_cfg.parse_back：數字、0～7、0.1 m 一格
  if (!/^\s*\d+(\.\d+)?\s*$/.test(text)) return {err: '請填 0～7 的數字'};
  const v = Number(text);
  if (!(v >= 0 && v <= 7)) return {err: '範圍 0～7 m'};
  if (Math.abs(v * 10 - Math.round(v * 10)) > 1e-6) return {err: '精度 0.1 m'};
  return {v: Math.round(v * 10) / 10};
}

function render() {
  const s = last?.start, run = last?.run?.state === 'running', paused = last?.pause?.state === 'paused', race = !!(last?.race?.armed || last?.race?.firing);
  const known = s && s.back_m !== null && s.back_m !== undefined;
  const now = !s ? '—' : known ? `${where(s.back_m)}（起步左舵 ${s.left_bias_on ? '開' : '關'}）` : '不明';
  $('startNow').textContent = now;
  $('startNow').dataset.bad = String(!!s && !s.consistent);
  const w = $('startWhere');
  w.textContent = !s ? '目前起點：讀取中' : `目前起點：${now}` + (s.consistent ? '' : s.reason === 'START_NEEDS_APPLY' ? '・要再按一次「套用」' : '・設定不一致，見下方「起點位置」');
  w.dataset.bad = String(!!s && !s.consistent);
  if (known && !touched && document.activeElement !== $('startBack')) $('startBack').value = s.back_m.toFixed(1);
  const p = parse($('startBack').value);
  const same = p.v !== undefined && known && s.consistent && p.v === s.back_m;
  $('startPreview').textContent = p.err ? p.err : `套用後：${where(p.v)}，內部距離 D ${dOf(p.v).toFixed(2)} m，起步左舵 ${p.v <= NEAR_MAX ? '開' : '關'}` + (same ? '（與目前相同）' : '');
  $('startPreview').dataset.bad = String(!!p.err);
  const blocked = !s || BLOCKING.includes(s.reason);
  const sprint = last?.sprint?.on === true;   // r7: the sprint zones were placed for the present start position
  $('startApply').disabled = busy || !last || run || paused || race || sprint || !!p.err || blocked || same;
  for (const id of ['startMinus', 'startPlus', 'startFlag']) $(id).disabled = busy;
  const probs = s && !s.consistent ? (s.problems || []) : [];
  $('startWarn').hidden = !s || s.consistent;
  if (s && !s.consistent) {
    const head = s.reason === 'START_NEEDS_APPLY' ? '起點設定有一項沒有照規則（例如 C1s 跳過距離、起步左舵），按「套用」會補正：' : reasonText(s.reason);
    $('startWarn').replaceChildren(document.createTextNode(head), ...(probs.length ? [list(probs)] : []));
  }
  const canRebuild = !!s && ['START_INCONSISTENT', 'START_NO_REF'].includes(s.reason);
  $('startRebuild').hidden = !canRebuild; $('startRebuild').disabled = busy || race || sprint;
  const canForce = !!s && ['START_INCONSISTENT', 'START_S_UNKNOWN'].includes(s.reason);   // r6a: rewrite every rule key from the reference
  $('startForce').hidden = !canForce; $('startForce').disabled = busy || race || run || paused || sprint || !!p.err;
  $('startForce').textContent = p.err ? '以參考值重寫四個檔（先填好上面的距離）' : `以參考值重寫成「${where(p.v)}」（上次寫入中斷、斷電後用）`;
  $('startNote').hidden = !note; $('startNote').dataset.bad = String(noteBad);
  $('startNote').replaceChildren(document.createTextNode(note), ...(extra.length ? [list(extra)] : []));
  $('startHint').textContent = sprint ? '直線衝刺模式開啟中不能改起點，請先關閉直線衝刺。' : race ? '比賽起跑模式開啟中不能改起點。' : run || paused ? '行駛或暫停中不能改起點，請先停止本輪。' : '只在停車時能改，下一輪起跑生效。';
}

function list(items) { const ul = document.createElement('ul'); ul.replaceChildren(...items.map(t => { const li = document.createElement('li'); li.textContent = t; return li; })); return ul; }
function say(text, bad = false, items = []) { note = text; noteBad = bad; extra = items; render(); if (!bad) setTimeout(() => { if (note === text) { note = ''; extra = []; render(); } }, 15000); }

async function poll() { try { last = await call('/api/pid/status'); } catch { last = null; } render(); }

function nudge(dv) {
  const p = parse($('startBack').value), base = p.v === undefined ? (last?.start?.back_m ?? 0) : p.v;
  $('startBack').value = Math.min(7, Math.max(0, Math.round((base + dv) * 10) / 10)).toFixed(1); touched = true; render();
}
$('startMinus').onclick = () => nudge(-0.5);
$('startPlus').onclick = () => nudge(0.5);
$('startFlag').onclick = () => { $('startBack').value = '0.0'; touched = true; render(); };
$('startBack').oninput = () => { touched = true; render(); };

$('startApply').onclick = async () => {
  const p = parse($('startBack').value); if (busy || p.err) return;
  if (!confirm(`把起點改成「${where(p.v)}」？\n起步左舵 ${p.v <= NEAR_MAX ? '開' : '關'}，下一輪起跑生效。\n車要放在這個位置起跑。`)) return;
  busy = true; say('套用中…');
  try {
    const d = await call('/api/start/set', {back_m: p.v}, 10000);
    touched = false;
    say(d.backup ? `已套用：${where(d.back_m)}（起步左舵 ${d.left_bias_on ? '開' : '關'}），改了 ${d.changed.length} 個值，原檔備份 ${d.backup}。` : `本來就是 ${where(d.back_m)}，沒有變更。`);
  } catch (e) { say(`沒有套用：${e.name === 'AbortError' ? '車端沒有回覆（結果未知，請看上方「目前起點」確認）' : e.message}`, true, e.problems || []); }
  busy = false; await poll();
};

$('startForce').onclick = async () => {
  const p = parse($('startBack').value); if (busy || p.err) return;
  if (!confirm(`四個設定檔對不上。要用保存的旗子參考值，把它們全部重寫成「${where(p.v)}」嗎？

適用：上次套用到一半斷電或中斷。
注意：如果有人刻意改過第 1 圈減速區的位置，那個改動會被蓋掉。
原檔會先備份。車要放在這個位置起跑。`)) return;
  busy = true; say('重寫中…');
  try {
    const d = await call('/api/start/set', {back_m: p.v, force: true}, 10000);
    touched = false;
    say(`已重寫：${where(d.back_m)}（起步左舵 ${d.left_bias_on ? '開' : '關'}），改了 ${d.changed.length} 個值，原檔備份 ${d.backup}。`);
  } catch (e) { say(`沒有重寫：${e.name === 'AbortError' ? '車端沒有回覆（結果未知，請看上方「目前起點」確認）' : e.message}`, true, e.problems || []); }
  busy = false; await poll();
};

$('startRebuild').onclick = async () => {
  const s = last?.start; if (busy || !s) return;
  if (!confirm(`以車上目前四個檔重建旗子參考值？\n\n這等於認定車上設定是對的、目前起點是「${s.back_m === null ? '不明' : where(s.back_m)}」。\n只有在確定車上設定是刻意改的（例如有人改了第 1 圈減速區的長度）時才按；不確定就不要按，請通知維護人員。`)) return;
  busy = true; say('重建參考中…');
  try {
    const d = await call('/api/start/rebuild-ref', {}, 8000), z = d.ref.zones[0];
    say(`參考已重建（以 ${where(d.back_m)} 反推）：c1 gap_m ${d.ref.c1_gap_m}` + (z ? `，第一個起跑區 ${z.from_m}～${z.to_m} m` : '') + `，共 ${d.ref.zones.length} 個起跑區。` + (d.problems.length ? '仍有下列項目，按「套用」補正：' : ''), false, d.problems);
  } catch (e) { say(`沒有重建：${e.name === 'AbortError' ? '車端沒有回覆' : e.message}`, true, e.problems || []); }
  busy = false; await poll();
};

render();
(async () => { while (true) { await poll(); await sleep(document.hidden ? 3000 : 1000); } })();
