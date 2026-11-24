// mobile-service-r7 (field-20261124 sprint, offline candidate): 直線衝刺模式。改的是 Orin 上車控每輪開跑時讀的 speed_adapt.json／brake_zones.json，邏輯在 sprint_cfg.py。
import {reasonText} from './model.mjs';
const $ = id => document.getElementById(id), sleep = ms => new Promise(r => setTimeout(r, ms));
let last = null, busy = false, note = '', noteBad = false, extra = [];
const START_TEXT = $('start').textContent;

async function call(path, body, timeout = 4000) {
  const c = new AbortController(), t = setTimeout(() => c.abort(), timeout);
  try {
    const r = await fetch(path, {cache: 'no-store', signal: c.signal, ...(body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Orin-Control': '1'}, body: JSON.stringify(body)})});
    const d = await r.json();
    if (!r.ok || d.ok === false) { const e = Error(reasonText(d.reason, d.reason_text)); e.problems = d.problems || []; throw e; }
    return d;
  } finally { clearTimeout(t); }
}

const m = v => (Number.isFinite(v) ? v.toFixed(2).replace(/0$/, '') : '—');
function parseFinish(text, max) {   // 同 sprint_cfg.parse_finish：留空＝手動停車；數字、0.1 m 一格、1 m～上限
  if (/^\s*$/.test(text)) return {v: null};
  if (!/^\s*\d+(\.\d+)?\s*$/.test(text)) return {err: '請填數字，或留空（手動停車）'};
  const v = Number(text);
  if (Math.abs(v * 10 - Math.round(v * 10)) > 1e-6) return {err: '精度 0.1 m'};
  if (!(v >= 1) || (Number.isFinite(max) && v > max + 1e-9)) return {err: `範圍 1～${Number.isFinite(max) ? max.toFixed(1) : '?'} m`};
  return {v: Math.round(v * 10) / 10};
}

function render() {
  const s = last?.sprint, run = last?.run?.state === 'running', paused = last?.pause?.state === 'paused', race = !!(last?.race?.armed || last?.race?.firing);
  const on = s?.on === true, bad = on && !s.consistent;
  document.body.dataset.sprint = !s ? 'unknown' : bad ? 'bad' : on ? 'on' : 'off';
  $('sprintBanner').hidden = !on;
  $('sprintBannerText').textContent = bad ? '衝刺設定異常（只寫一半或對不上），不能起跑。到下方「直線衝刺」按關閉清除。'
    : `起跑後 ${m(s?.brake_from_m)} m 內油門全開（100），之後整輪上限 ${s?.after_cap_pwm ?? 50}。` + (s?.finish_m != null ? `${s.finish_m.toFixed(1)} m 起油門歸零自動滑停。` : '要手動停車。') + '不是正常跑圈／比賽設定。';
  $('start').textContent = on ? (bad ? '衝刺設定異常（不能起跑）' : '直線衝刺起跑（全油門）') : START_TEXT;
  $('sprintState').textContent = !s ? '—' : s.on === null ? '讀不到' : bad ? '異常' : on ? '開啟中' : '關閉（正常）';
  $('sprintState').dataset.on = String(on);
  const where = s?.back_m == null ? '不明' : s.back_m === 0 ? '旗子位置 0 m' : `旗子往後 ${s.back_m.toFixed(1)} m`;
  $('sprintDetail').textContent = !s ? '讀取狀態中' : s.on === null ? reasonText(s.reason)
    : on && !bad ? `起點 ${where}（離彎 1 約 ${m(s.D)} m）：油門全開到起跑後 ${m(s.brake_from_m)} m（彎 1 前 ${m(s.brake_before_c1s_m)} m），再降到 ${s.after_cap_pwm}；油門爬升 ${s.up_rate} PWM／秒；${s.finish_m != null ? `自動停車 ${s.finish_m.toFixed(1)} m` : '手動停車'}。`
    : bad ? reasonText(s.reason)
    : s.can_on ? `目前起點 ${where}，可以開啟。開啟後油門全開到起跑後 ${m((s.D ?? 0) - (s.brake_before_c1s_m ?? 3.44))} m。` : `目前起點 ${where}：要先在「起點位置」設成旗子往後 ${(s.min_back_m ?? 2.6).toFixed(1)} m 以上（起點設定也要一致）才能開啟。`;
  const fin = parseFinish($('sprintFinish').value, s?.finish_max_m);
  $('sprintFinish').disabled = busy || on;
  $('sprintFinishHint').textContent = on ? '' : fin.err ? fin.err : (fin.v == null ? '留空：衝刺後車照常繼續跑，請手動停止本輪。' : `油門在起跑後 ${fin.v.toFixed(1)} m 歸零，約 1～1.3 m 內滑停（未實測）。`) + (Number.isFinite(s?.finish_max_m) ? `上限 ${s.finish_max_m.toFixed(1)} m（彎 1 前留滑行距離）。` : '');
  $('sprintFinishHint').dataset.bad = String(!on && !!fin.err);
  $('sprintToggle').textContent = on ? (bad ? '關閉直線衝刺（清除設定）' : '關閉直線衝刺（回正常設定）') : '開啟直線衝刺';
  $('sprintToggle').className = 'wide ' + (on ? 'stop' : 'primary');
  $('sprintToggle').disabled = busy || !s || s.on === null || run || paused || race || (!on && (!s.can_on || !!fin.err));
  const probs = bad ? (s.problems || []) : [];
  $('sprintWarn').hidden = !probs.length;
  if (probs.length) $('sprintWarn').replaceChildren(document.createTextNode('問題：'), list(probs));
  $('sprintNote').hidden = !note; $('sprintNote').dataset.bad = String(noteBad);
  $('sprintNote').replaceChildren(document.createTextNode(note), ...(extra.length ? [list(extra)] : []));
}

function list(items) { const ul = document.createElement('ul'); ul.replaceChildren(...items.map(t => { const li = document.createElement('li'); li.textContent = t; return li; })); return ul; }
function say(text, bad = false, items = []) { note = text; noteBad = bad; extra = items; render(); if (!bad) setTimeout(() => { if (note === text) { note = ''; extra = []; render(); } }, 15000); }
async function poll() { try { last = await call('/api/pid/status'); } catch { last = null; } render(); }

// 衝刺模式下按「起跑」要再確認一次（app.js 的 onclick 在這個捕獲階段的監聽之後才執行）
$('start').addEventListener('click', e => {
  const s = last?.sprint;
  if (s?.on !== true) return;
  const ok = s.consistent && confirm(`直線衝刺模式開啟中！\n\n油門全開到起跑後 ${m(s.brake_from_m)} m，之後整輪上限 ${s.after_cap_pwm}。\n${s.finish_m != null ? `起跑後 ${s.finish_m.toFixed(1)} m 油門歸零滑停。` : '要手動停車。'}\n\n這不是正常跑圈／比賽設定。確定以衝刺模式起跑？`);
  if (!ok) { e.stopImmediatePropagation(); e.preventDefault(); }
}, true);

$('sprintFinish').oninput = () => render();
$('sprintToggle').onclick = async () => {
  const s = last?.sprint; if (busy || !s) return;
  const on = s.on !== true;
  const fin = parseFinish($('sprintFinish').value, s.finish_max_m);
  if (on && fin.err) return;
  const msg = on ? `開啟直線衝刺？\n\n起點：旗子往後 ${s.back_m?.toFixed(1)} m（車要擺在這裡）。\n油門全開到起跑後 ${m((s.D ?? 0) - (s.brake_before_c1s_m ?? 3.44))} m，之後整輪上限 50。\n${fin.v == null ? '手動停車。' : `起跑後 ${fin.v.toFixed(1)} m 油門歸零滑停。`}\n下一輪起跑生效。` : '關閉直線衝刺，回到正常設定？';
  if (!confirm(msg)) return;
  busy = true; say(on ? '開啟中…' : '關閉中…');
  try {
    const d = await call('/api/sprint/set', on ? {on: true, finish_m: fin.v} : {on: false}, 10000);
    say(d.backup ? `${on ? '已開啟直線衝刺' : '已關閉直線衝刺，回到正常設定'}（改了 ${d.changed.join('、')}，原檔備份 ${d.backup}）。` : '沒有變更。');
  } catch (e) { say(`${on ? '沒有開啟' : '沒有關閉'}：${e.name === 'AbortError' ? '車端沒有回覆（結果未知，請看狀態確認）' : e.message}`, true, e.problems || []); }
  busy = false; await poll();
};

render();
(async () => { while (true) { await poll(); await sleep(document.hidden ? 3000 : 1000); } })();
