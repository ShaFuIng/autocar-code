// mobile-service-r3 (2026-11-21): 圈數賽起跑模式。等綠燈的是 Orin 上的手機服務（手機關螢幕或斷線都不影響）；這裡只負責開關與顯示。
import {reasonText} from './model.mjs';
const $ = id => document.getElementById(id), sleep = ms => new Promise(r => setTimeout(r, ms));
let last = null, busy = false, note = '';

async function call(path, body, timeout = 4000) {
  const c = new AbortController(), t = setTimeout(() => c.abort(), timeout);
  try {
    const r = await fetch(path, {cache: 'no-store', signal: c.signal, ...(body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Orin-Control': '1'}, body: JSON.stringify(body)})});
    const d = await r.json();
    if (!r.ok || d.ok === false) throw Error(reasonText(d.reason, d.reason_text));
    return d;
  } finally { clearTimeout(t); }
}

function detail(g) {
  if (g.firing) return '綠燈！正在送出起跑指令…';
  if (g.armed) {
    if (g.flag === null || g.flag === undefined) return g.reads ? `連不到中控台（連續 ${g.misses} 次讀不到），仍在重試。` : '正在連線中控台…';
    if (g.flag === 1 && !g.saw_zero) return '旗標目前是 1（上一場未重置），要等裁判重置歸零後的下一次綠燈。';
    return `中控台連線正常，旗標 0，等待綠燈（讀取 ${g.read_ms ?? '—'} ms）。開啟時間 ${g.armed_at}。`;
  }
  const r = g.result;
  if (r?.ok) return `${r.at} 綠燈起跑成功，綠燈→車控接受 ${r.latency_ms} ms。起跑模式已自動關閉。`;
  if (r) return `${r.at} 綠燈但沒有起跑：${reasonText(r.code, r.text)}`;
  return '未開啟（不讀中控台）。車放到旗子起點、載入跑車版本後再開啟。';
}

function render() {
  const g = last?.race, running = last?.run?.state === 'running';
  document.body.dataset.race = g?.armed || g?.firing ? 'armed' : 'off';
  $('raceState').textContent = !g ? '—' : g.firing ? '起跑中' : g.armed ? '等待綠燈' : g.result?.ok ? '已起跑' : '未開啟';
  $('raceState').dataset.on = String(!!(g?.armed || g?.firing));
  $('raceBadge').hidden = !g?.practice;
  $('raceDetail').textContent = note || (g ? detail(g) : '讀取狀態中');
  $('raceToggle').textContent = g?.armed ? '取消起跑模式' : '開啟起跑模式（等綠燈）';
  $('raceToggle').className = g?.armed ? 'stop' : 'primary';
  $('raceToggle').disabled = busy || !g || g.firing || (!g.armed && (running || !last.model_ready || !!last.fault));
}

async function poll() { try { last = await call('/api/pid/status'); } catch { last = null; } render(); }

$('raceToggle').onclick = async () => {
  if (busy || !last?.race) return;
  const arm = !last.race.armed; busy = true; note = arm ? '開啟中…' : '取消中…'; render();
  try { await call(arm ? '/api/race/arm' : '/api/race/disarm', {}); note = ''; }
  catch (e) { note = `${arm ? '沒有開啟' : '沒有取消'}：${e.name === 'AbortError' ? '車端沒有回覆' : e.message}`; setTimeout(() => { note = ''; render(); }, 8000); }
  busy = false; await poll();
};
render();
(async () => { while (true) { await poll(); await sleep(document.hidden ? 2000 : (last?.race?.armed || last?.race?.firing ? 400 : 1000)); } })();
