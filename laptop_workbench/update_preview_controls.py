from pathlib import Path
p=Path(__file__).resolve().parents[1]/'web/manual.html';s=p.read_text(encoding='utf-8-sig')
s=s.replace('<h2>即時相機</h2>','<h2>即時相機</h2><label>筆電預覽 <select id="previewRate"><option value="0">關閉預覽（現場目視）</option><option value="1">1 FPS</option><option value="2">2 FPS</option><option value="5" selected>5 FPS</option><option value="10">10 FPS</option></select></label><p class="muted">只影響筆電預覽流量，不更改 Orin 相機取像與錄影。關閉時請在車旁目視操作；相機健康與通訊失聯保護仍有效。</p>')
s=s.replace('min="0.2" max="5"','min="0" max="5"')
s=s.replace('時間表示 PWM 從零升至設定上限','0 秒表示直接設定 PWM（保留韌體毫秒級漸增）；大於 0 的時間表示 PWM 從零升至設定上限')
s=s.replace("let runId=null", "let lastStartError='';\nlet runId=null",1)
s=s.replace("if(!canDrive()||!previewFresh()){el('message').textContent='先啟用手動模式，確認即時畫面正常。';return}","if(!canDrive()||!previewFresh()){el('message').textContent=lastStartError||'請先按「啟用手動模式並定位至中心」，再點入駕駛操作區。';return}")
start=s.index("el('enable').onclick=");end=s.index('\n',start)
line=s[start:end].replace("try{if(linkTesting)","try{lastStartError='';if(linkTesting)").replace("el('message').textContent=e.message","lastStartError='啟用失敗：'+e.message;el('message').textContent=lastStartError;diagnostic('page_error',lastStartError)")
s=s[:start]+line+s[end:]
s=s.replace("function previewFresh(){return lastRenderedAt!==null&&performance.now()-lastRenderedAt<1000}","""let previewRate=5;
try{const saved=Number(localStorage.getItem('orin-preview-rate'));if(localStorage.getItem('orin-preview-rate')!==null&&[0,1,2,5,10].includes(saved))previewRate=saved}catch{}
el('previewRate').value=String(previewRate);
el('previewRate').onchange=()=>{previewRate=Number(el('previewRate').value);try{localStorage.setItem('orin-preview-rate',String(previewRate))}catch{}streamEpoch++;streamController?.abort();pendingJpeg=null;lastRenderedAt=null;el('camera').style.visibility=previewRate?'visible':'hidden';streamError=previewRate?'等待新影格':'預覽已關閉：現場目視操作';};
el('camera').style.visibility=previewRate?'visible':'hidden';
function previewFresh(){return previewRate===0?!!(lastStatus&&!lastStatus.camera_error&&lastStatus.camera_age_s<1&&lastStatus.uart_age_s<1):lastRenderedAt!==null&&performance.now()-lastRenderedAt<Math.max(1000,2000/previewRate)}""")
start=s.index('async function camera(){');end=s.index('function previewMetrics(){',start)
s=s[:start]+'''async function camera(){
 if(document.hidden||previewRate===0){setTimeout(camera,250);return}
 const epoch=++streamEpoch,controller=new AbortController();streamController=controller;streamStartedAt=performance.now();
 const deadline=setTimeout(()=>controller.abort(),2000);
 try{const r=await fetch('/camera.jpg?t='+Date.now(),{cache:'no-store',priority:'low',signal:controller.signal});if(!r.ok)throw Error('預覽影格不可用');const bytes=new Uint8Array(await r.arrayBuffer());if(epoch===streamEpoch&&previewRate){pendingJpeg={bytes,epoch};await renderLatest()}}
 catch(e){if(epoch===streamEpoch&&previewRate){streamError='預覽失敗：'+e.message;diagnostic('preview_error',e.message)}}
 finally{clearTimeout(deadline);controller.abort();if(streamController===controller)streamController=null;setTimeout(camera,Math.max(0,1000/Math.max(previewRate,1)-(performance.now()-streamStartedAt)))}
}
'''+s[end:]
s=s.replace("(streamError||'串流正常')+' · 顯示 '","(previewRate===0?'預覽已關閉（相機仍在 Orin 運作）':streamError||'預覽正常，要求 '+previewRate+' FPS')+' · 顯示 '")
s=s.replace("age>1.5", "age>Math.max(1.5,2/Math.max(previewRate,1))")
# Give every disabled condition an explicit explanation, using the same gates.
start=s.index(" el('enableHint').textContent=");end=s.index('\n',start)
s=s[:start]+" el('enableHint').textContent=!controlReady?'控制 WebSocket 尚未連線':!el('areaClear').checked?'請先確認現場環境':linkTesting?'連線測試進行中':active(lastRun)?'手動模式已啟用':!fresh?'相機或 UART 資料過期':!lastStatus.servo_settings_synced?'等待共用舵機設定同步':v?.pins?.D11!==1?'實體急停尚未釋放':!(v?.state==='READY'||(v?.state==='LOCKED'&&v?.web_manual_start===true))?'目前狀態不允許啟用':v?.fw!==d.firmware_required?'韌體版本不相符':!previewFresh()?'等待預覽影格；可選關閉預覽並現場目視':'可按下方啟用；啟用後按住 W 才會前進';"+s[end:]
p.write_text(s,encoding='utf-8')
