/* Camera measurement UI: snapshot analysis and documents only; no actuator API. */
(()=>{
'use strict';
const $=id=>document.getElementById(id);
let busy=false,cleanup=()=>{};
async function api(path,body){const r=await fetch(path,body?{method:'POST',headers:{'X-Workbench':'1','Content-Type':'application/json'},body:JSON.stringify(body)}:{});const d=await r.json();if(!r.ok)throw Error(d.error||r.status);return d;}
async function mount(){
 if(location.hash!=='#config'){cleanup();return;}
 if(busy||$('cameraMeasure')||!document.getElementById('main').querySelector('.heading'))return;busy=true;
 const host=document.createElement('section');host.id='cameraMeasure';host.className='card';
 host.innerHTML=`<h2>相機地面量測：拖曳距離參考線</h2>
 <p>量測起點沿用保存資料；本次已確認以前輪輪胎最前緣為0，沿地面往前量。把三條線對準實體量尺的50、80、120 cm位置；拖線身可上下左右移動整條線；拖兩端圓點可個別調整端點。端點請放在左右白線的<b>內緣</b>，紫色為V4白線區域，不是道路內緣真值。</p>
 <p><b>這是人工對位草稿，不是已完成標定。</b>初始線位置僅方便操作；拖曳不會改變實車參數。車輛與相機須保持不動。三條線本身不足以確定完整地面座標，仍需確認橫向位置及另留驗證點。</p>
 <div class="actions"><button id="cmCapture">拍攝凍結快照並辨識白線</button><button id="cmLive" class="ghost">開啟即時調整</button><button id="cmRestore" class="ghost">恢復最後保存</button></div>
 <label>預覽目標幀率<select id="cmFps"><option value="1">1 FPS</option><option value="2" selected>2 FPS</option><option value="5">5 FPS</option><option value="10">10 FPS</option><option value="15">15 FPS</option><option value="0">不限（上一張到就取下一張）</option></select></label><p>即時模式直接在下方畫面拖曳三條線；不疊舊遮罩。拍攝凍結辨識後保留線的位置，再確認保存。幀率受連線速度限制。</p>
 <p id="cmStatus" role="status">尚未拍攝；參考線尚未量測。</p>
 <label><input type="checkbox" id="cmMask" checked>顯示本張快照的紫色白線辨識</label>
 <canvas id="cmCanvas" width="640" height="360" style="width:100%;max-width:960px;display:block;touch-action:none;background:#18252b" aria-label="可拖曳50、80、120公分量測線的凍結相機畫面"></canvas>
 <p>圖上實線為你對位的距離橫線；虛線連接端點中間，不代表已知車身中線。亦可用下方像素欄位微調。</p>
 <div id="cmRows" style="max-height:350px;overflow:auto"></div>
 <label>量尺零點／相機姿態／補充說明<input id="cmNote" placeholder="確認零點在鏡頭地面垂足；相機保持固定"></label>
 <div class="actions"><button id="cmSave">保存本張快照與量測草稿</button></div>
 <p id="cmSaved"></p>`;
 document.getElementById('main').querySelector('.heading').after(host);
 let savedMetadata={},data=null,raw=null,mask=null,rev=0,live=false,timer=null,poll=null,dead=false,drag=null,dirty=false,previewBusy=false,epoch=0,lastFrame=0;
 let lines=[50,80,120].map((cm,i)=>({cm,left:[100+i*60,300-i*65],right:[540-i*60,300-i*65],confirmed:false}));
 const colors=['#ffe600','#00e5ff','#ff8b33'];
 cleanup=()=>{dead=true;live=false;epoch++;clearTimeout(timer);clearTimeout(poll);};
 const canvas=$('cmCanvas'),ctx=canvas.getContext('2d');
 function draw(){
  ctx.clearRect(0,0,canvas.width,canvas.height);if(!raw)return;ctx.drawImage(raw,0,0);
  if(mask&&$('cmMask').checked){const c=document.createElement('canvas');c.width=canvas.width;c.height=canvas.height;const x=c.getContext('2d');x.drawImage(mask,0,0);x.globalCompositeOperation='source-in';x.fillStyle='#d92de0';x.fillRect(0,0,c.width,c.height);ctx.globalAlpha=.4;ctx.drawImage(c,0,0);ctx.globalAlpha=1;}
  ctx.font=`bold ${Math.max(14,canvas.width/45)}px sans-serif`;
  lines.forEach((l,i)=>{ctx.strokeStyle=colors[i];ctx.fillStyle=colors[i];ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(...l.left);ctx.lineTo(...l.right);ctx.stroke();
   for(const p of [l.left,l.right]){ctx.beginPath();ctx.arc(...p,7,0,Math.PI*2);ctx.fill();}
   ctx.fillText(`${l.cm} cm${l.confirmed?' 已確認':' 待對位'}`,Math.min(l.left[0]+10,canvas.width-160),Math.max(20,l.left[1]-12));});
  ctx.setLineDash([5,5]);ctx.strokeStyle='#fff';ctx.beginPath();lines.forEach((l,i)=>{const p=[(l.left[0]+l.right[0])/2,(l.left[1]+l.right[1])/2];i?ctx.lineTo(...p):ctx.moveTo(...p);});ctx.stroke();ctx.setLineDash([]);
 }
 function rows(){
  $('cmRows').replaceChildren(...lines.map((l,i)=>{
   const box=document.createElement('div');box.style.cssText='border-left:4px solid '+colors[i]+';padding:10px;margin:8px 0';
   const title=document.createElement('b');title.textContent=l.cm+' cm ';box.append(title);
   for(const side of ['left','right'])for(const axis of [0,1]){
    const label=document.createElement('label');label.style.cssText='display:inline-block;margin:4px';
    label.append(document.createTextNode((side==='left'?'左':'右')+(axis?' y':' x')+' '));
    const input=document.createElement('input');input.type='number';input.step='1';input.min='0';input.max=String((axis?canvas.height:canvas.width)-1);input.value=Math.round(l[side][axis]);input.style.width='80px';input.setAttribute('aria-label',l.cm+'cm '+(side==='left'?'左':'右')+(axis?' y':' x'));
    input.oninput=()=>{const n=Number(input.value);if(!Number.isFinite(n)||input.value==='')return;l[side][axis]=Math.max(0,Math.min(Number(input.max),n));l.confirmed=false;dirty=true;draw();box.querySelector('input[type=checkbox]').checked=false;};label.append(input);box.append(label);
   }
   const label=document.createElement('label'),check=document.createElement('input');check.type='checkbox';check.checked=l.confirmed;check.onchange=()=>{l.confirmed=check.checked;dirty=true;draw();};label.append(check,document.createTextNode('我已用量尺確認此線距離'));box.append(label);return box;
  }));
 }
 function point(e){const b=canvas.getBoundingClientRect();return [(e.clientX-b.left)*canvas.width/b.width,(e.clientY-b.top)*canvas.height/b.height];}
 canvas.onpointerdown=e=>{
  if(!raw)return;const p=point(e),tolerance=20*canvas.width/canvas.getBoundingClientRect().width;
  let best=null,dist=tolerance;
  lines.forEach((l,i)=>{for(const side of ['left','right']){const d=Math.hypot(p[0]-l[side][0],p[1]-l[side][1]);if(d<dist){dist=d;best={i,side};}}
   const a=l.left,b=l.right,dx=b[0]-a[0],dy=b[1]-a[1],u=Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy||1))),d=Math.hypot(p[0]-a[0]-u*dx,p[1]-a[1]-u*dy);
   if(!best&&d<dist){dist=d;best={i,side:'both'};}});
  if(best){drag={...best,p,left:[...lines[best.i].left],right:[...lines[best.i].right]};canvas.setPointerCapture(e.pointerId);}
 };
 canvas.onpointermove=e=>{if(!drag)return;const p=point(e),l=lines[drag.i],clamp=(v,max)=>Math.max(0,Math.min(max-1,v));
  if(drag.side==='both'){let dy=p[1]-drag.p[1];dy=Math.max(-Math.min(drag.left[1],drag.right[1]),Math.min(canvas.height-1-Math.max(drag.left[1],drag.right[1]),dy));let dx=p[0]-drag.p[0];dx=Math.max(-Math.min(drag.left[0],drag.right[0]),Math.min(canvas.width-1-Math.max(drag.left[0],drag.right[0]),dx));l.left=[drag.left[0]+dx,drag.left[1]+dy];l.right=[drag.right[0]+dx,drag.right[1]+dy];}
  else l[drag.side]=[clamp(p[0],canvas.width),clamp(p[1],canvas.height)];
  l.confirmed=false;dirty=true;draw();
 };
 canvas.onpointerup=canvas.onpointercancel=()=>{drag=null;rows();};
 function image(url){return new Promise((resolve,reject)=>{const im=new Image();im.onload=()=>resolve(im);im.onerror=()=>reject(Error('快照載入失敗'));im.src=url;});}
 async function show(d,restored){
  const root='/experiment/camera-measure/'+d.id+'/';
  const ims=await Promise.all([image(root+'raw.jpg'),image(root+'mask.png')]);if(dead)return;
  data=d;[raw,mask]=ims;canvas.width=d.width;canvas.height=d.height;
  if(!restored)lines.forEach(l=>l.confirmed=false);
  rows();draw();$('cmStatus').textContent=`凍結快照：${new Date(d.received_at*1000).toLocaleString()}（筆電收到時間，非曝光時間）· ${d.width}×${d.height} · 紫色與原圖為同一張V4分析。`;
 }
 async function watch(){
  try{const d=await api('/api/camera-measure/status');if(dead)return;
   if(d.state==='running'){poll=setTimeout(watch,1000);return;}
   $('cmCapture').disabled=false;$('cmLive').disabled=false;$('cmRestore').disabled=false;$('cmSave').disabled=false;
   if(d.state==='failed')throw Error(d.error);
   if(d.state==='completed'){await show(d,false);dirty=true;}
  }catch(e){if(!dead){$('cmStatus').textContent=e.message;$('cmCapture').disabled=false;$('cmLive').disabled=false;$('cmRestore').disabled=false;$('cmSave').disabled=false;}}
 }
 $('cmCapture').onclick=async()=>{
  stopLive();data=null;mask=null;$('cmLive').disabled=true;$('cmRestore').disabled=true;$('cmSave').disabled=true;
  $('cmCapture').disabled=true;$('cmStatus').textContent='正在取得快照並在筆電執行V4；不啟動車控…';
  try{await api('/api/camera-measure/capture',{});await watch();}catch(e){$('cmStatus').textContent=e.message;$('cmCapture').disabled=false;$('cmLive').disabled=false;$('cmRestore').disabled=false;$('cmSave').disabled=false;}
 };
 function stopLive(){live=false;epoch++;clearTimeout(timer);$('cmLive').textContent='開啟即時調整';}
 async function preview(){
  if(!live||dead||document.hidden||previewBusy)return;
  previewBusy=true;const token=epoch,start=performance.now();
  try{const im=await image('/orin-camera.jpg?t='+Date.now());
   if(token!==epoch||dead||!live)return;
   if(canvas.width!==im.naturalWidth||canvas.height!==im.naturalHeight){
    const sx=im.naturalWidth/canvas.width,sy=im.naturalHeight/canvas.height;
    lines.forEach(l=>{for(const side of ['left','right'])l[side]=[l[side][0]*sx,l[side][1]*sy];});
    canvas.width=im.naturalWidth;canvas.height=im.naturalHeight;rows();
   }
   raw=im;mask=null;data=null;draw();const now=performance.now();
   $('cmStatus').textContent=`即時調整 · 目標 ${Number($('cmFps').value)>0?$('cmFps').value+' FPS':'不限'} · 實際 ${lastFrame?(1000/(now-lastFrame)).toFixed(1):'—'} FPS · ${new Date().toLocaleTimeString()} 收到影像 · 紫色遮罩暫停，三條線可直接拖曳。`;lastFrame=now;
  }catch(e){if(token===epoch&&!dead)$('cmStatus').textContent='相機更新失敗，畫面停留在上一張：'+e.message;}
  finally{previewBusy=false;if(live&&!dead)timer=setTimeout(preview,Math.max(0,(Number($('cmFps').value)>0?1000/Number($('cmFps').value):0)-(performance.now()-start)));}
 }
 $('cmLive').onclick=()=>{if(live){stopLive();$('cmStatus').textContent='已暫停預覽；請拍攝凍結辨識後保存。';}else{live=true;epoch++;lastFrame=0;mask=null;data=null;lines.forEach(l=>l.confirmed=false);rows();draw();$('cmLive').textContent='暫停即時調整';preview();}};
 $('cmFps').onchange=()=>{clearTimeout(timer);if(live)preview();};
 const visibility=()=>{if(!document.hidden&&live)preview();};document.addEventListener('visibilitychange',visibility);
 const stop=cleanup;cleanup=()=>{stop();document.removeEventListener('visibilitychange',visibility);};
 $('cmMask').onchange=draw;
 async function restore(){
  stopLive();
  const doc=await api('/api/document/camera-ground-measure');if(dead)return;rev=doc.rev;savedMetadata=doc.data||{};
  if(doc.data?.snapshot){lines=doc.data.lines;$('cmNote').value=doc.data.note||'';await show(doc.data.snapshot,true);dirty=false;$('cmSaved').textContent='已恢復草稿版本 '+rev;}
 }
 $('cmRestore').onclick=()=>restore().catch(e=>$('cmSaved').textContent=e.message);
 $('cmSave').onclick=async()=>{
  if(live||!data){$('cmSaved').textContent='請先按「拍攝凍結快照並辨識白線」，再確認保存；參考線位置會保留。';return;}
  if(lines.some(l=>l.left[0]>=l.right[0])){$('cmSaved').textContent='左端點須位於右端點左側，請檢查。';return;}
  try{const d=await api('/api/document',{key:'camera-ground-measure',rev,data:{...savedMetadata,snapshot:data,lines,note:$('cmNote').value,origin:savedMetadata.origin||'量測起點待確認',endpoint_reference:'白線內緣（人工對位）',metric_calibrated:false,hardware_output:false}});rev=d.rev;dirty=false;$('cmSaved').textContent=`已保存版本 ${rev}；${lines.filter(l=>l.confirmed).length}/3 條已由你確認。仍是標定草稿，未套用車控。`;}catch(e){$('cmSaved').textContent=e.message;}
 };
 try{await restore();}catch(e){$('cmSaved').textContent=e.message;}
 busy=false;
}
new MutationObserver(mount).observe(document.getElementById('main'),{childList:true});
window.addEventListener('hashchange',mount);mount();
})();
