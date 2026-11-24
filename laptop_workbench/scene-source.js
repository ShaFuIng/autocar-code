/* Offline 3D geometry viewer. No hardware endpoints or external dependencies. */
const VehicleSceneMath={
 near(h,pitch,vfov){const a=(pitch+vfov/2)*Math.PI/180;return a<=0?null:a>=Math.PI/2?0:h/Math.tan(a)},
 far(h,pitch,vfov){const a=(pitch-vfov/2)*Math.PI/180;return a<=0?null:h/Math.tan(a)}
};
const vehicleSceneBase=configPage;
configPage=async()=>{
 await vehicleSceneBase();if(route!=='config')return;
 const key='vehicle-scene-simulation';let measure,record;
 try{[measure,record]=await Promise.all([api('/api/document/vehicle-measurements'),api('/api/document/'+key)]);localStorage.setItem('wb-scene-measured-cache',JSON.stringify(measure))}catch(e){try{measure=JSON.parse(localStorage.getItem('wb-scene-measured-cache'))}catch{}record={rev:0,data:{}};if(!measure)return}
 if(route!=='config')return;
 const items=measure.data.items||{}, assumptions=[];
 const dim=(id,fallback,label)=>{const r=items[id];if(r?.value!=null&&Number.isFinite(Number(r.value))){if(r.status!=='measured')assumptions.push(label+'：'+r.value+'（估算／待確認）');return Number(r.value)}assumptions.push(label+'：'+fallback+' cm（暫估）');return fallback};
 const wheelbase=dim('wheelbase_left',23,'軸距'),tf=dim('track_front',26.5,'前輪輪距'),tr=dim('track_rear',27.5,'後輪輪距'),rd=dim('diameter_rear_left',7.5,'後輪外徑'),fd=dim('diameter_front_left',7.5,'前輪外徑'),fw=dim('width_front_left',4.5,'前輪胎寬'),rw=dim('width_rear_left',4.5,'後輪胎寬'),deck=dim('deck_height',13.5,'頂板高度'),camX=dim('camera_forward',10,'相機前伸'),camZ=dim('camera_left',0,'相機左右偏移');
 const measured={height:dim('camera_height',14.75,'相機高度'),pitch:dim('camera_pitch',0,'相機俯角'),hfov:dim('camera_hfov',66,'水平視角'),vfov:dim('camera_vfov',40,'垂直視角')};
 const nearMeasured=items.camera_near_ground_front_tire?.value??null;
 const frontEdgeX=fd/2, measuredCameraNear=nearMeasured===null?null:frontEdgeX+nearMeasured-camX;
 assumptions.push("最近可見距離以前輪輪胎最前緣為零點；前輪外徑未量時使用暫估值，只影響白模座標與反推俯角，不改27 cm實量。過去39／32 cm也同此基準。");
 if(items.camera_pitch?.value==null&&measuredCameraNear>0){measured.pitch=Math.atan(measured.height/measuredCameraNear)*180/Math.PI-measured.vfov/2;assumptions.push('模擬俯角 '+measured.pitch.toFixed(2)+'° 由前輪基準近界、暫估輪徑與估算視角換算，非實量');}

 assumptions.push('木板外形、厚度 0.5 cm、中下層高度、支柱、連桿及設備大小：依照片暫估','左右輪暫採同尺寸；外八角未量測，模型先畫平行輪；不代表轉向標定');
 const savedCompatible=record.data.measurement_revision===measure.rev&&record.data.distance_reference==='front_tire_foremost_edge';
 let rev=record.rev, s={...measured,...(savedCompatible?record.data.parameters:{})},draft;try{draft=JSON.parse(localStorage.getItem('wb-scene-draft'))}catch{}if(draft&&draft.measurement_revision===measure.rev&&draft.distance_reference==='front_tire_foremost_edge'){s={...measured,...draft.parameters};rev=draft.rev}else{draft=null}
 const sanitize=()=>{for(const [k,lo,hi] of [['height',1,60],['pitch',-20,70],['hfov',10,140],['vfov',10,100]])s[k]=Math.max(lo,Math.min(hi,Number.isFinite(Number(s[k]))?Number(s[k]):measured[k]))};sanitize();
 $('geometryMeasurements').insertAdjacentHTML('beforebegin',`<section class="card" id="vehicleScene"><h2>3D 白模車與相機視野 <span class="pill">幾何估算 · 不控制實車</span></h2><p>拖曳旋轉、滾輪縮放。白模尺寸取自已保存量測 v${measure.rev}；藍色是幾何視野，不代表模型辨識成功。單位 cm。</p><div class="actions"><button type="button" data-view="orbit">立體</button><button type="button" data-view="top">俯視</button><button type="button" data-view="side">側視</button><button type="button" data-view="camera">虛擬相機視角</button></div><label><input type="checkbox" id="sceneDimensions" checked> 顯示重點尺寸（已量／估算）</label><p class="ribbon">此圖為模擬，調整不會移動實車相機；「恢復目前實量／估算設定」可回到保存的量測基準。</p><canvas id="vehicleCanvas" style="display:block;width:100%;height:440px;background:#eef3f1;touch-action:none;border-radius:12px" aria-label="可旋轉的3D白模車與藍色視野示意"></canvas><p class="note">白色：車體／設備；藍色：視錐與地面覆蓋；橘色：最近可見邊界；紫色：前輪輪胎最前緣往前 ${nearMeasured??'未知'} cm 實量參考線；橘色依模擬參數計算。前方顯示到 180 cm，並非相機最遠距離。車身遮擋與鏡頭畸變未計算。</p><div class="formgrid">${[['height','模擬相機高度 cm',1,60,.25],['pitch','模擬俯角 °（向下為正）',-20,70,.5],['hfov','水平視角 °（估算）',10,140,.5],['vfov','垂直視角 °（估算）',10,100,.5]].map(([k,label,min,max,step])=>`<label>${label}<input id="scene-${k}" type="number" min="${min}" max="${max}" step="${step}" value="${s[k]}"><input aria-label="${label}滑桿" id="scene-range-${k}" type="range" min="${min}" max="${max}" step="${step}" value="${s[k]}"></label>`).join('')}</div><div id="sceneDistances" class="ribbon" role="status"></div><details><summary>固定假設比較：0°／下俯10°／15°（不是目前實測俯角）</summary><p>固定高度14.75 cm、視角66°×40°，相機置中且朝直線跑道；淨寬46 cm，含白線總寬55 cm。下列距離均從鏡頭地面投影往前量，為幾何估算，不隨上方滑桿改變。</p><table><tr><th>俯角</th><th>最近地面</th><th>最近處可見寬</th><th>兩條白線全寬皆入鏡起點</th></tr><tr><td>0°</td><td>40.5 cm</td><td>52.6 cm</td><td>約42.3 cm</td></tr><tr><td>10°</td><td>25.5 cm</td><td>36.0 cm</td><td>約40.4 cm</td></tr><tr><td>15°</td><td>21.1 cm</td><td>31.4 cm</td><td>約39.9 cm</td></tr></table><p>下俯能看到更近的中央地面，但最近處未必同時包含兩條白線。0°最近處可見兩條白線的一部分；10°／15°最近處連兩側白線內緣也尚未同時入鏡。偏離跑道中心、彎道、外八、遮擋及鏡頭畸變需另驗證。</p></details><div class="actions"><button id="sceneSave">保存模擬方案</button><button class="secondary" id="sceneMeasured">恢復目前實量／估算設定</button><button class="ghost" id="sceneHistory">模擬保存紀錄</button></div><p id="sceneSaveStatus" role="status">${draft?'已恢復瀏覽器模擬草稿':savedCompatible?'已載入模擬方案 v'+rev:'目前使用實量與估算初值；模擬尚未保存'}</p><div id="sceneHistoryList" style="max-height:250px;overflow:auto"></div><details><summary>資料來源、未知尺寸與幾何限制</summary><ul>${assumptions.map(a=>'<li>'+esc(a)+'</li>').join('')}</ul><p>相機偏航、側傾暫按 0°；跑道直線淨寬46、白線各4.5。視野假設平地、針孔模型。調整只改模擬，不覆蓋實量、不改 Orin。離線指不需網際網路，仍須啟動筆電本機工作台。</p></details></section>`);
 const canvas=$('vehicleCanvas'),ctx=canvas.getContext('2d');let mode='orbit',yaw=-.8,elev=.65,zoom=1,drag=null;
 const vec=(a,b)=>a.map((x,i)=>x-b[i]),dot=(a,b)=>a.reduce((sum,x,i)=>sum+x*b[i],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],norm=a=>{const n=Math.hypot(...a);return a.map(x=>x/n)};
 function render(){if(!canvas.isConnected)return;const W=canvas.clientWidth,H=440,dpr=window.devicePixelRatio||1;canvas.width=W*dpr;canvas.height=H*dpr;ctx.scale(dpr,dpr);ctx.clearRect(0,0,W,H);
 const pitch=s.pitch*Math.PI/180,hv=Math.tan(s.hfov*Math.PI/360),vv=Math.tan(s.vfov*Math.PI/360),camera=[camX,s.height,camZ],f=[Math.cos(pitch),-Math.sin(pitch),0],up=[Math.sin(pitch),Math.cos(pitch),0];
 let eye,target=[30,0,0],right,U,F,scale=Math.min(W/210,H/145)*zoom;
 if(mode==='camera'){eye=camera;F=f;right=[0,0,-1];U=up}else{const y=mode==='top'?0:mode==='side'?-Math.PI/2:yaw,el=mode==='top'?Math.PI/2-.001:mode==='side'?.03:elev;eye=[target[0]+220*Math.cos(el)*Math.cos(y),220*Math.sin(el),220*Math.cos(el)*Math.sin(y)];F=norm(vec(target,eye));right=norm(cross(F,[0,1,0]));U=cross(right,F)}
 const project=p=>{const a=vec(p,eye),z=dot(a,F);if(mode==='camera'){if(z<=.1)return null;const fitW=Math.min(W,H*16/9),fitH=fitW*9/16;return [W/2+dot(a,right)/z/hv*fitW/2,H/2-dot(a,U)/z/vv*fitH/2,z]}return [W/2+dot(a,right)*scale,H*.57-dot(a,U)*scale,z]};
 const faces=[];const poly=(points,color,stroke='#677a79')=>{if(mode==='camera'){const clipped=[];for(let i=0;i<points.length;i++){const a=points[i],b=points[(i+1)%points.length],da=dot(vec(a,eye),F)-.11,db=dot(vec(b,eye),F)-.11;if(da>=0)clipped.push(a);if((da>=0)!==(db>=0)){const t=da/(da-db);clipped.push(a.map((v,j)=>v+t*(b[j]-v)))}}points=clipped;if(points.length<3)return}const p=points.map(project);if(p.some(x=>!x))return;faces.push({p,color,stroke,z:p.reduce((a,v)=>a+v[2],0)/p.length})};
 const box=(x,y,z,l,h,w,color)=>{const v=[[x,y,z],[x+l,y,z],[x+l,y+h,z],[x,y+h,z],[x,y,z+w],[x+l,y,z+w],[x+l,y+h,z+w],[x,y+h,z+w]];for(const a of [[0,1,2,3],[4,7,6,5],[0,4,5,1],[3,2,6,7],[0,3,7,4],[1,5,6,2]])poly(a.map(i=>v[i]),color)};
 for(let x=-50;x<190;x+=10)poly([[x,-.12,-65],[x+.15,-.12,-65],[x+.15,-.12,65],[x,-.12,65]],'#ccd8d3',null);
 for(let z=-60;z<=60;z+=10)poly([[-50,-.1,z],[190,-.1,z],[190,-.1,z+.15],[-50,-.1,z+.15]],'#ccd8d3',null);
 for(const z of [-27.5,23])poly([[-50,0,z],[190,0,z],[190,0,z+4.5],[-50,0,z+4.5]],'#ffffff','#c2c9c4');
 // Intersect camera cone with the finite display ground rectangle.
 let ground=[[-50,.08,-100],[190,.08,-100],[190,.08,100],[-50,.08,100]];
 const planes=[a=>dot(a,f),a=>dot(a,f)*hv-a[2],a=>dot(a,f)*hv+a[2],a=>dot(a,f)*vv-dot(a,up),a=>dot(a,f)*vv+dot(a,up)];
 for(const plane of planes){const result=[];for(let i=0;i<ground.length;i++){const a=ground[i],b=ground[(i+1)%ground.length],da=plane(vec(a,camera)),db=plane(vec(b,camera));if(da>=0)result.push(a);if((da>=0)!==(db>=0)){const t=da/(da-db);result.push(a.map((v,j)=>v+t*(b[j]-v)))}}ground=result}
 if(ground.length>2)poly(ground,'rgba(40,160,210,.26)','#218aaf');
 const near=VehicleSceneMath.near(s.height,s.pitch,s.vfov),far=VehicleSceneMath.far(s.height,s.pitch,s.vfov);
 if(near!==null&&near<=180){const x=camX+near;poly([[x,.2,-26],[x+.5,.2,-26],[x+.5,.2,26],[x,.2,26]],'#ec8734',null)}
 if(nearMeasured!=null)poly([[frontEdgeX+nearMeasured,.22,-22],[frontEdgeX+nearMeasured+.3,.22,-22],[frontEdgeX+nearMeasured+.3,.22,22],[frontEdgeX+nearMeasured,.22,22]],'#9860b0',null);
 for(const h of [2,7,deck]){const outline=[[-wheelbase-3,h,-11],[-5,h,-11],[4,h,-6],[4,h,6],[-5,h,11],[-wheelbase-3,h,11]];poly(outline.map(p=>[p[0],p[1]+.5,p[2]]),'#f6f3e9');poly(outline,'#e1e4de');for(let i=0;i<outline.length;i++){const a=outline[i],b=outline[(i+1)%outline.length];poly([a,b,[b[0],b[1]+.5,b[2]],[a[0],a[1]+.5,a[2]]],'#d6dcd7')}}
 for(const x of [-wheelbase+1,1])for(const z of [-8,8])box(x,2,z,.5,deck-2,.5,'#d0d5d2');
 box(-wheelbase+1,deck+.5,-5,8,3,10,'#e1e8e5');box(-wheelbase+4,7.5,-5,11,3,10,'#e4e8e5');
 const wheel=(x,z,d,w)=>{const n=20,r=d/2;for(let i=0;i<n;i++){const a=i*2*Math.PI/n,b=(i+1)*2*Math.PI/n;poly([[x+r*Math.cos(a),r+r*Math.sin(a),z-w/2],[x+r*Math.cos(b),r+r*Math.sin(b),z-w/2],[x+r*Math.cos(b),r+r*Math.sin(b),z+w/2],[x+r*Math.cos(a),r+r*Math.sin(a),z+w/2]],i%3===0?'#bbc6c3':'#e9eeeb')}
 for(const side of [-1,1])poly(Array.from({length:n},(_,i)=>[x+r*Math.cos(i*2*Math.PI/n),r+r*Math.sin(i*2*Math.PI/n),z+side*w/2]),'#f4f6f3')};
 for(const side of [-1,1]){wheel(0,side*tf/2,fd,fw);wheel(-wheelbase,side*tr/2,rd,rw)}
 box(-1,fd/2,-tf/2,1,1,tf,'#bac5c0');box(camX-2,deck+.5,camZ-.4,1,Math.max(.2,s.height-deck-1),.8,'#ced8d3');box(camX-2,s.height-1,camZ-3,2,2,6,'#eceee9');
 const corners=[];for(const [u,v] of [[-1,-1],[1,-1],[1,1],[-1,1]])corners.push(camera.map((a,i)=>a+f[i]*50+(i===2?u*hv*50:0)+up[i]*v*vv*50));
 if(mode!=='camera')for(let i=0;i<4;i++)poly([camera,corners[i],corners[(i+1)%4]],'rgba(38,166,220,.08)','#51a5c0');
 faces.sort((a,b)=>b.z-a.z);for(const q of faces){ctx.beginPath();q.p.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.fillStyle=q.color;ctx.fill();if(q.stroke){ctx.strokeStyle=q.stroke;ctx.lineWidth=.65;ctx.stroke()}}
 if($('sceneDimensions').checked&&mode!=='camera'){
 const labelRects=[];const annotation=(a,b,label)=>{const pa=project(a),pb=project(b);if(!pa||!pb)return;ctx.save();ctx.strokeStyle='#a74c22';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(pa[0],pa[1]);ctx.lineTo(pb[0],pb[1]);ctx.stroke();for(const q of [pa,pb]){ctx.beginPath();ctx.arc(q[0],q[1],2.5,0,Math.PI*2);ctx.fillStyle='#a74c22';ctx.fill()}ctx.font='12px sans-serif';const w=ctx.measureText(label).width;const x=Math.max(3,Math.min(W-w-8,(pa[0]+pb[0])/2-w/2)),y=Math.max(40,Math.min(H-18,(pa[1]+pb[1])/2-8));let ly=y;for(let j=0;j<10&&labelRects.some(r=>x<r.x+r.w&&x+w>r.x&&ly-13<r.y+18&&ly+5>r.y);j++)ly-=22;labelRects.push({x:x-3,y:ly-13,w:w+6});ctx.strokeStyle='#a74c22';ctx.beginPath();ctx.moveTo((pa[0]+pb[0])/2,(pa[1]+pb[1])/2);ctx.lineTo(x+w/2,ly);ctx.stroke();ctx.fillStyle='rgba(255,255,255,.94)';ctx.fillRect(x-3,ly-13,w+6,18);ctx.fillStyle='#573722';ctx.fillText(label,x,ly);ctx.restore()};
 annotation([-wheelbase,0,-tr/2-6],[0,0,-tr/2-6],'軸距 '+wheelbase+' cm');
 annotation([-wheelbase-6,0,-tr/2],[-wheelbase-6,0,tr/2],'後輪距 '+tr+' cm');
 annotation([3,0,-tf/2],[3,0,tf/2],'前輪距 '+tf+' cm');
 annotation([camX,0,camZ],[camX,s.height,camZ],'模擬鏡頭高 '+s.height+' cm');
 if(nearMeasured!==null)annotation([frontEdgeX,.3,32],[frontEdgeX+nearMeasured,.3,32],'前輪最前緣 → 最近地面 '+nearMeasured+' cm（實量）');
 }
 ctx.fillStyle='#193e3c';ctx.font='14px sans-serif';ctx.fillText(mode==='camera'?'虛擬相機視角（非實景）':'車頭 → 相機方向；地面格距 10 cm',12,24);
 if(mode==='camera'){const w=Math.min(W,H*16/9),h=w*9/16;ctx.strokeStyle='#244';ctx.strokeRect((W-w)/2,(H-h)/2,w,h)}
 $('sceneDistances').textContent=`實量最近地面（前輪最前緣往前）：${nearMeasured??'未知'} cm。模擬最近地面（同基準）：${near===null?'無前方平地交點':(camX+near-frontEdgeX).toFixed(1)+' cm'}；模擬距前軸：${near===null?'—':(camX+near).toFixed(1)+' cm'}。最遠平地交點：${far===null?'無有限上界（視線達水平或天空）':far.toFixed(1)+' cm'}。視角 ${s.hfov}° × ${s.vfov}°（估算）。`;
 }
 const changed=()=>{sanitize();for(const k of ['height','pitch','hfov','vfov']){$('scene-'+k).value=s[k];$('scene-range-'+k).value=s[k]}localStorage.setItem('wb-scene-draft',JSON.stringify({rev,parameters:s,measurement_revision:measure.rev,distance_reference:'front_tire_foremost_edge'}));$('sceneSaveStatus').textContent='模擬草稿已保留；未改實量資料';render()};
 for(const k of ['height','pitch','hfov','vfov'])for(const prefix of ['scene-','scene-range-'])$(prefix+k).oninput=e=>{if(e.target.value==='')return;s[k]=Number(e.target.value);changed()};
 $('vehicleScene').querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>{mode=b.dataset.view;render()});
 canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};canvas.onpointermove=e=>{if(!drag)return;mode='orbit';yaw-=(e.clientX-drag[0])*.008;elev=Math.max(.08,Math.min(1.5,elev+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];render()};canvas.onpointerup=canvas.onpointercancel=()=>drag=null;canvas.onwheel=e=>{e.preventDefault();zoom=Math.max(.4,Math.min(3,zoom*Math.exp(-e.deltaY*.001)));render()};
 $('sceneDimensions').onchange=render;
 $('sceneMeasured').onclick=()=>{s={...measured};changed()};
 $('sceneSave').onclick=async()=>{const b=$('sceneSave');b.disabled=true;try{const r=await api('/api/document',{key,rev,data:{parameters:{...s},measurement_revision:measure.rev,distance_reference:'front_tire_foremost_edge',kind:'simulation-only',assumptions}});rev=r.rev;localStorage.removeItem('wb-scene-draft');$('sceneSaveStatus').textContent='模擬方案已保存 v'+rev+'；實量資料未更改'}catch(e){$('sceneSaveStatus').textContent='保存失敗，草稿保留：'+e.message}finally{b.disabled=false}};
 $('sceneHistory').onclick=async()=>{try{const h=await api('/api/history/'+key);$('sceneHistoryList').innerHTML=h.slice(0,10).map(r=>`<details><summary>模擬 v${r.rev}</summary><pre>${esc(JSON.stringify(r.data.parameters,null,2))}</pre></details>`).join('')||'尚未保存'}catch(e){$('sceneSaveStatus').textContent=e.message}};
 const observer=new ResizeObserver(()=>{if(canvas.isConnected)render();else observer.disconnect()});observer.observe(canvas);render();
};
