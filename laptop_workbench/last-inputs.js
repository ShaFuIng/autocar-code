/* Last-input drafts only. Restoration never dispatches events or invokes actions. */
'use strict';
window.LastInputs=(()=>{
 const prefix='wb-last-inputs-v1:';
 const bindings=new WeakMap();
 const denied=new Set(['livePIDClear','livePIDPose','trialRecordOnly','areaClear','groundClear','wheelsRaised']);
 function read(scope){try{const x=JSON.parse(localStorage.getItem(prefix+scope));return x&&typeof x==='object'&&!Array.isArray(x)?x:{};}catch{return {};}}
 function put(scope,values){try{localStorage.setItem(prefix+scope,JSON.stringify({...read(scope),...values}));return true;}catch{return false;}}
 function eligible(el){return el&&!denied.has(el.id)&&!['file','password','hidden','submit','button'].includes(el.type);}
 function value(el){return ['checkbox','radio'].includes(el.type)?el.checked:el.value;}
 function restore(el,saved){
  if(!eligible(el)||saved===undefined)return false;
  if(['checkbox','radio'].includes(el.type)){if(typeof saved!=='boolean')return false;el.checked=saved;return true;}
  if(typeof saved!=='string')return false;
  if(el.tagName==='SELECT'&&![...el.options].some(o=>o.value===saved&&!o.disabled))return false;
  if(el.type==='number'||el.type==='range'){
   const n=Number(saved);if(saved!==''&&(!Number.isFinite(n)||(el.min!==''&&n<Number(el.min))||(el.max!==''&&n>Number(el.max))))return false;
  }
  el.value=saved;return true;
 }
 function bind(scope,ids,root=document,refresh=false){
  const saved=read(scope);let restored=0;
  for(const id of ids){const el=typeof id==='string'?root.querySelector('#'+id):id;if(!eligible(el))continue;
   const key=el.id||el.name||el.dataset.key;if(!key)continue;
   const old=bindings.get(el);if(old?.scope===scope&&!refresh)continue;
   if(old){el.removeEventListener('input',old.save);el.removeEventListener('change',old.save);}
   if(restore(el,saved[key]))restored++;
   const save=()=>put(scope,{[key]:value(el)});
   el.addEventListener('input',save);el.addEventListener('change',save);bindings.set(el,{scope,save});
  }
  return restored;
 }
 function capture(scope,ids,root=document){return put(scope,Object.fromEntries(ids.map(id=>root.querySelector('#'+id)).filter(eligible).map(el=>[el.id,value(el)])));}
 return {read,put,bind,capture,restore};
})();
