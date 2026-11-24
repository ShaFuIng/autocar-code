// Pure client-logic regression tests; no browser or real hardware.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
let storage=new Map(),element={textContent:''};
const context={console,crypto:require('node:crypto').webcrypto,sessionStorage:{},localStorage:{getItem:k=>storage.get(k)||null,setItem:(k,v)=>storage.set(k,v),removeItem:k=>storage.delete(k)},document:{getElementById:()=>element,addEventListener:()=>{}},setTimeout,clearTimeout};
vm.createContext(context);
let code=fs.readFileSync(__dirname+'/app.js','utf8').split("window.addEventListener('hashchange'")[0];
vm.runInContext(code,context);
vm.runInContext("documents.config={rev:1};draft('config',{name:'draft'});documents.config={rev:2};draft('config',{name:'still draft'});",context);
assert.equal(JSON.parse(storage.get('wb-draft-config')).rev,1,'conflict must keep original base revision');
let estimate=(id,values)=>vm.runInContext(`estimate({id:${JSON.stringify(id)}},{values:${JSON.stringify(values)}})`,context);
assert.match(estimate('C',{distance:2,left_counts:-140}),/-70.00 counts\/m/);
assert.match(estimate('F',{distance:0,seconds:0}),/資料不足/);
assert.match(estimate('F',{distance:2,seconds:4}),/0.500 m\/s/);
assert.match(estimate('H',{chord:2,sagitta:1}),/1.000 m/);
assert.match(estimate('K',{capture:10,analysis:8,ack:20}),/時間順序錯誤/);
assert.match(estimate('K',{capture:10,ack:20}),/10 ms/);
console.log('7 client regressions passed');
