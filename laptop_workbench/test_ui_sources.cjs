// Exercise the actual parameter read function with fake form elements.
// No HTTP, SSH or hardware calls.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const source=fs.readFileSync(__dirname+'/pid-console.js','utf8');
const valuesSource=source.slice(source.indexOf(' function values(){'),source.indexOf(' function fill('));
const parameters={kp:240,direction_enabled:1,direction_far_row:.5,backlash_enabled:1,backlash_initial_enabled:1,future_field:42};
function run(elements){
 const ctx={doc:{data:{parameters}},names:{kp:'P'},$:id=>elements[id]};
 vm.createContext(ctx);vm.runInContext(valuesSource+';result=values();',ctx);
 return JSON.parse(JSON.stringify(ctx.result));
}
const edited=run({fieldParams:{},'field-kp':{value:'260'}});
assert.equal(edited.kp,260);
assert.equal(edited.direction_enabled,1);
assert.equal(edited.backlash_initial_enabled,1);
assert.equal(edited.future_field,42,'saving a visible field must not erase other parameters');
assert.deepEqual(run({}),parameters,'offline view uses the same saved parameters');
assert.throws(()=>run({fieldParams:{},'field-kp':{value:''}}),/有效數字/);
assert.equal(parameters.kp,240,'reading form must not mutate the saved document');
const offline=fs.readFileSync(__dirname+'/pid.js','utf8').split('/* CURVE_REPLAY_PANEL_V1 */')[0];
assert(!offline.includes("key:'visual-pid'"),'06 must not write a second PID configuration');
assert(offline.includes("latest.rev!==doc.rev"),'reject replay with stale displayed revision');
console.log('9 parameter-source assertions passed; all APIs absent');
