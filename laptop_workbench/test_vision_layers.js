const fs=require('fs'),vm=require('vm'),assert=require('assert');
const pixels={data:new Uint8ClampedArray([0,0,0,255,255,255,255,255])};let rendered;
const tmp={drawImage(){},getImageData(){return pixels},putImageData(p){rendered=p}};
const context={window:{},document:{createElement:()=>({getContext:()=>tmp})}};vm.runInNewContext(fs.readFileSync(__dirname+'/vision-layers.js','utf8'),context);const v=context.window.VisionLayers;
const lines=[];const c={save(){},restore(){},drawImage(){},setLineDash(x){this.dash=x},beginPath(){},moveTo(x,y){lines.push([this.strokeStyle,x,y])},lineTo(){},stroke(){}};
v.mask(c,{},.25);assert.equal(rendered.data[3],0);assert.equal(rendered.data[7],255);assert.equal(c.globalAlpha,.25);
v.geometry(c,{sample_row_ratio:.7,path_diagnostic:{observed_edges:[[[10,20],[20,30]]],boundaries:[[[20,2,6],[30,4,8]]]}},{edges:true,candidates:true,row:true});assert(lines.some(x=>x[0]==='#00dfff'));assert(lines.some(x=>x[0]==='#a5adb4'));assert(lines.some(x=>x[0]==='#ffdb36'&&Math.abs(x[2]-.7*359.5)<1e-9));
lines.length=0;v.geometry(c,{path:[[1,2],[3,4]]},{raw:true,road:true,row:true});assert.equal(lines.length,0);
console.log('3 new layer checks passed: transparent background/opacity, edges+candidates+actual row, raw-only.');
