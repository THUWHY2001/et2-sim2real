import assert from "node:assert/strict";
import {existsSync,readFileSync} from "node:fs";
import vm from "node:vm";

for(const file of ["torso-3d-view.js","torso-joint.fragment.html","torso-combined.fragment.html"]){
  assert.ok(existsSync(new URL(`../src/${file}`,import.meta.url)),`${file} must exist`);
}

const cameraSource=readFileSync(new URL("../src/torso-3d-view.js",import.meta.url),"utf8");
const window={};
vm.runInNewContext(cameraSource,{window,Math});
const camera=window.Torso3DView.createCamera({width:600,height:500,center:{x:-40,y:0,z:60},scale:3});
const p0=camera.project({x:-20,y:20,z:100});
camera.rotate(30,-10).zoom(8);
const p1=camera.project({x:-20,y:20,z:100});
assert.notDeepEqual({...p1},{...p0},"camera drag and zoom must change projection");
camera.reset();
assert.deepEqual({...camera.state()},{yaw:-35,pitch:-20,zoom:1},"camera reset must restore the engineering view");

const joint=readFileSync(new URL("../src/torso-joint.fragment.html",import.meta.url),"utf8");
for(const marker of [
  "data-torso-svg","data-torso-output","data-torso-play","data-torso-reset-view",
  "data-torso-status","data-torso-rod-error","data-torso-residual","data-torso-coupling",
  "data-torso-crank-l","data-torso-crank-r","data-torso-loads","data-torso-instant",
  "data-torso-run-cycle","data-torso-cycle-ranking","data-torso-cycle-chart",
])assert.match(joint,new RegExp(marker),`joint page requires ${marker}`);
for(const label of ["Yaw轴","Roll轴","Pitch轴","左曲柄","右曲柄","左上球铰","左下球铰","右上球铰","右下球铰"]){
  assert.match(joint,new RegExp(label),`joint page requires ${label}`);
}
assert.match(joint,/1 Hz、±10°/);
assert.match(joint,/正向/);
assert.match(joint,/反向/);
assert.match(joint,/静摩擦/);
assert.match(joint,/动摩擦/);
assert.match(joint,/各连接副自身瞬时速度方向/);
assert.match(joint,/pointerdown/);
assert.match(joint,/wheel/);

const combined=readFileSync(new URL("../src/torso-combined.fragment.html",import.meta.url),"utf8");
for(const marker of [
  "data-torso-svg","data-active-yaw","data-active-crank-l","data-active-crank-r",
  "data-active-roll","data-active-pitch","data-active-status","data-active-rod-error",
  "data-active-preset=\"yaw\"","data-active-preset=\"equal\"","data-active-preset=\"opposite\"",
])assert.match(combined,new RegExp(marker),`combined page requires ${marker}`);
assert.doesNotMatch(combined,/data-active-roll-input|data-active-pitch-input/,"combined mode must not directly drive outputs");
assert.match(combined,/真实主动输入/);

for(const file of ["torso-j0.html","torso-j1.html","torso-j2.html","torso-combined.html","torso-model.js","torso-3d-view.js"]){
  assert.ok(existsSync(new URL(`../dist/${file}`,import.meta.url)),`dist/${file} must be generated`);
  assert.ok(existsSync(new URL(`../${file}`,import.meta.url)),`${file} must be generated at site root`);
}
const index=readFileSync(new URL("../dist/index.html",import.meta.url),"utf8");
for(const file of ["torso-j0.html","torso-j1.html","torso-j2.html","torso-combined.html"])assert.match(index,new RegExp(`href="${file.replace('.','\\.')}"`),`home page links ${file}`);
const generated={
  J0:readFileSync(new URL("../dist/torso-j0.html",import.meta.url),"utf8"),
  J1:readFileSync(new URL("../dist/torso-j1.html",import.meta.url),"utf8"),
  J2:readFileSync(new URL("../dist/torso-j2.html",import.meta.url),"utf8"),
  combined:readFileSync(new URL("../dist/torso-combined.html",import.meta.url),"utf8"),
};
for(const [key,page] of Object.entries(generated)){
  assert.match(page,/src="torso-model\.js\?v=/,`${key} loads torso model`);
  assert.match(page,/src="torso-3d-view\.js\?v=/,`${key} loads torso camera`);
  assert.match(page,/aria-label="躯干等效旋转轴"/,`${key} has torso subnavigation`);
  assert.doesNotMatch(page,/<iframe|data-srcdoc=/,`${key} stays a direct page`);
}
assert.match(generated.J0,/躯干 J0 Yaw 阻力分析/);
assert.match(generated.J1,/躯干 J1 Roll 阻力分析/);
assert.match(generated.J2,/躯干 J2 Pitch 阻力分析/);
assert.match(generated.combined,/躯干三主动轴组合运动/);

console.log("torso page source regression test passed");
