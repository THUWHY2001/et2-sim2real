import assert from "node:assert/strict";
import {readFileSync} from "node:fs";

const fragment=readFileSync(new URL("../src/leg-full-joint.fragment.html",import.meta.url),"utf8");
for(const text of ["ET2 左腿完整三推杆闭链","三维完整机构","20 个运动刚体","23 条硬点记录","50 条运动副 Marker","连接副阻力输入","实时贡献度","完整周期贡献度","承载参数","加载后实时贡献度","加载后完整周期贡献度","总力矩组成","闭链运动关系","质量属性范围"])assert.match(fragment,new RegExp(text),`missing ${text}`);
for(const marker of ["data-leg-full-svg","data-leg-full-loads","data-leg-full-load-settings","data-leg-full-loaded-live","data-leg-full-loaded-cycle","data-leg-full-cycle-chart","data-leg-full-mass-rows"])assert.match(fragment,new RegExp(marker),`missing ${marker}`);
assert.match(fragment,/createLegFullRuntime/);
assert.match(fragment,/data\/leg-full-package\/model\.json/);
assert.match(fragment,/M\.solveOutput\(joint,value\)/,"output command must go through the runtime inverse+forward closed-chain path");
assert.doesNotMatch(fragment,/directAngles\(/,"page must not directly drive J2/J3/J4 angles");
assert.match(fragment,/hip_mm/);assert.match(fragment,/knee_mm/);assert.match(fragment,/ankle_mm/);

for(const [file,joint] of [["../j2.html","J2"],["../j3.html","J3"],["../j4.html","J4"]]){
  const page=readFileSync(new URL(file,import.meta.url),"utf8");
  assert.match(page,new RegExp(`data-leg-full-joint="${joint}"`),`${joint} must use the full closed-chain page`);
  assert.match(page,/leg-full-runtime\.mjs\?v=20261106-leg-full-1/,`${joint} must load the cache-busted full runtime`);
  assert.match(page,/闭链运动关系/);
  assert.match(page,/加载后实时贡献度/);
}

console.log("leg full page structure regression test passed");
