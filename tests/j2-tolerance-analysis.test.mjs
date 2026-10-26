import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

const fragment=readFileSync("src/j2.fragment.html","utf8");
const source=fragment.match(/<script data-j2-core>([\s\S]*?)<\/script>/)?.[1];
assert.ok(source,"J2 core model script must exist");
const context={window:{}};
vm.createContext(context);
vm.runInContext(readFileSync("src/directional-friction.js","utf8"),context);
vm.runInContext(source,context);
const M=context.window.J2Model;

assert.equal(M.toleranceForLength(99.999),1,"lengths below 100 mm use ±1 mm");
assert.equal(M.toleranceForLength(100),1,"100–200 mm uses ±1 mm");
assert.equal(M.toleranceForLength(199.999),1,"lengths below 200 mm use ±1 mm");
assert.equal(M.toleranceForLength(200),2,"200–300 mm uses ±2 mm");
assert.equal(M.toleranceForLength(299.999),2,"lengths below 300 mm use ±2 mm");
assert.equal(M.toleranceForLength(300),3,"300–400 mm uses ±3 mm");
assert.equal(M.toleranceForLength(400),4,"the tolerance pattern continues by 100 mm bands");

const loads=Object.fromEntries(["axis1","axis2","axis3","point2","rod"].map(key=>[key,{positive:key==="rod"?10:1,negative:key==="rod"?10:1}]));
const expectedTolerance={j2Axis3:1,axis1Axis2:1,axis2Axis3:1,axis3Lower:3,point2Lower:1};
for(const [key,tolerance] of Object.entries(expectedTolerance)){
  const result=M.toleranceCurve(key,loads,1,9);
  assert.equal(result.tolerance,tolerance,`${key} tolerance band`);
  assert.equal(result.points.length,9,`${key} sample count`);
  assert.equal(result.points[0].delta,-tolerance,`${key} lower endpoint`);
  assert.equal(result.points.at(-1).delta,tolerance,`${key} upper endpoint`);
  assert.equal(result.points[4].delta,0,`${key} includes standard state`);
  assert.ok(result.points.every(point=>Number.isFinite(point.torque)&&point.peakAngle>=-10&&point.peakAngle<=10),`${key} returns finite peak torques`);
  const spread=Math.max(...result.points.map(point=>point.torque))-Math.min(...result.points.map(point=>point.torque));
  assert.ok(spread>1e-6,`${key} tolerance must affect peak equivalent startup torque`);
}

const fit=M.quadraticFit([
  {delta:-2,torque:6},
  {delta:-1,torque:3},
  {delta:0,torque:4},
  {delta:1,torque:9},
  {delta:2,torque:18},
]);
assert.ok(Math.abs(fit.a-2)<1e-10,"quadratic coefficient a");
assert.ok(Math.abs(fit.b-3)<1e-10,"quadratic coefficient b");
assert.ok(Math.abs(fit.c-4)<1e-10,"quadratic coefficient c");
assert.ok(Math.abs(fit.r2-1)<1e-10,"perfect quadratic fit has R²=1");

assert.match(fragment,/id="j2-tolerance-charts"/,"page exposes the tolerance analysis chart area");
console.log("J2 tolerance analysis regression test passed");
