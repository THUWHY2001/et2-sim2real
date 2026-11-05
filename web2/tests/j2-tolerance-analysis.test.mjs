import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

const fragment=readFileSync("src/j2.fragment.html","utf8");
const source=fragment.match(/<script data-j2-core>([\s\S]*?)<\/script>/)?.[1];
assert.ok(source,"J2 core model script must exist");
const context={window:{}};
vm.createContext(context);
vm.runInContext(readFileSync("src/directional-friction.js","utf8"),context);
vm.runInContext(readFileSync("src/link-tolerance.js","utf8"),context);
vm.runInContext(source,context);
const M=context.window.J2Model;
const T=context.window.LinkTolerance;

const loads=Object.fromEntries(["axis1","axis2","axis3","point2","rod"].map(key=>[key,{positive:key==="rod"?10:1,negative:key==="rod"?10:1}]));
const expectedTolerance={j2Axis3:1,axis1Axis2:1,axis2Axis3:1,axis3Lower:3,point2Lower:1};
const results=T.analyze({model:M,links:Object.keys(expectedTolerance).map(key=>({key,label:key})),loads,direction:1,sampleCount:9});
for(const result of results){
  const {key}=result;
  const tolerance=expectedTolerance[result.key];
  assert.equal(result.tolerance,tolerance,`${key} tolerance band`);
  assert.equal(result.points.length,9,`${key} sample count`);
  assert.equal(result.points[0].delta,-tolerance,`${key} lower endpoint`);
  assert.equal(result.points.at(-1).delta,tolerance,`${key} upper endpoint`);
  assert.equal(result.points[4].delta,0,`${key} includes standard state`);
  assert.ok(result.points.every(point=>Number.isFinite(point.torque)&&point.peakAngle>=-10&&point.peakAngle<=10),`${key} returns finite peak torques`);
  const spread=Math.max(...result.points.map(point=>point.torque))-Math.min(...result.points.map(point=>point.torque));
  assert.ok(spread>1e-6,`${key} tolerance must affect peak equivalent startup torque`);
  assert.ok(Number.isFinite(result.sensitivity),`${key} standard-state sensitivity`);
  assert.ok(result.maxChangePercent>0,`${key} maximum percent change`);
}

assert.match(fragment,/id="j2-tolerance-charts"/,"page exposes the tolerance analysis chart area");
assert.match(fragment,/标准态灵敏度/,"page explains the standard-state sensitivity output");
console.log("J2 tolerance analysis regression test passed");
