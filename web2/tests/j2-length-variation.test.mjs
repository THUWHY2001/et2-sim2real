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
const distance=(a,b)=>Math.hypot(a.x-b.x,a.z-b.z);
const near=(actual,expected,tolerance=1e-6)=>assert.ok(Math.abs(actual-expected)<=tolerance,`${actual} != ${expected}`);

const standard=M.solve(0);
for(const key of ["j2Axis3","axis1Axis2","axis2Axis3","axis3Lower","point2Lower"]) assert.ok(M.lengths[key]>0,`${key} standard length missing`);
near(distance(standard.j2,standard.axis3),M.lengths.j2Axis3);
near(distance(standard.axis1,standard.axis2),M.lengths.axis1Axis2);
near(distance(standard.axis2,standard.axis3),M.lengths.axis2Axis3);
near(distance(standard.axis3,standard.lower),M.lengths.axis3Lower);
near(distance(standard.point2,standard.lower),M.lengths.point2Lower);

for(const key of Object.keys(M.lengths)){
  const varied={...M.lengths,[key]:M.lengths[key]+1};
  const state=M.solve(0,varied);
  const actual={
    j2Axis3:distance(state.j2,state.axis3),
    axis1Axis2:distance(state.axis1,state.axis2),
    axis2Axis3:distance(state.axis2,state.axis3),
    axis3Lower:distance(state.axis3,state.lower),
    point2Lower:distance(state.point2,state.lower),
  }[key];
  near(actual,varied[key],1e-5);
  for(const rate of Object.values(state.rates)) assert.ok(Number.isFinite(rate));
  assert.ok(Number.isFinite(state.rodJacobian));
}

const loads=Object.fromEntries(["axis1","axis2","axis3","point2","rod"].map(key=>[key,{positive:key==="rod"?10:1,negative:key==="rod"?10:1}]));
const standardTorque=M.equivalentTorque(standard,loads,1).total;
for(const key of Object.keys(M.lengths)){
  const varied={...M.lengths,[key]:M.lengths[key]+5};
  const torque=M.equivalentTorque(M.solve(0,varied),loads,1).total;
  assert.ok(Math.abs(torque-standardTorque)>1e-5,`${key} must affect equivalent torque contribution result`);
}

assert.throws(()=>M.solve(0,{...M.lengths,axis1Axis2:1}),/闭环|closure/);
console.log("J2 link-length variation regression test passed");
