import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

const context={window:{}};
vm.createContext(context);
vm.runInContext(readFileSync("src/directional-friction.js","utf8"),context);
vm.runInContext(readFileSync("src/link-tolerance.js","utf8"),context);
vm.runInContext(readFileSync("src/arm-j3-model.js","utf8"),context);
const M=context.window.ArmJ3Model,T=context.window.LinkTolerance;
const distance=(a,b)=>Math.hypot(...a.map((value,index)=>value-b[index]));
for(const key of ["L","Z","K"]){
  const state=M.solve(0,{[key]:M.lengths[key]+1});
  const endpoints={L:[state.points.O,state.points.P],Z:[state.points.P,state.points.Q],K:[state.points.Q,state.points.R]}[key];
  assert.ok(Math.abs(distance(...endpoints)-(M.lengths[key]+1))<1e-5,`${key} override changes the physical link`);
  assert.ok(Object.values(state.rates).every(Number.isFinite),`${key} keeps finite spatial kinematics`);
  assert.deepEqual(state.points.R,M.nominal.R,"R remains fixed in space");
  assert.deepEqual(state.points.S,M.nominal.S,"pushrod B end remains fixed in space");
}
assert.throws(()=>M.solve(0,{K:1}),/closure|闭环/i,"invalid arm closure must be rejected");
const loads=Object.fromEntries(["p","q","r","b","rod"].map(key=>[key,{positive:key==="rod"?10:1,negative:key==="rod"?10:1}]));
const results=T.analyze({model:M,links:["L","Z","K"].map(key=>({key,label:key})),loads,direction:1,sampleCount:7});
assert.equal(results.length,3,"all three non-pushrod arm links are analyzed");
assert.ok(results.every(result=>Number.isFinite(result.sensitivity)&&result.maxChangePercent>0),"each arm link yields a usable tolerance curve");
const fragment=readFileSync("src/arm-j3.fragment.html","utf8");
assert.match(fragment,/id="arm-j3-tolerance-charts"/,"arm J3 page exposes tolerance charts");
assert.doesNotMatch(fragment,/key:\s*['"]pushrodReference/i,"pushrod is excluded from link tolerance variables");
console.log("arm J3 tolerance analysis regression test passed");
