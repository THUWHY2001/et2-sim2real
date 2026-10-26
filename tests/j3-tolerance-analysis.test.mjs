import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

const fragment=readFileSync("src/j3.fragment.html","utf8");
const source=fragment.match(/<script data-j3-core>([\s\S]*?)<\/script>/)?.[1];
assert.ok(source,"J3 core model script must exist");
const context={window:{}};
vm.createContext(context);
vm.runInContext(readFileSync("src/directional-friction.js","utf8"),context);
vm.runInContext(readFileSync("src/link-tolerance.js","utf8"),context);
vm.runInContext(source,context);
const M=context.window.J3Model,T=context.window.LinkTolerance;

for(const key of ["JP","PA","AQ","JQ"]){
  const state=M.solve(0,{[key]:M.lengths[key]+1});
  const endpoints={JP:[state.J,state.P],PA:[state.P,state.A],AQ:[state.A,state.Q],JQ:[state.J,state.Q]}[key];
  assert.ok(Math.abs(M.distance(...endpoints)-(M.lengths[key]+1))<1e-6,`${key} override changes the physical link`);
  assert.ok(Number.isFinite(state.rodJacobian)&&Object.values(state.rates).every(Number.isFinite),`${key} keeps a finite Jacobian`);
}
assert.throws(()=>M.solve(0,{PA:1}),/闭环/,"invalid J3 closure must be rejected");
const loads=Object.fromEntries(["p","a","q","rod"].map(key=>[key,{positive:key==="rod"?10:1,negative:key==="rod"?10:1}]));
const results=T.analyze({model:M,links:["JP","PA","AQ","JQ"].map(key=>({key,label:key})),loads,direction:1,sampleCount:7});
assert.equal(results.length,4,"all four non-pushrod links are analyzed");
assert.ok(results.every(result=>result.points.length>=3&&Number.isFinite(result.sensitivity)&&result.maxChangePercent>0),"each J3 link yields a usable tolerance curve");
assert.match(fragment,/id="j3-tolerance-charts"/,"J3 page exposes tolerance charts");
assert.doesNotMatch(fragment,/key:\s*['"]rod/i,"pushrod is not a tolerance variable");
console.log("J3 tolerance analysis regression test passed");
