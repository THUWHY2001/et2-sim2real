import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

const context={window:{}};
vm.createContext(context);
vm.runInContext(readFileSync("src/link-tolerance.js","utf8"),context);
const T=context.window.LinkTolerance;

for(const [length,want] of [[50,1],[99.999,1],[100,1],[199.999,1],[200,2],[299.999,2],[300,3],[399.999,3],[400,4]]){
  assert.equal(T.toleranceForLength(length),want,`${length} mm tolerance`);
}
assert.throws(()=>T.toleranceForLength(0),/大于 0/);

const fit=T.quadraticFit([
  {delta:-2,torque:6},{delta:-1,torque:3},{delta:0,torque:4},{delta:1,torque:9},{delta:2,torque:18},
]);
assert.ok(Math.abs(fit.a-2)<1e-10);
assert.ok(Math.abs(fit.b-3)<1e-10);
assert.ok(Math.abs(fit.c-4)<1e-10);
assert.ok(Math.abs(fit.r2-1)<1e-10);
assert.equal(T.formatDecimal(0.000418,6),"0.000418");
assert.equal(T.formatDecimal(2.52,6),"2.520000");
assert.doesNotMatch(T.formatDecimal(0.000000418,9),/[eE]/,"engineering output must not use scientific notation");

const model={
  lengths:{link:80},
  solve(angle,overrides){const length=overrides.link;if(length>80.8&&angle>9.9)throw new Error("closure");return{angle,length}},
  equivalentTorque(state){return{total:2+0.02*(state.length-80)+0.0005*(state.length-80)**2+0.001*state.angle**2}},
};
const result=T.analyze({model,links:[{key:"link",label:"测试杆"}],loads:{},direction:1,sampleCount:11})[0];
assert.equal(result.tolerance,1);
assert.equal(result.points.length,10,"infeasible tolerance samples are excluded");
assert.equal(result.invalidCount,1);
assert.ok(Math.abs(result.standard.torque-2.1)<1e-10,"standard point uses ±10° peak");
assert.ok(Math.abs(result.sensitivity-0.02)<1e-6,"standard-state slope is dτ/dL");
assert.ok(result.maxChangePercent>0);
assert.ok(result.points.every(point=>Number.isFinite(point.torque)));

console.log("shared link tolerance regression test passed");
