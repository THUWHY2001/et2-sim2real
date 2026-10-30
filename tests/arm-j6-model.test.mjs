import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

const context={window:{}};
vm.createContext(context);
vm.runInContext(readFileSync("src/directional-friction.js","utf8"),context);
vm.runInContext(readFileSync("src/arm-j6-model.js","utf8"),context);

const M=context.window.ArmJ6Model,dist=(a,b)=>Math.hypot(...a.map((v,i)=>v-b[i]));
assert.deepEqual(Array.from(M.nominal.axisA),[25.282,328.524,-106.969]);
assert.deepEqual(Array.from(M.nominal.axisB),[25.869,355.606,-96.874]);
const zero=M.solve(0),plus=M.solve(10),minus=M.solve(-10);
assert.ok(zero.rates.rod1*zero.rates.rod2<0,"J6 pitch must extend one rod while retracting the other");
assert.ok((plus.coordinates.rod1-zero.coordinates.rod1)*(plus.coordinates.rod2-zero.coordinates.rod2)<0);
assert.ok((minus.coordinates.rod1-zero.coordinates.rod1)*(minus.coordinates.rod2-zero.coordinates.rod2)<0);
for(const degrees of [-10,0,10]){
  const s=M.solve(degrees);
  assert.ok(Math.abs(dist(s.points.lower1,s.points.axisA)-dist(zero.points.lower1,zero.points.axisA))<1e-9);
  assert.ok(Math.abs(dist(s.points.lower2,s.points.axisB)-dist(zero.points.lower2,zero.points.axisB))<1e-9);
  for(const key of ["upper1","lower1","upper2","lower2","rod1","rod2"])
    assert.ok(Number.isFinite(s.rates[key]),`${key} rate is finite at ${degrees}°`);
}
console.log("arm J6 model regression test passed");
