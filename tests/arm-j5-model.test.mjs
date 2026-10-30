import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

const context={window:{}};
vm.createContext(context);
vm.runInContext(readFileSync("src/directional-friction.js","utf8"),context);
vm.runInContext(readFileSync("src/arm-j5-model.js","utf8"),context);

const M=context.window.ArmJ5Model;
const near=(actual,expected,tolerance=1e-5)=>assert.ok(Math.abs(actual-expected)<=tolerance,`${actual} != ${expected}`);

const nominal=M.solve(0);
near(M.lengths.link1,33.600,0.001);
near(M.lengths.link2,33.600,0.001);
near(nominal.coordinates.rod1,124.961275,0.00001);
near(nominal.coordinates.rod2,124.961468,0.00001);
near(nominal.rates.rod1,-16.430464,0.001);
near(nominal.rates.rod2,-16.427284,0.001);
near(nominal.rates.upper1,0.037880,0.0001);
near(nominal.rates.upper2,0.037885,0.0001);
near(nominal.rates.lower1,0.962120,0.0001);
near(nominal.rates.lower2,0.962115,0.0001);

for(const [degrees,expected] of [[-10,127.747],[0,124.961],[10,122.038]]){
  const state=M.solve(degrees);
  near(state.coordinates.rod1,expected,0.001);
  near(state.coordinates.rod2,expected,0.0015);
  for(const key of ["upper1","lower1","upper2","lower2","rod1","rod2"])
    assert.ok(Number.isFinite(state.rates[key]),`${key} rate is not finite at ${degrees}°`);
}

const staticLoads={
  upper1:{positive:1,negative:1},lower1:{positive:1,negative:1},
  upper2:{positive:1,negative:1},lower2:{positive:1,negative:1},
  rod1:{positive:10,negative:10},rod2:{positive:10,negative:10},
};
const staticResult=M.equivalentTorque(nominal,staticLoads,1);
near(staticResult.total,2.328577,0.00001);
assert.deepEqual(Object.keys(staticResult.parts),["upper1","lower1","upper2","lower2","rod1","rod2"]);

const runningLoads=Object.fromEntries(Object.keys(staticLoads).map(key=>[key,{
  positive:{coulomb:key.startsWith("rod")?10:1,viscous:0.1},
  negative:{coulomb:key.startsWith("rod")?12:2,viscous:0.2},
}]));
for(const qdot of [1,0,-1]){
  const result=M.runningResistance(nominal,qdot,runningLoads);
  assert.ok(Number.isFinite(result.total));
  assert.ok(Number.isFinite(result.power));
}

console.log("arm J5 model regression test passed");
