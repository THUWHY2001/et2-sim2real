import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

const context = {window: {}};
vm.createContext(context);
vm.runInContext(readFileSync("src/directional-friction.js", "utf8"), context);
vm.runInContext(readFileSync("src/arm-j3-model.js", "utf8"), context);

const M = context.window.ArmJ3Model;
const near = (actual, expected, tolerance=1e-5) => assert.ok(Math.abs(actual-expected) <= tolerance, `${actual} != ${expected}`);
const distance = (a,b) => Math.hypot(...a.map((v,i)=>v-b[i]));

const nominal = M.solve(0);
near(nominal.coordinates.rod, distance(M.nominal.Q,M.nominal.S), 1e-8);
near(M.lengths.pushrodReference, distance(M.nominal.Q,M.nominal.S), 1e-8);
assert.equal("T" in nominal.points, false, "pushrod has only Q(A) and S(B) endpoints");
for (let i=0;i<3;i++) {
  near(nominal.points.P[i], M.nominal.P[i]);
  near(nominal.points.Q[i], M.nominal.Q[i]);
  near(nominal.points.R[i], M.nominal.R[i]);
  near(nominal.points.S[i], M.nominal.S[i]);
}

for (const degrees of [-10,-5,0,5,10]) {
  const state = M.solve(degrees);
  near(distance(state.points.O,state.points.P), M.lengths.L, 1e-4);
  near(distance(state.points.P,state.points.Q), M.lengths.Z, 1e-4);
  near(distance(state.points.Q,state.points.R), M.lengths.K, 1e-4);
  near(distance(state.points.R,M.nominal.R), 0, 1e-8);
  near(distance(state.points.S,M.nominal.S), 0, 1e-8);
  near(state.coordinates.rod, distance(state.points.Q,state.points.S), 1e-8);
  for (const key of ["p","q","r","b","rod"]) assert.ok(Number.isFinite(state.rates[key]), `${key} rate is not finite at ${degrees}°`);
}

const staticLoads = Object.fromEntries(["p","q","r","b","rod"].map(key => [key,{positive:1,negative:3}]));
const positive = M.equivalentTorque(nominal, staticLoads, 1);
const negative = M.equivalentTorque(nominal, staticLoads, -1);
assert.ok(Math.abs(positive.total-negative.total) > 1e-4, "bidirectional static loads must change equivalent torque");

const runningLoads = Object.fromEntries(["p","q","r","b","rod"].map(key => [key,{
  positive:{coulomb:1,viscous:0.1},
  negative:{coulomb:3,viscous:0.2},
}]));
const forward = M.runningResistance(nominal, 1, runningLoads);
const reverse = M.runningResistance(nominal, -1, runningLoads);
assert.ok(Math.abs(forward.total-reverse.total) > 1e-4, "bidirectional running loads must change equivalent torque");
assert.equal(Math.sign(forward.signedTotal), -1);
assert.equal(Math.sign(reverse.signedTotal), 1);

console.log("arm J3 model regression test passed");
