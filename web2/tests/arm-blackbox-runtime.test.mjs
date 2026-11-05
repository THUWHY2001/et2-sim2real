import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createArmRuntime} from '../src/arm-runtime.mjs';

const model=JSON.parse(readFileSync(new URL('../data/arm/model.json',import.meta.url),'utf8'));
const poseReference=JSON.parse(readFileSync(new URL('../data/arm/pose_reference.json',import.meta.url),'utf8'));
const arm=createArmRuntime(model,poseReference);

const validation=arm.validate();
assert.equal(validation.ok,true);
assert.equal(validation.bodyCount,18);
assert.equal(validation.massPropertiesComplete,18);
assert.equal(validation.totalMassKg,3.7249532);
assert.deepEqual(validation.gravityMps2,[0,0,-9.80665]);
assert.deepEqual(validation.excludedInterfaces.sort(),['Hand_L','waist_link3_3_shoulder_L'].sort());

const summary=arm.summary();
assert.equal(summary.bodyCount,18);
assert.equal(summary.totalMassKg,3.7249532);
assert.deepEqual(summary.joints.map(x=>x.id),['J0','J1','J2','J3','J4','J5','J6']);
assert.equal(summary.joints.find(x=>x.id==='J3').driverType,'singleRod');
assert.equal(summary.joints.find(x=>x.id==='J5').driverType,'dualRod');
assert.equal(summary.joints.find(x=>x.id==='J6').driverType,'dualRod');
for(const id of ['J0','J1','J2','J4'])assert.equal(summary.joints.find(x=>x.id===id).driverType,'revolute');

for(const joint of ['J0','J1','J2','J3','J4','J5','J6'])for(const degrees of [-10,10]){
  const solution=arm.solveOutput(joint,degrees);
  assert.equal(solution.complete,true,`${joint} ${degrees}° must solve`);
  assert.ok(solution.residualMm<1e-6,`${joint} ${degrees}° must round-trip`);
  assert.ok(Math.abs(solution.outputs[joint]-degrees)<1e-5,`${joint} output must match target`);
}

const elbow=arm.solveOutput('J3',10);
assert.ok(Number.isFinite(elbow.activeInputs.elbowRodMm));
assert.equal(arm.solveActive({elbowRodMm:elbow.activeInputs.elbowRodMm}).complete,true);
const wrist=arm.solveOutput('J5',10);
assert.ok(Number.isFinite(wrist.activeInputs.wristLeftMm));
assert.ok(Number.isFinite(wrist.activeInputs.wristRightMm));
assert.equal(arm.solveActive({wristLeftMm:wrist.activeInputs.wristLeftMm,wristRightMm:wrist.activeInputs.wristRightMm}).complete,true);

assert.equal(arm.bodyNames.includes('Hand_L'),false);
assert.equal(arm.bodyNames.includes('waist_link3_3_shoulder_L'),false);
assert.equal(arm.bodyNames.length,18);
console.log('arm blackbox runtime acceptance test passed');
