import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createArmRuntime} from '../src/arm-runtime.mjs';

const model=JSON.parse(readFileSync(new URL('../data/arm/model.json',import.meta.url),'utf8'));
const reference=JSON.parse(readFileSync(new URL('../data/arm/pose_reference.json',import.meta.url),'utf8'));
const arm=createArmRuntime(model,reference);
const near=(actual,expected,tol=1e-6,label='value')=>assert.ok(Math.abs(actual-expected)<=tol,`${label}: ${actual} != ${expected}`);
const nearPoint=(actual,expected,tol,label)=>actual.forEach((value,index)=>near(value,expected[index],tol,`${label}[${index}]`));

const zero=arm.solveOutput('J0',0);
const zeroPoses=arm.bodyPoses(zero);
assert.equal(zeroPoses.complete,true);
assert.equal(Object.keys(zeroPoses.bodies).length,18);
near(Object.values(zeroPoses.bodies).reduce((sum,body)=>sum+body.massKg,0),3.7249532,1e-8,'total mass');

for(const joint of ['J0','J1','J2','J3','J4','J5','J6']){
  const moved=arm.solveOutput(joint,10);
  const poses=arm.bodyPoses(moved);
  assert.equal(poses.complete,true,`${joint} body poses`);
  assert.ok(Object.values(poses.bodies).some((body,index)=>{
    const initial=arm.body(arm.bodyNames[index]).comGlobalInitialMm;
    return Math.hypot(...body.comMm.map((v,i)=>v-initial[i]))>1e-5;
  }),`${joint} moves at least one COM`);
  const gravity=arm.gravityHoldingTorque(joint,0),inertia=arm.effectiveInertia(joint,0);
  assert.equal(gravity.complete,true);assert.ok(Number.isFinite(gravity.torqueNm));
  assert.equal(inertia.complete,true);assert.ok(inertia.kgM2>0&&Number.isFinite(inertia.kgM2));
  const trajectory=arm.trajectoryDynamics(joint,0,{direction:1});
  for(const value of [trajectory.velocityRadS,trajectory.accelerationRadS2,trajectory.gravityTorqueNm,trajectory.inertiaTorqueNm,trajectory.totalTorqueNm])assert.ok(Number.isFinite(value));
}

const elbowMinus20=arm.blackbox.elbowPitchToRod({deltaPitchDeg:-20}).rodStrokeMm;
const referenceState=arm.solveActive({shoulderRollDeg:20,elbowRodMm:elbowMinus20});
assert.equal(referenceState.complete,true);
const centers=arm.kinematics(referenceState).jointCentersMm;
for(const id of ['J0','J1','J2','J3','J4','J5','J6'])nearPoint(centers[id],reference.reference_joint_centers_mm[id].J1_20_J3_minus20_xyz,2e-6,id);

const unitLoads=Object.fromEntries(arm.rateKeys.map(key=>[key,{static:{positive:1,negative:1},dynamic:{positive:.7,negative:.7}}]));
for(const joint of ['J0','J1','J2','J3','J4','J5','J6']){
  const solution=arm.solveOutput(joint,0),result=arm.equivalentResistance(joint,solution,unitLoads,1,'static');
  assert.equal(result.complete,true);assert.ok(Number.isFinite(result.total)&&result.total>0);
  near(Object.values(result.contribution).reduce((a,b)=>a+b,0),100,1e-8,`${joint} contribution sum`);
  assert.ok(Object.values(result.rates).every(Number.isFinite));
  const cycle=arm.cycle(joint,unitLoads,{samples:21});
  assert.equal(cycle.complete,true,`${joint} cycle`);
  near(cycle.samples[0].output,cycle.samples.at(-1).output,1e-10,`${joint} cycle closure`);
  near(cycle.rankings.static.reduce((sum,item)=>sum+item.contribution,0),100,1e-8,`${joint} cycle contribution`);
}

console.log('arm dynamics runtime regression test passed');
