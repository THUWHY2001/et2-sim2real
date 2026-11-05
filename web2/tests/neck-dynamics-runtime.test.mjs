import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createNeckRuntime} from '../src/neck-runtime.mjs';

const data=JSON.parse(readFileSync(new URL('../data/neck/model.json',import.meta.url),'utf8'));
const M=createNeckRuntime(data),close=(a,b,t=1e-8)=>assert.ok(Math.abs(a-b)<=t,`${a} != ${b}`);

const zero=M.solveInputs({leftMm:0,rightMm:0,yawDeg:0}),zeroPoses=M.bodyPoses(zero);
assert.equal(zeroPoses.complete,true);
assert.equal(Object.keys(zeroPoses.bodies).length,13);
close(Object.values(zeroPoses.bodies).reduce((sum,b)=>sum+b.massKg,0),2.8109963,1e-10);
for(const name of Object.keys(M.bodies)){
  const expected=M.body(name).com_world0_mm,actual=zeroPoses.bodies[name].comMm;
  expected.forEach((value,index)=>close(actual[index],value,1e-7));
}
const moved=M.bodyPoses(M.solveInputs({leftMm:1,rightMm:-1,yawDeg:7}));
assert.notDeepEqual(moved.bodies.head_camera_link1.comMm,zeroPoses.bodies.head_camera_link1.comMm);
assert.notDeepEqual(moved.bodies.neck_link5.comMm,zeroPoses.bodies.neck_link5.comMm);

const load=(s=1,d=.7)=>({static:{positive:s,negative:s},dynamic:{positive:d,negative:d}}),loads={};
for(const key of M.rateKeys)loads[key]=load(key.includes('Prismatic')?10:1,key.includes('Prismatic')?7:.7);
for(const joint of ['J0','J1','J2']){
  const solution=M.solveOutput(joint,4),rates=M.rates(joint,4);
  assert.equal(rates.complete,true,`${joint} rates`);
  assert.ok(Object.values(rates.values).every(Number.isFinite),`${joint} finite rates`);
  const base=M.equivalentResistance(joint,solution,loads,1,'static');
  assert.equal(base.complete,true);
  close(Object.values(base.contribution).reduce((a,b)=>a+b,0),100,1e-7);
  const dynamics=M.trajectoryDynamics(joint,4,{direction:1});
  assert.equal(dynamics.complete,true);
  assert.ok(dynamics.effectiveInertiaKgM2>0);
  for(const key of ['gravityTorqueNm','inertiaTorqueNm','totalTorqueNm'])assert.ok(Number.isFinite(dynamics[key]));
  const assumptions={coefficients:Object.fromEntries(M.rateKeys.map(key=>[key,.001])),preloads:Object.fromEntries(M.rateKeys.map(key=>[key,0]))};
  const loaded=M.loadedResistance(joint,solution,loads,1,'static',assumptions);
  assert.ok(loaded.loadedTotal>loaded.baseTotal);
  close(Object.values(loaded.items).reduce((sum,item)=>sum+item.loadedContribution,0),100,1e-7);
  const cycle=M.cycle(joint,loads,assumptions,{samples:41});
  assert.equal(cycle.complete,true,`${joint} cycle`);
  assert.ok(Math.max(...cycle.samples.map(sample=>sample.solution.residualMm))<1e-8);
}
close(M.gravityHoldingTorque('J2',4).torqueNm,0,1e-6);
console.log('neck dynamics runtime regression test passed');
