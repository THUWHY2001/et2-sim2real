import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

const window={};
vm.runInNewContext(readFileSync(new URL("../src/neck-model.js",import.meta.url),"utf8"),{window,Math});
vm.runInNewContext(readFileSync(new URL("../src/neck-tolerance-model.js",import.meta.url),"utf8"),{window,Math});
const M=window.NeckModel,T=window.NeckToleranceModel;

const zero=Object.fromEntries(T.pointKeys.map(key=>[key,{x:0,y:0,z:0}]));
assert.equal(T.validateOffsets(zero).valid,true);
assert.equal(T.validateOffsets({...zero,R1:{x:1,y:0,z:0}}).valid,true);
assert.equal(T.validateOffsets({...zero,R1:{x:1,y:1,z:0}}).valid,false);

const projected=T.projectOffset({x:1,y:1,z:0});
assert.ok(Math.abs(Math.hypot(projected.x,projected.y,projected.z)-1)<1e-12);
const previous={...zero,R1:{x:.4,y:.2,z:0}};
const rejected=T.applyOffset(previous,"R1",{x:1,y:1,z:0});
assert.equal(rejected.accepted,false);
assert.deepEqual({...rejected.offsets.R1},previous.R1);
assert.notEqual(rejected.offsets,previous,"offset collection must be immutable");

for(const joint of ["J0","J1"]){
  const nominal=T.createState(joint,5,zero);
  const legacy=M.solve(joint,5);
  assert.ok(Math.abs(nominal.lengths.rod1-legacy.lengths.rod1)<1e-9);
  assert.ok(Math.abs(nominal.lengths.rod2-legacy.lengths.rod2)<1e-9);
  assert.deepEqual({...nominal.axis.a},{...legacy.axis.a});
  assert.deepEqual({...nominal.axis.u},{...legacy.axis.u});
}

const moved=T.createState("J0",5,{...zero,R1:{x:0,y:1,z:0},R2:{x:0,y:0,z:1}});
const standard=T.createState("J0",5,zero);
assert.deepEqual({...moved.axis.a},{...standard.axis.a},"hardpoint error must not rebuild the nominal axis");
assert.deepEqual({...moved.axis.u},{...standard.axis.u},"hardpoint error must not tilt the nominal axis");
assert.ok(moved.radialResiduals.R1>.99);
assert.ok(moved.radialResiduals.R2>.99);

const reset=T.applyOffset(previous,"R1",{x:0,y:0,z:0});
assert.equal(reset.accepted,true);
assert.deepEqual({...reset.offsets.R1},{x:0,y:0,z:0});
assert.deepEqual(
  {...T.createState("J1",0,reset.offsets).points.R1},
  {...M.points.R1},
  "reset must restore the published standard point",
);
assert.deepEqual({...M.points.R1},{x:-52,y:0,z:479.66},"tolerance analysis must not mutate NeckModel.points");

const sideParameters={radialStiffness:100,clearancePositive:.1,clearanceNegative:.05};
assert.equal(T.sideLoad(.05,sideParameters,1),0);
assert.ok(Math.abs(T.sideLoad(.2,sideParameters,1)-10)<1e-12);
assert.ok(Math.abs(T.sideLoad(.2,sideParameters,-1)-15)<1e-12);

const loads={};
for(const key of [...M.keys.J0.rotary,...M.keys.J0.rods])loads[key]={
  static:{positive:key.startsWith('rod')?10:1,negative:key.startsWith('rod')?11:1.2},
  dynamic:{positive:key.startsWith('rod')?7:.7,negative:key.startsWith('rod')?8:.8},
};
const parameters=T.createDefaultParameters();
parameters.points.R1={...parameters.points.R1,radialStiffness:100,clearancePositive:0,clearanceNegative:0,staticLoadCoefficient:.01,dynamicLoadCoefficient:.005};
const r1Offset={...zero,R1:{x:0,y:1,z:0}};
const offsetState=T.createState("J0",0,r1Offset);
const staticResistance=T.equivalentResistance("J0",offsetState,parameters,loads,1,"static");
const dynamicResistance=T.equivalentResistance("J0",offsetState,parameters,loads,1,"dynamic");
assert.ok(staticResistance.sideLoads.R1>99.9);
assert.ok(staticResistance.parts.R1>dynamicResistance.parts.R1,"static and dynamic load coefficients must remain independent");
assert.ok(Number.isFinite(staticResistance.frictionTotal));
assert.ok(Number.isFinite(staticResistance.driveTorque));

const commonRodShift={...zero,lower1:{x:0,y:1,z:0},upper1:{x:0,y:1,z:0}};
const commonState=T.createState("J0",0,commonRodShift);
const commonResistance=T.equivalentResistance("J0",commonState,parameters,loads,1,"static");
assert.ok(commonResistance.sideLoads.lower1<1e-9&&commonResistance.sideLoads.upper1<1e-9,"common endpoint translation must not create duplicate rod side load");

const calibratedOffset={...zero,lower1:{x:.5,y:0,z:0}};
const calibratedZero=T.createState("J0",0,calibratedOffset);
assert.ok(Math.abs(calibratedZero.lengthErrors.rod1)<1e-12,"pushrod axial tolerance preload must reset to zero after calibration");
const calibratedFive=T.createState("J0",5,calibratedOffset);
const actualZero=T.createState("J0",0,calibratedOffset).lengths.rod1;
const nominalZero=M.solve("J0",0).lengths.rod1;
const expectedStrokeError=(calibratedFive.lengths.rod1-actualZero)-(M.solve("J0",5).lengths.rod1-nominalZero);
assert.ok(Math.abs(calibratedFive.lengthErrors.rod1-expectedStrokeError)<1e-9,"axial load must use stroke error relative to the calibrated zero length");

const searchOptions={directions:[{x:1,y:0,z:0},{x:-1,y:0,z:0},{x:0,y:1,z:0},{x:0,y:-1,z:0},{x:0,y:0,z:1},{x:0,y:0,z:-1}],steps:12,refine:false};
const single=T.rankContributions("J0",parameters,loads,searchOptions);
assert.equal(single.static.items.length,8);
assert.equal(single.dynamic.items.length,8);
assert.ok(Math.abs(single.static.items.reduce((sum,item)=>sum+item.contribution,0)-100)<1e-8);
assert.ok(Math.abs(single.dynamic.items.reduce((sum,item)=>sum+item.contribution,0)-100)<1e-8);
for(const item of [...single.static.items,...single.dynamic.items])assert.ok(Math.hypot(item.direction.x,item.direction.y,item.direction.z)<=1+1e-12);

const pairs=T.rankPairContributions("J0",parameters,loads,searchOptions);
assert.equal(pairs.static.items.length,28);
assert.equal(pairs.dynamic.items.length,28);
assert.equal(new Set(pairs.static.items.map(item=>item.points.slice().sort().join('|'))).size,28);
assert.ok(Math.abs(pairs.static.items.reduce((sum,item)=>sum+item.contribution,0)-100)<1e-8);
for(const item of pairs.static.items){
  assert.ok(Math.hypot(item.directions[0].x,item.directions[0].y,item.directions[0].z)<=1+1e-12);
  assert.ok(Math.hypot(item.directions[1].x,item.directions[1].y,item.directions[1].z)<=1+1e-12);
}
const refinedPairs=T.rankPairContributions("J0",parameters,loads,{steps:8});
assert.ok(refinedPairs.static.items.some(item=>item.directions.some(direction=>[direction.x,direction.y,direction.z].filter(value=>Math.abs(value)>.05).length>=2)),"default pair search must refine beyond six cardinal directions");
const zeroParameters=T.createDefaultParameters();
for(const point of T.pointKeys)zeroParameters.points[point]={radialStiffness:0,clearancePositive:0,clearanceNegative:0,staticLoadCoefficient:0,dynamicLoadCoefficient:0};
for(const rod of ['rod1','rod2'])zeroParameters.rods[rod]={axialStiffness:0,staticLoadCoefficient:0,dynamicLoadCoefficient:0};
const zeroRanking=T.rankContributions("J1",zeroParameters,{},searchOptions);
assert.ok(zeroRanking.static.items.every(item=>item.contribution===0&&Number.isFinite(item.impact)));
assert.ok(zeroRanking.dynamic.items.every(item=>item.contribution===0&&Number.isFinite(item.impact)));

const robustOptions={samples:4,seed:5913,directions:searchOptions.directions,steps:8,refine:false};
const robustA=T.robustRanking("J0",parameters,loads,{lowerFactor:.5,upperFactor:2},robustOptions);
const robustB=T.robustRanking("J0",parameters,loads,{lowerFactor:.5,upperFactor:2},robustOptions);
assert.equal(JSON.stringify(robustA),JSON.stringify(robustB),"same seed must reproduce robust ranking");
for(const mode of ['static','dynamic'])for(const item of robustA[mode].items){
  assert.ok(item.low<=item.high);
  assert.ok(item.firstProbability>=0&&item.firstProbability<=1);
  assert.ok(item.topThreeProbability>=0&&item.topThreeProbability<=1);
  assert.ok(item.rankStability>=0&&item.rankStability<=1);
}
const collapsed=T.robustRanking("J0",parameters,loads,{lowerFactor:1,upperFactor:1},{...robustOptions,samples:2});
const nominal=T.rankContributions("J0",parameters,loads,{...searchOptions,steps:8});
for(const mode of ['static','dynamic'])for(const item of collapsed[mode].items){
  const expected=nominal[mode].items.find(entry=>entry.point===item.point).contribution;
  assert.ok(Math.abs(item.low-expected)<1e-9&&Math.abs(item.high-expected)<1e-9);
}

console.log("neck tolerance model regression test passed");
