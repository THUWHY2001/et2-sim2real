import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

const source=readFileSync(new URL("../src/torso-model.js",import.meta.url),"utf8");
const window={};
vm.runInNewContext(source,{window,Math});
const M=window.TorsoModel;

const close=(actual,expected,tolerance=1e-9,message="")=>assert.ok(Math.abs(actual-expected)<=tolerance,`${message} expected ${expected}, got ${actual}`);
const point=(actual,expected,name)=>{
  close(actual.x,expected[0],1e-9,`${name}.x`);
  close(actual.y,expected[1],1e-9,`${name}.y`);
  close(actual.z,expected[2],1e-9,`${name}.z`);
};

point(M.points.yawPivot,[-29.2574,.0001,-24.0598],"yawPivot");
point(M.points.outputPivot,[-29.255,0,30.8402],"outputPivot");
point(M.points.crankPivotL,[-29.255,22.6,97.8402],"crankPivotL");
point(M.points.crankPivotR,[-29.255,-22.6,97.8402],"crankPivotR");
point(M.points.upperBallL,[-68.4219,13.35,109.0301],"upperBallL");
point(M.points.upperBallR,[-68.4219,-13.35,109.0301],"upperBallR");
point(M.points.lowerBallL,[-73.0189,25.7497,40.2936],"lowerBallL");
point(M.points.lowerBallR,[-73.0189,-25.7497,40.2936],"lowerBallR");

close(M.lengths.crankL,41.77107154024182,1e-9,"left crank length");
close(M.lengths.crankR,41.77107154024182,1e-9,"right crank length");
close(M.lengths.rodL,69.99708137729745,1e-9,"left rod length");
close(M.lengths.rodR,69.99708137729745,1e-9,"right rod length");

const zero=M.zeroState();
assert.deepEqual({...zero},{yaw:0,roll:0,pitch:0,crankL:0,crankR:0});
const geometry=M.worldGeometry(zero);
for(const key of ["crankPivotL","crankPivotR","upperBallL","upperBallR","lowerBallL","lowerBallR"]){
  close(geometry.points[key].x,M.points[key].x,1e-9,`${key} zero x`);
  close(geometry.points[key].y,M.points[key].y,1e-9,`${key} zero y`);
  close(geometry.points[key].z,M.points[key].z,1e-9,`${key} zero z`);
}
close(geometry.rodLengths.L,M.lengths.rodL,1e-9,"zero left rod");
close(geometry.rodLengths.R,M.lengths.rodR,1e-9,"zero right rod");
close(geometry.rodErrors.L,0,1e-9,"zero left error");
close(geometry.rodErrors.R,0,1e-9,"zero right error");
close(geometry.closureResiduals.L,0,1e-6,"zero left residual");
close(geometry.closureResiduals.R,0,1e-6,"zero right residual");

for(const degrees of [-10,0,10]){
  const j0=M.solveJoint("J0",degrees);
  assert.equal(j0.converged,true,`J0 ${degrees}° must converge`);
  close(j0.state.yaw,degrees,1e-12,"J0 yaw output");
  close(j0.state.roll,0,1e-12,"J0 roll remains zero");
  close(j0.state.pitch,0,1e-12,"J0 pitch remains zero");
  close(j0.state.crankL,0,1e-12,"J0 left crank remains zero");
  close(j0.state.crankR,0,1e-12,"J0 right crank remains zero");

  const j1=M.solveJoint("J1",degrees);
  assert.equal(j1.converged,true,`J1 ${degrees}° must converge`);
  close(j1.state.roll,degrees,1e-8,"J1 roll output");
  close(j1.state.crankR,-j1.state.crankL,1e-8,"J1 opposite cranks");
  assert.ok(Math.max(Math.abs(j1.geometry.rodErrors.L),Math.abs(j1.geometry.rodErrors.R))<1e-6,"J1 rods must stay fixed");

  const j2=M.solveJoint("J2",degrees);
  assert.equal(j2.converged,true,`J2 ${degrees}° must converge`);
  close(j2.state.pitch,degrees,1e-8,"J2 pitch output");
  close(j2.state.crankR,j2.state.crankL,1e-8,"J2 equal cranks");
  assert.ok(Math.max(Math.abs(j2.geometry.rodErrors.L),Math.abs(j2.geometry.rodErrors.R))<1e-6,"J2 rods must stay fixed");
}

for(const joint of ["J1","J2"]){
  let previous=null;
  const outputs=[0,2,4,6,8,10,8,6,4,2,0,-2,-4,-6,-8,-10,-8,-6,-4,-2,0];
  for(const output of outputs){
    const solved=M.solveJoint(joint,output,previous);
    assert.equal(solved.converged,true,`${joint} branch sweep must converge at ${output}°`);
    if(previous){
      assert.ok(Math.abs(solved.state.crankL-previous.crankL)<4,`${joint} left crank must stay on one branch`);
      assert.ok(Math.abs(solved.state.crankR-previous.crankR)<4,`${joint} right crank must stay on one branch`);
    }
    previous=solved.state;
  }
}

const small=.01;
const smallJ1=M.solveJoint("J1",small).state;
const smallJ2=M.solveJoint("J2",small).state;
close(smallJ1.crankL/small,-.6878331741392661,5e-4,"J1 left normalized initial slope");
close(smallJ1.crankR/small,.6878331741392661,5e-4,"J1 right normalized initial slope");
close(smallJ2.crankL/small,-1.1122594417717997,5e-4,"J2 left normalized initial slope");
close(smallJ2.crankR/small,-1.1122594417717997,5e-4,"J2 right normalized initial slope");

const forward=M.solveActive({yaw:3,crankL:smallJ1.crankL,crankR:smallJ1.crankR});
assert.equal(forward.converged,true,"forward active-input solve must converge");
close(forward.state.yaw,3,1e-12,"forward yaw passthrough");
close(forward.state.roll,small,1e-6,"forward solve recovers roll");
close(forward.state.pitch,smallJ1.pitch,1e-6,"forward solve recovers companion pitch");

const invalid=M.solveActive({yaw:0,crankL:Number.NaN,crankR:0});
assert.equal(invalid.converged,false,"invalid active input must fail explicitly");
assert.equal(invalid.geometry,null,"invalid active input must not fabricate geometry");

const rateKeys=["yaw","roll","pitch","crankL","crankR","ballLUpper","ballLLower","ballRUpper","ballRLower"];
const ratesJ0=M.rates("J0",0);
assert.equal(ratesJ0.converged,true,"J0 rates must solve");
close(ratesJ0.values.yaw,1,1e-9,"J0 yaw rate");
for(const key of rateKeys.filter(key=>key!=="yaw"))close(ratesJ0.values[key],0,1e-7,`J0 ${key} relative rate`);
const ratesJ1=M.rates("J1",0),ratesJ2=M.rates("J2",0);
assert.equal(ratesJ1.converged,true,"J1 rates must solve");
assert.equal(ratesJ2.converged,true,"J2 rates must solve");
close(ratesJ1.values.roll,1,1e-7,"J1 roll rate");
close(ratesJ1.values.crankL,-.6878331741392661,5e-5,"J1 left crank rate");
close(ratesJ1.values.crankR,.6878331741392661,5e-5,"J1 right crank rate");
close(ratesJ2.values.pitch,1,1e-7,"J2 pitch rate");
close(ratesJ2.values.crankL,-1.1122594417717997,5e-5,"J2 left crank rate");
close(ratesJ2.values.crankR,-1.1122594417717997,5e-5,"J2 right crank rate");
for(const rates of [ratesJ1,ratesJ2])for(const key of rateKeys){assert.ok(Number.isFinite(rates.values[key]),`${key} rate must be finite`);if(key.startsWith("ball"))assert.ok(rates.values[key]>=0,`${key} swing rate must be nonnegative`)}

const load=(sPos,sNeg,dPos,dNeg)=>({static:{positive:sPos,negative:sNeg},dynamic:{positive:dPos,negative:dNeg}});
const loads={};
rateKeys.forEach((key,index)=>loads[key]=load(index+1,index+1.5,(index+1)*.6,(index+1)*.8));
const j0Positive=M.equivalentResistance("J0",M.solveJoint("J0",0),loads,1,"static");
const j0Negative=M.equivalentResistance("J0",M.solveJoint("J0",0),loads,-1,"static");
close(j0Positive.total,1,1e-8,"J0 positive static total");
close(j0Negative.total,1.5,1e-8,"J0 negative static total");
close(j0Positive.contribution.yaw,100,1e-8,"J0 yaw contribution");
for(const joint of ["J1","J2"]){
  const solution=M.solveJoint(joint,3);
  const result=M.equivalentResistance(joint,solution,loads,1,"dynamic");
  assert.ok(result.total>0,`${joint} dynamic resistance must be positive`);
  close(result.total,Object.values(result.parts).reduce((sum,value)=>sum+value,0),1e-10,`${joint} parts sum`);
  close(Object.values(result.contribution).reduce((sum,value)=>sum+value,0),100,1e-8,`${joint} contribution sum`);
}

for(const joint of ["J1","J2"]){
  const cycle=M.cycle(joint,loads,{samples:121});
  assert.equal(cycle.complete,true,`${joint} cycle must complete`);
  assert.equal(cycle.samples.length,121,`${joint} cycle sample count`);
  close(Math.max(...cycle.samples.map(sample=>sample.output)),10,1e-9,`${joint} positive output peak`);
  close(Math.min(...cycle.samples.map(sample=>sample.output)),-10,1e-9,`${joint} negative output peak`);
  assert.ok(Math.max(...cycle.samples.flatMap(sample=>[Math.abs(sample.solution.geometry.rodErrors.L),Math.abs(sample.solution.geometry.rodErrors.R)]))<1e-6,`${joint} cycle closure`);
  close(cycle.rankings.static.reduce((sum,item)=>sum+item.contribution,0),100,1e-8,`${joint} static cycle contribution`);
  close(cycle.rankings.dynamic.reduce((sum,item)=>sum+item.contribution,0),100,1e-8,`${joint} dynamic cycle contribution`);
}

console.log("torso source geometry regression test passed");
