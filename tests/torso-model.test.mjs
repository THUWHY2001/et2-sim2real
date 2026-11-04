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

console.log("torso source geometry regression test passed");
