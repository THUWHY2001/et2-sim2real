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

console.log("neck tolerance geometry regression test passed");
