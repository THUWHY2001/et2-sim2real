import assert from "node:assert/strict";
import fs from "node:fs";
import {createLegFullRuntime} from "../src/leg-full-runtime.mjs";
import {createLegGravityStatics} from "../src/leg-gravity-statics.mjs";

const model=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8"));
const runtime=createLegFullRuntime(model),statics=createLegGravityStatics(model,runtime);
const boundary={
  worldInterface:"J_LEG_L_HIP_roll_RE",footContact:"none",
  heldRevolutes:["J_LEG_L_HIP_roll_RE","J_LEG_L_HIP_yaw_RE","J_LEG_L_ANKLE_roll_RE","J_LEG_L_TOE_pitch_RE"],
  toeConstraint:"position-held-unverified",loadCase:"gravity-quasi-static",
};
const result=statics.solveNominal(boundary);

assert.deepEqual(result.pose.output_degrees,{J2:-27.5,J3:51.57,J4:-24.06});
assert.equal(result.matrix.consistent,true);
assert.equal(result.matrix.svd.converged,true);
assert.ok(Number.isInteger(result.matrix.rank));
assert.ok(Number.isInteger(result.matrix.nullity));
assert.ok(Number.isFinite(result.matrix.condition_number));
assert.ok(result.matrix.residual_norm<1e-2);
assert.equal(Object.keys(result.residuals.byBody).length,20);
assert.ok(result.residuals.max_body_norm<1e-2);
assert.equal(result.jointReactions.length,25);
assert.equal(result.actuatorForces.length,3);
assert.equal(result.holdingTorques.length,4);
assert.equal(result.globalBalance.status,"unique");
assert.ok(Math.hypot(...result.globalBalance.force_residual_N)<1e-2);
assert.ok(Math.hypot(...result.globalBalance.moment_residual_Nm)<1e-2);
assert.equal(result.loadInterpretation.label,"重力准静态理论约束扳手");
assert.equal(result.loadInterpretation.contact_normal_load,"not_mapped");
assert.equal(JSON.stringify(result).includes("normal_load_N"),false);
for(const joint of result.jointReactions){
  assert.ok(joint.passive_constraint_wrench);
  assert.ok(joint.total_joint_wrench);
  assert.ok(joint.force_decomposition.axial_N);
  assert.ok(joint.force_decomposition.radial_magnitude_N);
  assert.equal(joint.contact_load_mapping,"not_mapped");
  if(joint.joint_type==="translational")assert.ok(joint.actuator_axial_force);
}
const quantities=[];
for(const joint of result.jointReactions){
  quantities.push(...Object.values(joint.passive_constraint_wrench.components),...Object.values(joint.total_joint_wrench.components));
  if(joint.actuator_axial_force)quantities.push(joint.actuator_axial_force);
}
for(const quantity of quantities)if(quantity.status==="indeterminate")assert.equal(quantity.value,null,"indeterminate physical result must not leak a particular solution");
assert.ok(result.internalWrenchBalance.max_force_N<1e-10);
assert.ok(result.internalWrenchBalance.max_moment_Nm<1e-10);

const Z=result.solution.nullspace;
for(const vector of Z){
  const residual=result.assembly.A.map(row=>row.reduce((sum,value,index)=>sum+value*vector[index],0));
  assert.ok(Math.hypot(...residual)<1e-7);
}
for(let i=0;i<Z.length;i++)for(let j=0;j<Z.length;j++){
  const value=Z[i].reduce((sum,item,index)=>sum+item*Z[j][index],0);
  assert.ok(Math.abs(value-(i===j?1:0))<1e-7);
}

const perturbed=statics.solve(runtime.solveOutput("J2",1e-4),boundary);
assert.equal(perturbed.matrix.rank,result.matrix.rank);
assert.deepEqual(perturbed.actuatorForces.map(item=>item.status),result.actuatorForces.map(item=>item.status));

const shiftedSolution=structuredClone(runtime.solveOutput("J2",0)),shiftMm=[321,-147,88];
for(const marker of shiftedSolution.frame.all_joint_marker_sides)marker.xyz_mm=marker.xyz_mm.map((value,index)=>value+shiftMm[index]);
for(const com of Object.values(shiftedSolution.frame.mass.COM_world_mm))for(let index=0;index<3;index++)com[index]+=shiftMm[index];
const shifted=statics.solve(shiftedSolution,boundary);
assert.equal(shifted.matrix.rank,result.matrix.rank);
assert.deepEqual(shifted.actuatorForces.map(item=>item.status),result.actuatorForces.map(item=>item.status));
for(let index=0;index<3;index++)if(result.actuatorForces[index].status==="unique")assert.ok(Math.abs(shifted.actuatorForces[index].value-result.actuatorForces[index].value)<1e-7);

console.log("nominal leg gravity statics regression test passed");
