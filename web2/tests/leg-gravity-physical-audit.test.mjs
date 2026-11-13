import assert from "node:assert/strict";
import fs from "node:fs";
import {createLegFullRuntime} from "../src/leg-full-runtime.mjs";
import {createLegGravityStatics} from "../src/leg-gravity-statics.mjs";

const model=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8"));
const runtime=createLegFullRuntime(model);
const boundary={worldInterface:"J_LEG_L_HIP_roll_RE",footContact:"none",heldRevolutes:["J_LEG_L_HIP_roll_RE","J_LEG_L_HIP_yaw_RE","J_LEG_L_ANKLE_roll_RE","J_LEG_L_TOE_pitch_RE"],toeConstraint:"position-held-unverified",loadCase:"gravity-quasi-static"};
const nominalStatics=createLegGravityStatics(model,runtime),nominal=nominalStatics.solveNominal(boundary);
const near=(actual,expected,tolerance,label)=>assert.ok(Math.abs(actual-expected)<=tolerance,`${label}: expected ${expected}, got ${actual}`);

assert.ok(nominal.residuals.max_force_norm_N>0);
assert.ok(nominal.residuals.max_moment_norm_Nm>0);
assert.ok(nominal.residuals.max_force_norm_N<3e-5);
assert.ok(nominal.residuals.max_moment_norm_Nm<4e-3);
assert.ok(nominal.residuals.normalized.force>0);
assert.ok(nominal.residuals.normalized.moment>0);
assert.equal(nominal.residuals.byBodyPhysical.leg_l_link18.moment_norm_Nm>3e-3,true);
assert.equal(nominal.residuals.byBodyPhysical.leg_l_link19.moment_norm_Nm>3e-3,true);

assert.equal(nominal.unbalancedModes.length,3);
assert.ok(nominal.unbalancedModes.every(mode=>mode.dominant_rod_spin_candidates.length>0));
assert.ok(nominal.unbalancedModes.some(mode=>mode.dominant_rod_spin_candidates[0].joint_id==="J_LEG_L_ANKLE_pitch_T"));

const zero=createLegGravityStatics(model,runtime,{gravityScale:0}).solveNominal(boundary);
assert.ok(zero.matrix.residual_norm<1e-12);
for(const item of [...zero.actuatorForces,...zero.holdingTorques])if(item.status==="unique")near(item.value,0,1e-10,`${item.joint_id} zero gravity`);
for(const joint of zero.jointReactions)for(const quantity of Object.values(joint.total_joint_wrench.components))if(quantity.status==="unique")near(quantity.value,0,1e-10,`${joint.joint_id} zero-gravity component`);

const doubled=createLegGravityStatics(model,runtime,{gravityScale:2}).solveNominal(boundary);
for(let index=0;index<nominal.actuatorForces.length;index++){
  assert.equal(doubled.actuatorForces[index].status,nominal.actuatorForces[index].status);
  if(nominal.actuatorForces[index].status==="unique")near(doubled.actuatorForces[index].value,2*nominal.actuatorForces[index].value,1e-7,`${nominal.actuatorForces[index].joint_id} doubled gravity`);
}
near(doubled.matrix.residual_norm,2*nominal.matrix.residual_norm,1e-10,"doubled-gravity residual");

for(const output of ["J2","J3","J4"])for(const delta of [-.1,.1]){
  const perturbed=nominalStatics.solve(runtime.solveOutput(output,delta),boundary);
  assert.equal(perturbed.matrix.rank,nominal.matrix.rank);
  assert.equal(perturbed.matrix.nullity,nominal.matrix.nullity);
  assert.ok(Math.abs(perturbed.matrix.residual_norm/nominal.matrix.residual_norm-1)<.02);
}

const yawAudit=nominalStatics.auditYawGravity(nominal);
assert.equal(yawAudit.joint,"J1");
assert.ok(yawAudit.axis_vertical_alignment>1-1e-12);
assert.equal(yawAudit.expected_gravity_torque_Nm,0);
assert.ok(Math.abs(yawAudit.potential_derivative_Nm)>6e-3);
assert.equal(yawAudit.passed,false);
assert.equal(yawAudit.dominant_part,"leg_l_link18");

for(const output of ["J2","J3","J4"]){
  const check=nominalStatics.verifyGravityVirtualWork(nominal,output);
  assert.equal(check.acceptance.absolute_threshold_Nm,2e-3);
  assert.equal(check.acceptance.relative_threshold,1e-5);
  assert.equal(check.acceptance.formula,"unique && abs_residual <= abs_threshold && relative_residual <= relative_threshold");
  assert.equal(check.passed,check.total_generalized_actuator_torque.status==="unique"&&check.absolute_residual_Nm<=check.acceptance.absolute_threshold_Nm&&check.relative_residual<=check.acceptance.relative_threshold);
}

console.log("leg gravity physical audit regression test passed");
