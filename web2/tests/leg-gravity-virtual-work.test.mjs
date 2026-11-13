import assert from "node:assert/strict";
import fs from "node:fs";
import {createLegFullRuntime} from "../src/leg-full-runtime.mjs";
import {createLegGravityStatics} from "../src/leg-gravity-statics.mjs";

const model=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8"));
const runtime=createLegFullRuntime(model),statics=createLegGravityStatics(model,runtime);
const boundary={worldInterface:"J_LEG_L_HIP_roll_RE",footContact:"none",heldRevolutes:["J_LEG_L_HIP_roll_RE","J_LEG_L_HIP_yaw_RE","J_LEG_L_ANKLE_roll_RE","J_LEG_L_TOE_pitch_RE"],toeConstraint:"position-held-unverified",loadCase:"gravity-quasi-static"};
const result=statics.solveNominal(boundary);
const actuatorIds=["J_LEG_L_HIP_pitch_T","J_LEG_L_KNEE_pitch_T","J_LEG_L_ANKLE_pitch_T"];

for(const output of ["J2","J3","J4"]){
  const rates=runtime.signedRates(output,0);
  assert.equal(rates.complete,true);
  assert.ok(Math.abs(rates.values[`${output==="J2"?"J_LEG_L_HIP":output==="J3"?"J_LEG_L_KNEE":"J_LEG_L_ANKLE"}_pitch_RE`]-1)<1e-5);
  for(const held of boundary.heldRevolutes)assert.ok(Math.abs(rates.values[held])<1e-6,`${held} must remain held during ${output} virtual displacement`);
  const check=statics.verifyGravityVirtualWork(result,output);
  assert.equal(check.output_joint,output);
  assert.deepEqual(check.actuators.map(item=>item.joint_id),actuatorIds);
  assert.equal(check.total_generalized_actuator_torque.status,"unique");
  assert.notEqual(check.total_generalized_actuator_torque.value,null);
  assert.equal(check.gravity_potential_derivative.status,"unique");
  assert.equal(check.passed,true,JSON.stringify(check.diagnostics));
  assert.ok(check.absolute_residual_Nm<2e-3,`${output} virtual-work residual ${check.absolute_residual_Nm}`);
}

console.log("leg gravity virtual-work regression test passed");
