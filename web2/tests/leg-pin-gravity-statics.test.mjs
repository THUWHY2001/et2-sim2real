import assert from "node:assert/strict";
import fs from "node:fs";
import {createLegPinRuntime} from "../src/leg-pin-runtime.mjs";
import {createLegGravityStatics} from "../src/leg-gravity-statics.mjs";

const legacy=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8"));
const runtime=createLegPinRuntime(legacy),model=runtime.model,statics=createLegGravityStatics(model,runtime);
const boundary={worldInterface:"J_LEG_L_HIP_roll_RE",footContact:"none",heldRevolutes:["J_LEG_L_HIP_roll_RE","J_LEG_L_HIP_yaw_RE","J_LEG_L_ANKLE_roll_RE","J_LEG_L_TOE_pitch_RE"],toeConstraint:"position-held-unverified",loadCase:"gravity-quasi-static-pin-revolute-v1"};
const nominal=runtime.solveOutput("J2",0),assembly=statics.assemble(nominal,boundary);
assert.equal(assembly.equations.length,120);
assert.equal(assembly.joints.length,25);
assert.equal(assembly.unknowns.length,131);
for(const id of ["J_LEG_L_P011_RE_L10_L03","J_LEG_L_P013_RE_L12_L11","J_LEG_L_P014_RE_L03_L13","J_LEG_L_P016_RE_B15_L14","J_LEG_L_P019_RE_L18_L04","J_LEG_L_P021_RE_B20_L19"]){
  const joint=assembly.joints.find(item=>item.joint_id===id);assert.equal(joint.joint_type,"revolute");assert.equal(joint.constraint_dimension,5);
}

const result=statics.solve(nominal,boundary);
assert.equal(result.matrix.nullity,assembly.unknowns.length-result.matrix.rank);
assert.equal(result.unbalancedModes.length,assembly.equations.length-result.matrix.rank);
assert.equal(result.matrix.consistent,true);
assert.ok(result.residuals.max_force_norm_N<1e-8,`force residual ${result.residuals.max_force_norm_N}`);
assert.ok(result.residuals.max_moment_norm_Nm<1e-8,`moment residual ${result.residuals.max_moment_norm_Nm}`);
for(const item of result.unknownValues){
  if(item.status==="indeterminate")assert.equal(item.value,null);
  else assert.ok(item.nullProjection<=1e-8);
}
const mobility=statics.auditPassiveKinematicMobility(nominal,boundary);
assert.equal(mobility.strict.mobility,0,"strict pin/prismatic constraints currently eliminate all three output virtual motions");
assert.equal(mobility.source_precision_diagnostic.mobility,3,"three intended output modes appear only at source-precision tolerance");
for(const output of ["J2","J3","J4"]){
  const check=statics.verifyGravityVirtualWork(result,output);
  assert.equal(check.total_generalized_actuator_torque.status,"indeterminate",`${output} must remain blocked until the strict velocity constraints admit the output mode`);
  assert.equal(check.total_generalized_actuator_torque.value,null);
  assert.equal(check.passed,false);
}
const yaw=statics.auditYawGravity(result);assert.equal(yaw.passed,true,JSON.stringify(yaw));

const zero=createLegGravityStatics(model,runtime,{gravityScale:0}).solve(nominal,boundary),doubled=createLegGravityStatics(model,runtime,{gravityScale:2}).solve(nominal,boundary);
assert.ok(zero.matrix.residual_norm<1e-10);
for(let index=0;index<result.actuatorForces.length;index++)if(result.actuatorForces[index].status==="unique")assert.ok(Math.abs(doubled.actuatorForces[index].value-2*result.actuatorForces[index].value)<1e-7);
assert.ok(Math.abs(doubled.matrix.residual_norm-2*result.matrix.residual_norm)<1e-10);
const reportPath=new URL("../data/leg-full-package/pin_revolute_v1_gravity_statics_report.json",import.meta.url),csvPath=new URL("../data/leg-full-package/pin_revolute_v1_gravity_statics_report.csv",import.meta.url);
assert.ok(fs.existsSync(reportPath));assert.ok(fs.existsSync(csvPath));
const report=JSON.parse(fs.readFileSync(reportPath,"utf8"));
assert.deepEqual({rows:report.matrix.rows,columns:report.matrix.columns,rank:report.matrix.rank,right_nullity:report.matrix.right_nullity,left_nullity:report.matrix.left_nullity},{rows:120,columns:131,rank:120,right_nullity:11,left_nullity:0});
assert.equal(report.joints.length,25);
assert.equal(report.actuator_holding_forces.length,3);
assert.ok(report.actuator_holding_forces.every(item=>item.status==="indeterminate"&&item.value===null));
assert.ok(report.virtual_work.every(item=>item.passed===false&&item.total_generalized_actuator_torque.status==="indeterminate"));
assert.equal(report.physical_acceptance.status,"blocked");
assert.equal(report.physical_acceptance.suitable_for_friction_contact_load_estimation,false);
assert.equal(report.friction_circle_development,"not_started");
console.log("leg pin-joint gravity statics regression test passed");
