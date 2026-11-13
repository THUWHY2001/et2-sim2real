import assert from "node:assert/strict";
import fs from "node:fs";
import {createLegPinRuntime} from "../src/leg-pin-runtime.mjs";
import {PIN_JOINT_MAPPINGS} from "../src/leg-pin-model.mjs";

const legacy=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8")),runtime=createLegPinRuntime(legacy),nominal=runtime.solveOutput("J2",0);
const norm=value=>Math.hypot(...value),sub=(a,b)=>a.map((value,index)=>value-b[index]);

assert.equal(runtime.model.meta.model_id,"pin-revolute-v1");
assert.equal(nominal.frame.model_id,"pin-revolute-v1");
assert.equal(nominal.complete,true);
assert.equal(nominal.frame.pin_constraint_checks.length,6);
for(const check of nominal.frame.pin_constraint_checks){
  assert.ok(check.position_residual_mm<1e-8,`${check.joint} position ${check.position_residual_mm}`);
  assert.ok(check.axis_line_angle_deg<1e-8,`${check.joint} axis ${check.axis_line_angle_deg}`);
  assert.equal(check.compatible,true);
}
for(const id of ["J_LEG_L_HIP_pitch_T","J_LEG_L_KNEE_pitch_T","J_LEG_L_ANKLE_pitch_T"]){
  const check=nominal.frame.joint_constraint_checks.find(item=>item.joint===id);
  assert.ok(check);
  assert.ok(check.lateral_gap_mm<1e-3);
  assert.ok(check.axis_cross_error<1e-8);
}

const disturbedPosition=structuredClone(runtime.model),positionJoint=disturbedPosition.joints[PIN_JOINT_MAPPINGS[0].pin_joint_id];positionJoint.initial_center_global_mm[0]+=.5;
const positionAudit=runtime.validatePinConstraints(nominal.frame,disturbedPosition);
assert.equal(positionAudit.ok,false);
assert.ok(positionAudit.checks[0].position_residual_mm>.01);

const disturbedAxis=structuredClone(runtime.model),axisJoint=disturbedAxis.joints[PIN_JOINT_MAPPINGS[0].pin_joint_id];axisJoint.i_marker_axis_local=[1,0,0];
const axisAudit=runtime.validatePinConstraints(nominal.frame,disturbedAxis);
assert.equal(axisAudit.ok,false);
assert.ok(axisAudit.checks[0].axis_line_angle_deg>80);

const yaw=runtime.solveRods({J1_deg:1}),center=legacy.output_joints.J1.initial_center_global_mm,angle=Math.PI/180,c=Math.cos(angle),s=Math.sin(angle),R=[[c,-s,0],[s,c,0],[0,0,1]],mv=(M,v)=>M.map(row=>row.reduce((sum,value,index)=>sum+value*v[index],0)),add=(a,b)=>a.map((value,index)=>value+b[index]),expected=point=>add(center,mv(R,sub(point,center)));
assert.equal(yaw.complete,true);
for(const body of Object.keys(legacy.parts)){
  if(body==="leg_l_link1")continue;
  const actual=yaw.frame.mass.COM_world_mm[body],target=expected(nominal.frame.mass.COM_world_mm[body]);
  assert.ok(norm(sub(actual,target))<1e-8,`${body} rigid yaw error ${norm(sub(actual,target))}`);
  assert.ok(Math.abs(actual[2]-nominal.frame.mass.COM_world_mm[body][2])<1e-9,`${body} changed height`);
}
assert.deepEqual({J2:yaw.outputs.J2,J3:yaw.outputs.J3,J4:yaw.outputs.J4,J5:yaw.outputs.J5},{J2:nominal.outputs.J2,J3:nominal.outputs.J3,J4:nominal.outputs.J4,J5:nominal.outputs.J5});
for(const key of ["hip_mm","knee_mm","ankle_mm"])assert.ok(Math.abs(yaw.rodStrokes[key]-nominal.rodStrokes[key])<1e-9);

console.log("leg pin-joint runtime regression test passed");
