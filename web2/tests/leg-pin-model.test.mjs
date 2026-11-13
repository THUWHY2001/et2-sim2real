import assert from "node:assert/strict";
import fs from "node:fs";
import {PIN_JOINT_MAPPINGS,createPinJointModel} from "../src/leg-pin-model.mjs";

const legacy=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8")),snapshot=JSON.stringify(legacy),pin=createPinJointModel(legacy);
const multiply=(R,v)=>R.map(row=>row.reduce((sum,value,index)=>sum+value*v[index],0));
const nearVector=(actual,expected,tolerance=1e-10)=>assert.ok(Math.hypot(...actual.map((value,index)=>value-expected[index]))<tolerance,`${actual} != ${expected}`);

assert.equal(JSON.stringify(legacy),snapshot,"legacy model must not be mutated");
assert.equal(pin.meta.model_id,"pin-revolute-v1");
assert.equal(pin.meta.pin_axis_assumption,"robot_world_+Y_pending_CAD_verification");
assert.equal(Object.keys(pin.joints).length,25);
assert.equal(PIN_JOINT_MAPPINGS.length,6);
assert.deepEqual(PIN_JOINT_MAPPINGS.map(item=>item.hardpoint),["P11","P13","P14","P16","P19","P21"]);

for(const mapping of PIN_JOINT_MAPPINGS){
  assert.equal(legacy.joints[mapping.legacy_joint_id].type,"spherical");
  assert.equal(pin.joints[mapping.legacy_joint_id],undefined);
  const joint=pin.joints[mapping.pin_joint_id];
  assert.equal(joint.type,"revolute");
  assert.equal(joint.i_part,legacy.joints[mapping.legacy_joint_id].i_part);
  assert.equal(joint.j_part,legacy.joints[mapping.legacy_joint_id].j_part);
  assert.equal(joint.legacy_joint_id,mapping.legacy_joint_id);
  assert.equal(joint.axis_source,"geometry_assumption_pending_cad");
  assert.deepEqual(joint.assumed_axis_world_nominal,[0,1,0]);
  assert.notStrictEqual(joint.i_marker_axis_local,joint.j_marker_axis_local);
  nearVector(multiply(pin.meta.nominal_world_rotations[joint.i_part],joint.i_marker_axis_local),[0,1,0]);
  nearVector(multiply(pin.meta.nominal_world_rotations[joint.j_part],joint.j_marker_axis_local),[0,1,0]);
}

for(const id of ["J_LEG_L_HIP_pitch_T","J_LEG_L_KNEE_pitch_T","J_LEG_L_ANKLE_pitch_T"])assert.deepEqual(pin.joints[id],legacy.joints[id]);

console.log("leg pin-joint model regression test passed");
