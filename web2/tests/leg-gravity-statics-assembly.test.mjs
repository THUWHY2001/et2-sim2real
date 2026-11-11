import assert from "node:assert/strict";
import fs from "node:fs";
import {createLegFullRuntime} from "../src/leg-full-runtime.mjs";
import {createLegGravityStatics} from "../src/leg-gravity-statics.mjs";

const model=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8"));
const runtime=createLegFullRuntime(model);
const statics=createLegGravityStatics(model,runtime);
const nominal=runtime.solveOutput("J2",0);
const boundary={
  worldInterface:"J_LEG_L_HIP_roll_RE",
  footContact:"none",
  heldRevolutes:["J_LEG_L_HIP_roll_RE","J_LEG_L_HIP_yaw_RE","J_LEG_L_ANKLE_roll_RE","J_LEG_L_TOE_pitch_RE"],
  toeConstraint:"position-held-unverified",
  loadCase:"gravity-quasi-static",
};
const near=(actual,expected,tolerance=1e-10,label="value")=>assert.ok(Math.abs(actual-expected)<=tolerance,`${label}: expected ${expected}, got ${actual}`);
const add=(a,b)=>a.map((value,index)=>value+b[index]);
const norm=value=>Math.hypot(...value);
const clone=value=>structuredClone(value);

assert.throws(()=>statics.assemble(nominal,{}),/boundary.*required/i);

const assembly=statics.assemble(nominal,boundary);
assert.equal(assembly.equations.length,20*6);
assert.equal(assembly.joints.length,25);
assert.equal(assembly.unknowns.length,119);
assert.equal(assembly.A.length,120);
assert.equal(assembly.A[0].length,119);
assert.equal(assembly.b.length,120);
assert.equal(assembly.boundary.footContact,"none");

const byJoint=Object.fromEntries(assembly.joints.map(item=>[item.joint_id,item]));
for(const key of boundary.heldRevolutes){
  assert.equal(byJoint[key].joint_type,"revolute");
  assert.equal(byJoint[key].constraint_dimension,6);
  assert.equal(byJoint[key].motion_state,"position-held");
}
for(const key of ["J_LEG_L_HIP_pitch_RE","J_LEG_L_KNEE_pitch_RE","J_LEG_L_ANKLE_pitch_RE"]){
  assert.equal(byJoint[key].constraint_dimension,5);
  assert.equal(byJoint[key].motion_state,"free-output");
}
for(const key of ["J_LEG_L_HIP_pitch_T","J_LEG_L_KNEE_pitch_T","J_LEG_L_ANKLE_pitch_T"]){
  assert.equal(byJoint[key].joint_type,"translational");
  assert.equal(byJoint[key].passive_constraint_dimension,5);
  assert.equal(byJoint[key].actuator_dimension,1);
  assert.equal(assembly.unknowns.filter(item=>item.joint_id===key&&item.category==="actuator_axial_force").length,1);
}
for(const item of assembly.joints.filter(item=>item.joint_type==="fixed")){
  assert.equal(item.constraint_dimension,6);
  assert.equal(item.friction_eligible,false);
}

const rodJoint="J_LEG_L_HIP_pitch_T";
const rod=byJoint[rodJoint];
assert.ok(Math.abs(rod.geometry.axial_separation_mm)>1);
assert.ok(rod.geometry.lateral_offset_mm<1e-3);
assert.ok(rod.geometry.axis_misalignment_deg<1e-6);

const rodUnknown=assembly.unknowns.find(item=>item.joint_id===rodJoint&&item.basis_label==="F_radial_1");
const sides=Object.values(rodUnknown.body_contributions);
assert.equal(sides.length,2);
near(norm(add(sides[0].force_N,sides[1].force_N)),0,1e-12,"internal force balance");
near(norm(add(sides[0].moment_about_world_origin_Nm,sides[1].moment_about_world_origin_Nm)),0,1e-12,"internal moment balance");
assert.ok(norm(add(sides[0].moment_at_marker_Nm,sides[1].moment_at_marker_Nm))>1e-6,"separated Marker moments must not be naively opposite");

const markerPair=solution=>solution.frame.all_joint_marker_sides.filter(item=>item.joint===rodJoint);
const axial=clone(nominal),axialPair=markerPair(axial),axis=axialPair[0].axis_xyz;
axialPair[1].xyz_mm=axialPair[1].xyz_mm.map((value,index)=>value+10*axis[index]);
assert.doesNotThrow(()=>statics.assemble(axial,boundary));

const lateral=clone(nominal),lateralPair=markerPair(lateral),a=lateralPair[0].axis_xyz;
const perpendicular=Math.abs(a[0])<.8?[0,-a[2],a[1]]:[-a[1],a[0],0],length=norm(perpendicular),unitPerpendicular=perpendicular.map(value=>value/length);
lateralPair[1].xyz_mm=lateralPair[1].xyz_mm.map((value,index)=>value+0.2*unitPerpendicular[index]);
assert.throws(()=>statics.assemble(lateral,boundary),error=>error?.code==="PRISMATIC_GEOMETRY_INVALID"&&error.details.lateral_offset_mm>.1);

const skewed=clone(nominal),skewedPair=markerPair(skewed);
skewedPair[1].axis_xyz=unitPerpendicular;
assert.throws(()=>statics.assemble(skewed,boundary),error=>error?.code==="PRISMATIC_GEOMETRY_INVALID"&&error.details.axis_misalignment_deg>80);

console.log("leg gravity statics assembly regression test passed");
