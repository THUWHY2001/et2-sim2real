import assert from "node:assert/strict";
import fs from "node:fs";
import {createLegPinRuntime} from "../src/leg-pin-runtime.mjs";
import {createLegGravityStatics} from "../src/leg-gravity-statics.mjs";
import {analyzePinNearModes} from "../src/leg-pin-modal-diagnostics.mjs";

const legacy=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8")),runtime=createLegPinRuntime(legacy),statics=createLegGravityStatics(runtime.model,runtime),boundary={worldInterface:"J_LEG_L_HIP_roll_RE",footContact:"none",heldRevolutes:["J_LEG_L_HIP_roll_RE","J_LEG_L_HIP_yaw_RE","J_LEG_L_ANKLE_roll_RE","J_LEG_L_TOE_pitch_RE"],toeConstraint:"position-held-unverified",loadCase:"gravity-quasi-static-pin-revolute-v1"};
const report=analyzePinNearModes(runtime.model,runtime,statics,boundary);
assert.equal(report.model_id,"pin-revolute-v1");
assert.equal(report.near_modes.length,3);
const singular=report.near_modes.map(item=>item.singular_value).sort((a,b)=>b-a);
for(const [actual,expected] of singular.map((value,index)=>[value,[1.749703598e-7,7.274723655e-8,2.188508830e-8][index]]))assert.ok(Math.abs(actual-expected)/expected<.02,`${actual} vs ${expected}`);
assert.ok(report.spectral_gap_ratio>1e4);
assert.equal(report.output_projection_rank,3,"the near-null subspace, rather than each arbitrary SVD basis vector, must span J2/J3/J4");
for(const mode of report.near_modes)assert.equal(Object.keys(mode.body_twists).length,20);
assert.deepEqual(Object.keys(report.output_aligned_modes),["J2","J3","J4"]);
for(const [output,mode] of Object.entries(report.output_aligned_modes))for(const [key,value] of Object.entries(mode.output_rates))assert.ok(Math.abs(value-(key===output?1:0))<1e-7,`${output}/${key}=${value}`);
for(const output of ["J2","J3","J4"]){
  const item=report.finite_difference[output];assert.equal(item.body_twist_dimension,120);assert.ok(item.constraint_residual_norm>0);assert.equal(item.ranking[0].joint_id,`J_LEG_L_${output==="J2"?"HIP":output==="J3"?"KNEE":"ANKLE"}_pitch_T`);assert.ok(item.ranking[0].components.some(component=>component.basis_label.startsWith("F_radial_")));
}
assert.equal(report.prismatic_audit.length,3);
assert.ok(report.prismatic_audit.every(item=>Math.abs(item.axis_world_xyz[1])>=7e-6&&Math.abs(item.axis_world_xyz[1])<=1.3e-5));
assert.ok(Math.max(...report.prismatic_audit.map(item=>item.max_lateral_gap_mm))>.0007);
assert.ok(report.prismatic_audit.every(item=>item.initial_marker_offset_from_endpoint_line_mm<.01));
assert.equal(report.strict_acceptance_changed,false);
const reportPath=new URL("../data/leg-full-package/pin_revolute_v1_modal_diagnostics_report.json",import.meta.url),csvPath=new URL("../data/leg-full-package/pin_revolute_v1_modal_diagnostics_report.csv",import.meta.url);
assert.ok(fs.existsSync(reportPath));assert.ok(fs.existsSync(csvPath));
const saved=JSON.parse(fs.readFileSync(reportPath,"utf8"));assert.equal(saved.model_id,"pin-revolute-v1");assert.equal(saved.near_modes.length,3);
console.log("leg pin near-mode diagnostics regression test passed");
