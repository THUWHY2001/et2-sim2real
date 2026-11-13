import assert from "node:assert/strict";
import fs from "node:fs";

const path=new URL("../data/leg-full-package/pin_revolute_v1_kinematics_report.json",import.meta.url);
assert.ok(fs.existsSync(path),"pin-revolute-v1 kinematics report must be generated");
const report=JSON.parse(fs.readFileSync(path,"utf8"));
assert.equal(report.model_id,"pin-revolute-v1");
assert.equal(report.legacy_model_unchanged,true);
assert.equal(report.mapping.length,6);
assert.equal(report.axis_assumption.world_axis,"+Y");
for(const item of report.mapping){
  assert.equal(item.i_axis_local.length,3);
  assert.equal(item.j_axis_local.length,3);
  assert.notEqual(item.i_axis_local,item.j_axis_local);
}
assert.equal(report.nominal.compatible,true);
for(const joint of ["J2","J3","J4"]){
  const scan=report.j5_zero_scans[joint];
  assert.equal(scan.samples.length,121);
  assert.equal(scan.samples[0].delta_deg,-10);
  assert.equal(scan.samples.at(-1).delta_deg,10);
  assert.equal(scan.compatible,true);
  for(const sample of scan.samples){
    assert.equal(sample.pin_checks.length,6);
    assert.equal(sample.translational_checks.length,3);
    assert.ok(!("projected_pose" in sample));
    assert.ok(!("tolerance_repair" in sample));
  }
}
for(const key of ["minus_5_deg","plus_5_deg"]){
  const item=report.j5_conflict[key];
  assert.equal(typeof item.compatible,"boolean");
  assert.ok(item.P19_axis_angle_deg>0);
  assert.ok(item.P21_axis_angle_deg>0);
  assert.equal(item.compatible,false);
}
assert.equal(report.stage_gate.j5_zero_passed,true);
assert.equal(report.stage_gate.proceed_to_statics,true);
assert.equal(report.full_leg_motion_valid,false);
assert.deepEqual(report.valid_scope,{J5_deg:0,J2_delta_deg:[-10,10],J3_delta_deg:[-10,10],J4_delta_deg:[-10,10]});
assert.ok(fs.existsSync(new URL("../data/leg-full-package/pin_revolute_v1_kinematics_report.csv",import.meta.url)));
console.log("leg pin-joint kinematics report regression test passed");
