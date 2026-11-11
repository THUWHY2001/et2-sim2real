import assert from "node:assert/strict";
import fs from "node:fs";
import {buildNominalGravityStaticsReport,reportPaths,serializeJointCsv} from "../scripts/report-leg-gravity-statics.mjs";

const report=buildNominalGravityStaticsReport(),csv=serializeJointCsv(report);
assert.equal(report.scope.stage,"nominal-pose-only");
assert.equal(report.scope.boundary.foot_contact,"none");
assert.equal(report.scope.load_definition,"重力准静态理论载荷");
assert.equal(report.matrix.consistent,true);
assert.equal(report.joints.length,25);
assert.equal(report.actuator_forces.length,3);
assert.equal(report.virtual_work.length,3);
assert.ok(report.virtual_work.every(item=>item.passed&&item.total_generalized_actuator_torque.status==="unique"));
assert.ok(report.residuals.matrix_norm<1e-2);
assert.ok(report.residuals.max_body_norm<1e-2);
assert.equal(report.contact_load_qualification,"not_mapped");
assert.equal(JSON.stringify(report).includes("normal_load_N"),false);
for(const joint of report.joints){
  assert.ok(joint.uniqueness.unique_components.length+joint.uniqueness.indeterminate_components.length>0);
  for(const component of joint.uniqueness.indeterminate_components)assert.equal(component.value,null);
}
assert.match(csv,/joint_id,joint_type,body_i,body_j,quantity,unit,status,value,load_category,contact_load_mapping/);
assert.match(csv,/,N,/);
assert.match(csv,/,N.m,/);
assert.ok(fs.existsSync(reportPaths.json));
assert.ok(fs.existsSync(reportPaths.csv));

console.log("leg gravity statics report regression test passed");
