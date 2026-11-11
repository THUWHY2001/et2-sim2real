import assert from "node:assert/strict";
import fs from "node:fs";
import {buildNominalGravityStaticsReport,reportPaths,serializeJointCsv} from "../scripts/report-leg-gravity-statics.mjs";

const report=buildNominalGravityStaticsReport(),csv=serializeJointCsv(report);
assert.equal(report.scope.stage,"nominal-pose-only");
assert.equal(report.scope.boundary.foot_contact,"none");
assert.equal(report.scope.load_definition,"重力准静态理论载荷");
assert.equal(report.matrix.consistent,true);
assert.equal(report.matrix.left_nullity,3);
assert.equal(report.matrix.exact_equilibrium_passed,false);
assert.match(report.matrix.solution_class,/least-squares/);
assert.equal(report.joints.length,25);
assert.equal(report.actuator_forces.length,3);
assert.equal(report.virtual_work.length,3);
assert.ok(report.virtual_work.every(item=>item.passed&&item.total_generalized_actuator_torque.status==="unique"));
assert.ok(report.residuals.matrix_norm<1e-2);
assert.ok(report.residuals.max_body_norm<1e-2);
assert.ok(report.residuals.max_force_norm_N<3e-5);
assert.ok(report.residuals.max_moment_norm_Nm<4e-3);
assert.ok(report.residuals.normalized.force>0);
assert.ok(report.residuals.normalized.moment>0);
assert.equal(report.yaw_gravity_audit.passed,false);
assert.equal(report.yaw_gravity_audit.dominant_part,"leg_l_link18");
assert.equal(report.physical_acceptance.passed,false);
assert.equal(report.physical_acceptance.suitable_for_friction_torque_estimation,false);
assert.equal(report.perturbation_tests.zero_gravity.passed,true);
assert.equal(report.perturbation_tests.double_gravity.passed,true);
assert.equal(report.perturbation_tests.pose_rank_stability.passed,true);
assert.ok(report.unbalanced_modes.length===3);
for(const item of report.virtual_work){
  assert.equal(item.acceptance.absolute_threshold_Nm,2e-3);
  assert.equal(item.acceptance.relative_threshold,1e-5);
}
assert.equal(report.contact_load_qualification,"not_mapped");
assert.equal(JSON.stringify(report).includes("normal_load_N"),false);
for(const joint of report.joints){
  assert.ok(joint.uniqueness.unique_components.length+joint.uniqueness.indeterminate_components.length>0);
  for(const component of joint.uniqueness.indeterminate_components)assert.equal(component.value,null);
}
assert.match(csv,/joint_id,joint_type,body_i,body_j,quantity,unit,status,value,load_category,contact_load_mapping/);
assert.match(csv,/,N,/);
assert.match(csv,/,N.m,/);
assert.match(csv,/__AUDIT__,audit,,,max_force_residual,N,observed/);
assert.match(csv,/__AUDIT__,audit,,,max_moment_residual,N.m,observed/);
assert.match(csv,/__AUDIT__,audit,,,J1_gravity_potential_derivative,N.m,failed/);
assert.match(csv,/__AUDIT__,audit,,,physical_acceptance,,blocked/);
assert.ok(fs.existsSync(reportPaths.json));
assert.ok(fs.existsSync(reportPaths.csv));

console.log("leg gravity statics report regression test passed");
