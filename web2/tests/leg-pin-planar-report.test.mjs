import assert from "node:assert/strict";
import fs from "node:fs";

const jsonUrl=new URL("../data/leg-full-package/pin_planar_ideal_v1_report.json",import.meta.url),csvUrl=new URL("../data/leg-full-package/pin_planar_ideal_v1_report.csv",import.meta.url);
assert.ok(fs.existsSync(jsonUrl));assert.ok(fs.existsSync(csvUrl));
const report=JSON.parse(fs.readFileSync(jsonUrl,"utf8"));
assert.equal(report.model.id,"pin-planar-ideal-v1");
assert.equal(report.model.parent,"pin-revolute-v1");
assert.equal(report.boundary.J5_deg,0);
assert.deepEqual(report.boundary.excluded,["inertia","assembly_preload","friction","3d_support_load_distribution","J5_roll"]);
assert.equal(report.baseline_acceptance.passed,true);
assert.equal(report.baseline.kinematics.constraint_jacobian.mobility,3);
assert.equal(report.baseline.statics.matrix.rank,60);
assert.equal(report.baseline.statics.joint_reactions.length,25);
assert.ok(report.baseline.statics.virtual_work.residual_norm_Nm<1e-8);
assert.deepEqual(report.baseline.statics.actuator_forces.map(item=>item.status),["unique","unique","unique"]);
for(const output of ["J2","J3","J4"]){const cycle=report.cycles[output];assert.equal(cycle.samples.length,121);assert.equal(cycle.samples[0].time_s,0);assert.equal(cycle.samples.at(-1).time_s,1);assert.ok(Math.max(...cycle.samples.map(item=>item.delta_deg))>9.99);assert.ok(Math.min(...cycle.samples.map(item=>item.delta_deg))< -9.99);assert.ok(cycle.samples.every(item=>item.complete&&item.joint_reactions.length===25&&item.virtual_work_residual_Nm<1e-8));assert.ok(Number.isFinite(cycle.max_length_jacobian_condition_number))}
assert.equal(report.friction_circle_development,"not_started");
console.log("leg pin planar ideal report regression test passed");
