import assert from "node:assert/strict";
import {buildReport,serializeCsv} from "../scripts/report-leg-pin-planar-friction.mjs";

const report=buildReport({samples:9});
assert.equal(report.schema,"et2-leg-pin-planar-friction-v1");assert.equal(report.model.id,"pin-planar-ideal-v1");assert.equal(report.boundary.load_mode,"gravity_only");assert.equal(report.boundary.includes_inertia,false);assert.equal(report.contact_catalog.length,25);assert.equal(report.parameter_set.kind,"software_verification_example_not_measurement");
assert.match(report.parameter_set.warning,/not.*engineering|非工程|software/i);
for(const id of ["J_LEG_L_P010_S_L09_L03","J_LEG_L_P018_S_L17_L04"]){const item=report.contact_catalog.find(value=>value.joint_id===id);assert.equal(item.verification_status,"real_joint_type_unverified");assert.equal(item.include_in_official_total,false)}
for(const output of ["J2","J3","J4"]){const cycle=report.cycles[output];assert.equal(cycle.samples.length,9);assert.ok(cycle.samples.some(sample=>sample.contacts.some(item=>item.motion_state==="reversal_or_stick_candidate")));assert.ok(cycle.max_abs_power_balance_residual_W<1e-9);assert.ok(cycle.samples.every(sample=>sample.load_mode==="gravity_only"))}
const csv=serializeCsv(report);for(const header of ["output_joint","time_s","joint_id","physical_contact_id","joint_resultant_load_N","guide_contact_normal_N","friction_contact_load_N","load_source","mu_static","mu_kinetic","effective_radius_m","relative_velocity_rad_s","kinetic_torque_Nm","static_friction_capacity_Nm","equivalent_output_torque_Nm","contribution_pct","status","parameter_set_kind","result_scope"])assert.match(csv,new RegExp(header));
console.log("leg pin planar friction report regression test passed");
