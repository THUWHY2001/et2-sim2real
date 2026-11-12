import assert from "node:assert/strict";
import fs from "node:fs";
import {createLegPinPlanarModel} from "../src/leg-pin-planar-model.mjs";
import {createLegPinPlanarKinematics} from "../src/leg-pin-planar-kinematics.mjs";
import {createLegPinPlanarStatics} from "../src/leg-pin-planar-statics.mjs";

const legacy=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8")),model=createLegPinPlanarModel(legacy),kinematics=createLegPinPlanarKinematics(model),statics=createLegPinPlanarStatics(model,kinematics),solution=kinematics.solveOutput("J2",0),result=statics.solve(solution);
assert.equal(result.complete,true);
assert.equal(result.matrix.rows,60);
assert.equal(result.matrix.consistent,true);
assert.ok(result.residuals.max_force_norm_N<1e-8);
assert.ok(result.residuals.max_moment_abs_Nm<1e-8);
assert.deepEqual(Object.keys(result.gravity_torque_Nm),["J2","J3","J4"]);
assert.ok(Object.values(result.gravity_torque_Nm).every(Number.isFinite));
assert.equal(result.length_jacobian.rank,3);
assert.equal(result.virtual_work.pushrod_forces.every(item=>item.status==="unique"),true);
assert.ok(result.virtual_work.residual_norm_Nm<1e-8);
assert.equal(result.actuator_forces.length,3);
assert.ok(result.actuator_forces.every(item=>item.status==="unique"));
for(const force of result.actuator_forces){const expected=result.virtual_work.pushrod_forces.find(item=>item.joint_id===force.joint_id);assert.ok(Math.abs(force.value-expected.value)<1e-5,`${force.joint_id}: ${force.value} vs ${expected.value}`)}
assert.equal(result.joint_reactions.length,25);
for(const joint of result.joint_reactions)for(const item of Object.values(joint.components))if(item.status==="indeterminate")assert.equal(item.value,null);
const j2Reaction=result.joint_reactions.find(item=>item.joint_id==="J_LEG_L_HIP_pitch_RE");assert.deepEqual(j2Reaction.components.My_Nm,{status:"not_transmitted",value:null});assert.ok(Number.isFinite(j2Reaction.force_direction_deg));
assert.equal(result.boundary.load,"gravity-quasi-static-planar");
assert.deepEqual(result.boundary.excluded,["inertia","assembly_preload","friction","3d_support_load_distribution","J5_roll"]);
const zero=createLegPinPlanarStatics(model,kinematics,{gravityScale:0}).solve(solution),doubled=createLegPinPlanarStatics(model,kinematics,{gravityScale:2}).solve(solution);
assert.ok(zero.actuator_forces.every(item=>item.status==="unique"&&Math.abs(item.value)<1e-9));
for(let index=0;index<result.actuator_forces.length;index++)assert.ok(Math.abs(doubled.actuator_forces[index].value-2*result.actuator_forces[index].value)<1e-7);
for(const output of ["J2","J3","J4"]){assert.ok(Math.abs(zero.gravity_torque_Nm[output])<1e-10);assert.ok(Math.abs(doubled.gravity_torque_Nm[output]-2*result.gravity_torque_Nm[output])<1e-9)}
console.log("leg pin planar ideal statics regression test passed");
