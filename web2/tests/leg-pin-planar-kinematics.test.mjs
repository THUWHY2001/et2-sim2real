import assert from "node:assert/strict";
import fs from "node:fs";
import {createLegPinPlanarModel} from "../src/leg-pin-planar-model.mjs";
import {createLegPinPlanarKinematics} from "../src/leg-pin-planar-kinematics.mjs";

const legacy=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8")),model=createLegPinPlanarModel(legacy),kinematics=createLegPinPlanarKinematics(model);
assert.equal(model.meta.model_id,"pin-planar-ideal-v1");
assert.equal(model.meta.parent_model_id,"pin-revolute-v1");
assert.equal(model.meta.J5_deg,0);
assert.equal(model.meta.plane,"X-Z");
assert.equal(model.bodies.length,20);
assert.equal(Object.keys(model.joints).length,25);
assert.equal(model.idealization.hardpoint_y_projection.length,Object.keys(legacy.hardpoints).length);
assert.equal(model.idealization.prismatic_axes.length,3);
assert.ok(model.idealization.prismatic_axes.every(item=>item.ideal_axis_xz.length===2&&Math.abs(item.source_axis_world_xyz[1])>0));

const nominal=kinematics.solveOutput("J2",0);
assert.equal(nominal.complete,true);
assert.equal(nominal.constraint_jacobian.rows,57);
assert.equal(nominal.constraint_jacobian.columns,60);
assert.equal(nominal.constraint_jacobian.rank,57);
assert.equal(nominal.constraint_jacobian.mobility,3);
assert.ok(nominal.max_constraint_residual_mm<1e-8);
const length=kinematics.lengthJacobian(nominal.outputs);
assert.equal(length.rank,3);
assert.ok(Math.abs(length.determinant_m3_per_rad3)>1e-8);
assert.ok(length.condition_number>1.05&&length.condition_number<1.1,"condition number must use the physical unscaled length Jacobian");

for(const output of ["J2","J3","J4"])for(const delta of [-10,10]){
  const solution=kinematics.solveOutput(output,delta);assert.equal(solution.complete,true,`${output} ${delta}`);assert.equal(solution.constraint_jacobian.mobility,3);assert.ok(solution.max_constraint_residual_mm<1e-6,`${output} ${delta}: ${solution.max_constraint_residual_mm}`);const J=kinematics.lengthJacobian(solution.outputs);assert.equal(J.rank,3);assert.ok(Number.isFinite(J.condition_number));
}
assert.equal(legacy.meta.model_id,undefined);
console.log("leg pin planar ideal kinematics regression test passed");
