import assert from "node:assert/strict";
import fs from "node:fs";
import {createLegPinPlanarModel} from "../src/leg-pin-planar-model.mjs";
import {createLegPinPlanarKinematics} from "../src/leg-pin-planar-kinematics.mjs";
import {createLegPinPlanarFrictionCatalog} from "../src/leg-pin-planar-friction-catalog.mjs";
import {createLegPinPlanarFrictionKinematics} from "../src/leg-pin-planar-friction-kinematics.mjs";

const legacy=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8")),model=createLegPinPlanarModel(legacy),kinematics=createLegPinPlanarKinematics(model),catalog=createLegPinPlanarFrictionCatalog(model),rates=createLegPinPlanarFrictionKinematics(model,kinematics,catalog),solution=kinematics.solveOutput("J2",0);
const outputIds={J2:"J_LEG_L_HIP_pitch_RE",J3:"J_LEG_L_KNEE_pitch_RE",J4:"J_LEG_L_ANKLE_pitch_RE"};
for(const [output,jointId] of Object.entries(outputIds)){const group=catalog.byJointId.get(jointId)[0],g=rates.relativeRateJacobian(solution,group);for(const key of Object.keys(outputIds))assert.ok(Math.abs(g.values[key]-(key===output?1:0))<1e-7,`${jointId}/${key}=${g.values[key]}`)}
for(const jointId of ["J_LEG_L_HIP_roll_RE","J_LEG_L_P013_F_L8_L12"]){const g=rates.relativeRateJacobian(solution,catalog.byJointId.get(jointId)[0]);assert.ok(Object.values(g.values).every(value=>Math.abs(value)<1e-8))}
const contact=catalog.byJointId.get("J_LEG_L_P013_RE_L12_L11")[0],g=rates.relativeRateJacobian(solution,contact),velocity=rates.relativeVelocity(solution,contact,{J2:.7,J3:-.2,J4:.4});assert.ok(Math.abs(velocity.value_rad_s-(g.values.J2*.7+g.values.J3*-.2+g.values.J4*.4))<1e-12);
const coarse=rates.relativeRateJacobian(solution,contact,{step_deg:.002}),fine=rates.relativeRateJacobian(solution,contact,{step_deg:.001});for(const output of ["J2","J3","J4"])assert.ok(Math.abs(coarse.values[output]-fine.values[output])<1e-6);
const cross=rates.crossCheckRelativeRates(solution);assert.equal(cross.nullity,3);assert.ok(cross.max_constraint_velocity_residual<1e-7);assert.ok(cross.max_relative_rate_difference<1e-6);assert.equal(cross.contacts.length,25);
console.log("leg pin planar friction relative-rate regression test passed");
