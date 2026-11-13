import assert from "node:assert/strict";
import fs from "node:fs";
import {createLegPinPlanarModel} from "../src/leg-pin-planar-model.mjs";
import {createLegPinPlanarKinematics} from "../src/leg-pin-planar-kinematics.mjs";
import {createLegPinPlanarStatics} from "../src/leg-pin-planar-statics.mjs";
import {createLegPinPlanarFriction} from "../src/leg-pin-planar-friction.mjs";

const source=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8"));
const model=createLegPinPlanarModel(source),kinematics=createLegPinPlanarKinematics(model),statics=createLegPinPlanarStatics(model,kinematics),engine=createLegPinPlanarFriction(model,kinematics,statics),solution=kinematics.solveOutput("J2",0);
const params={};for(const contact of engine.catalog.contacts)if(contact.include_in_official_total)params[contact.physical_contact_id]={mu_static:.15,mu_kinetic:.1,effective_radius_m:.01,provenance:"multi_output_test"};
const qdot={J2:.43,J3:-.31,J4:.19},result=engine.evaluatePose(solution,{output_rates_rad_s:qdot,parameters:params});
let expectedPower=0;const expectedTorque={J2:0,J3:0,J4:0};
for(const item of result.contacts.filter(value=>value.status==="computed")){
  const omega=Object.keys(qdot).reduce((sum,key)=>sum+item.relative_rate_jacobian[key]*qdot[key],0);
  assert.ok(Math.abs(omega-item.relative_velocity_rad_s)<1e-12);
  assert.equal(Math.sign(item.kinetic_torque_Nm),-Math.sign(omega));
  for(const key of Object.keys(qdot)){const expected=item.relative_rate_jacobian[key]*item.kinetic_torque_Nm;assert.ok(Math.abs(expected-item.equivalent_output_torque_Nm[key])<1e-12);expectedTorque[key]+=expected}
  expectedPower+=item.kinetic_torque_Nm*omega;
}
for(const key of Object.keys(qdot))assert.ok(Math.abs(expectedTorque[key]-result.totals.equivalent_output_torque_Nm[key])<1e-12);
const generalizedPower=Object.keys(qdot).reduce((sum,key)=>sum+result.totals.equivalent_output_torque_Nm[key]*qdot[key],0);
assert.ok(expectedPower<=0);assert.ok(Math.abs(generalizedPower-expectedPower)<1e-12);assert.ok(Math.abs(result.power_balance_residual_W)<1e-12);assert.ok(result.total_dissipated_power_W>=0);
console.log("leg pin planar friction simultaneous multi-output virtual-work test passed");
