import assert from "node:assert/strict";
import {solveAssembledSystem} from "../src/leg-gravity-statics.mjs";

const massKg=2,gravity=9.80665,leverArmM=.5,weight=massKg*gravity;
const A=Array.from({length:6},(_,row)=>Array.from({length:6},(_,column)=>row===column?1:0));
const b=[0,0,weight,0,-weight*leverArmM,0];
const labels=["Fx","Fy","Fz","Mx","My","Mz"];
const assembly={
  A,b,
  equations:labels.map(component=>({body:"analytic_link",component})),
  unknowns:labels.map((label,index)=>({id:`support:${label}`,joint_id:"support",category:"passive_constraint_wrench",basis_label:label,analyticIndex:index})),
  joints:[{joint_id:"support",joint_type:"revolute",motion_state:"position-held"}],
  bodies:["analytic_link"],
  boundary:{worldInterface:"support",footContact:"none",loadCase:"analytic-gravity"},
};

const result=solveAssembledSystem(assembly);
assert.equal(result.matrix.consistent,true);
assert.equal(result.matrix.rank,6);
assert.equal(result.matrix.nullity,0);
assert.ok(result.matrix.residual_norm<1e-10);
for(let index=0;index<6;index++){
  assert.equal(result.unknownValues[index].status,"unique");
  assert.ok(Math.abs(result.unknownValues[index].value-b[index])<1e-10);
}
assert.ok(Math.abs(result.unknownValues[2].value-weight)<1e-10,"support vertical reaction must equal weight");
assert.ok(Math.abs(result.unknownValues[4].value+weight*leverArmM)<1e-10,"support holding moment must equal hand calculation");
assert.ok(Math.hypot(...result.residuals.byBody.analytic_link)<1e-10);

console.log("analytic gravity statics benchmark passed");
