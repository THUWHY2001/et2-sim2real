import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import {createLegFullRuntime} from "../src/leg-full-runtime.mjs";

const model=JSON.parse(readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8"));
const leg=createLegFullRuntime(model);
const near=(actual,expected,tolerance=1e-8,message="")=>assert.ok(Math.abs(actual-expected)<=tolerance,`${message} expected ${expected}, got ${actual}`);

const validation=leg.validate();
assert.equal(validation.ok,true);
assert.equal(validation.parts,20);
assert.equal(validation.joints,25);
assert.equal(validation.all_joint_marker_sides,50);

const nominal=leg.solveRods({hip_mm:0,knee_mm:0,ankle_mm:0});
near(nominal.outputs.J2,-27.5,1e-9,"nominal J2");
near(nominal.outputs.J3,51.57,1e-9,"nominal J3");
near(nominal.outputs.J4,-24.06,1e-9,"nominal J4");
assert.equal(nominal.frame.all_hardpoints.length,23);
assert.equal(nominal.frame.all_joint_marker_sides.length,50);
assert.equal(Object.keys(nominal.frame.world_transforms).length,21);
assert.equal(Object.keys(nominal.frame.mass.COM_world_mm).length,20);
near(nominal.frame.mass.total_kg,13.02480932,1e-10,"leg mass");

const plusOne=leg.solveRods({hip_mm:1,knee_mm:1,ankle_mm:1});
near(plusOne.outputs.J2,-25.932618659,2e-9,"+1 mm J2");
near(plusOne.outputs.J3,50.052188561,2e-9,"+1 mm J3");
near(plusOne.outputs.J4,-25.530447752,2e-9,"+1 mm J4");
assert.ok(plusOne.frame.validation.max_prismatic_lateral_error_mm<0.002);

const j2Command=leg.solveOutput("J2",10);
near(j2Command.outputs.J2,-17.5,1e-7,"J2 relative output command");
near(j2Command.rodStrokes.knee_mm,0,1e-10);
near(j2Command.rodStrokes.ankle_mm,0,1e-10);
assert.notEqual(j2Command.rodStrokes.hip_mm,0);

const rollCoupled=leg.solveRods({J5_deg:5});
near(rollCoupled.outputs.J4,-23.842537,2e-5,"J5 to J4 coupling");

console.log("leg full closed-chain runtime acceptance test passed");
