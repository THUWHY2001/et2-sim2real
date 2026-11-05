import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createNeckRuntime} from '../src/neck-runtime.mjs';

const data=JSON.parse(readFileSync(new URL('../data/neck/model.json',import.meta.url),'utf8'));
const runtime=createNeckRuntime(data);
const validation=runtime.validate();
assert.equal(validation.pass,true);
assert.equal(validation.mass_kg,2.8109963);
assert.ok(validation.roundtrip_residual_mm<1e-9);

const pitch=runtime.solveInputs({leftMm:1,rightMm:1,yawDeg:0});
assert.equal(pitch.complete,true);
assert.ok(Math.abs(pitch.rollDeg)<1e-10);
assert.ok(Math.abs(pitch.pitchDeg-2.5694080027763526)<1e-9);
assert.ok(pitch.residualMm<1e-9);

const roll=runtime.solveInputs({leftMm:1,rightMm:-1,yawDeg:0});
assert.equal(roll.complete,true);
assert.ok(Math.abs(roll.rollDeg-1.8499226752610904)<1e-9);
assert.ok(Math.abs(roll.pitchDeg-.009610214766596384)<1e-9);

const output=runtime.solveOutput('J0',5);
assert.equal(output.complete,true);
assert.ok(Math.abs(output.rollDeg-5)<1e-8);
assert.ok(output.residualMm<1e-9);
const yaw=runtime.solveInputs({leftMm:output.leftMm,rightMm:output.rightMm,yawDeg:7});
assert.ok(Math.abs(yaw.rollDeg-output.rollDeg)<1e-10);
assert.ok(Math.abs(yaw.pitchDeg-output.pitchDeg)<1e-10);
assert.equal(yaw.yawDeg,7);

assert.throws(()=>runtime.solveOutput('J9',0),/未知头颈关节/);
console.log('neck blackbox runtime acceptance test passed');
