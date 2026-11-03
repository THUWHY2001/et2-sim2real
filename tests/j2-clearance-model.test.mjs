import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

const context={window:{}};vm.createContext(context);
vm.runInContext(readFileSync("src/j2-clearance-model.js","utf8"),context);
const M=context.window.J2ClearanceModel;
const passive=["axis1","axis2","axis3","point2"];
const empty=()=>Object.fromEntries([...passive.map(k=>[k,{radialPositive:0,radialNegative:0,axialPositive:0,axialNegative:0}]),["rod",{positive:0,negative:0}]]);

const zero=M.analyze(0,empty());
assert.ok(Math.abs(zero.combined.deadzone)<1e-10);
assert.ok(Math.abs(zero.combined.bias)<1e-10);

const rodGaps=empty();rodGaps.rod={positive:.2,negative:.1};
const rod=M.analyze(0,rodGaps);
assert.ok(rod.individual.rod.deadzone>0,"pushrod backlash creates J2 dead-zone");
assert.ok(Math.abs(rod.individual.rod.bias)>0,"asymmetric pushrod backlash creates zero bias");

const passiveGaps=empty();passiveGaps.axis2.radialPositive=.1;passiveGaps.axis2.radialNegative=.2;
const axis2=M.analyze(0,passiveGaps);
assert.ok(axis2.individual.axis2.deadzone>0,"passive radial clearance creates J2 dead-zone");
assert.ok(Number.isFinite(axis2.individual.axis2.sensitivity));

const axialOnly=empty();axialOnly.axis2.axialPositive=1;axialOnly.axis2.axialNegative=2;
assert.ok(Math.abs(M.analyze(0,axialOnly).combined.deadzone)<1e-10,"ideal X-Z model keeps axial interface inactive");

const all=empty();for(const k of passive){all[k].radialPositive=.1;all[k].radialNegative=.15}all.rod={positive:.2,negative:.1};
const cycle=M.cycle(all);
assert.equal(cycle.forward.length,81);assert.equal(cycle.reverse.length,81);
assert.ok(cycle.peakDeadzone>0);assert.ok(Number.isFinite(cycle.peakAngle));
assert.ok(Math.abs(Object.values(M.analyze(0,all).contribution).reduce((a,b)=>a+b,0)-100)<1e-6);

const zeroPresentation=M.presentation(0,empty(),"axis2");
assert.equal(zeroPresentation.type,"radial");
assert.equal(zeroPresentation.positive.gap,0);
assert.equal(zeroPresentation.negative.gap,0);
assert.deepEqual(zeroPresentation.positive.geometry.axis2Rod,zeroPresentation.negative.geometry.axis2Rod);

const asymmetric=empty();asymmetric.axis2.radialPositive=.2;asymmetric.axis2.radialNegative=.05;
const axis2Presentation=M.presentation(0,asymmetric,"axis2");
assert.equal(axis2Presentation.positive.gap,.2);
assert.equal(axis2Presentation.negative.gap,.05);
assert.ok(axis2Presentation.positive.actual>axis2Presentation.negative.actual);
assert.ok(Number.isFinite(axis2Presentation.direction.x));

const rodPresentation=M.presentation(0,rodGaps,"rod");
assert.equal(rodPresentation.type,"axial");
assert.equal(rodPresentation.positive.gap,.2);
assert.equal(rodPresentation.negative.gap,.1);

console.log("J2 clearance model regression test passed");
