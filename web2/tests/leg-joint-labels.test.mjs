import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import {createLegJointCatalog} from "../src/leg-joint-labels.mjs";

const model=JSON.parse(readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8"));
const catalog=createLegJointCatalog(model);

assert.equal(catalog.entries.length,25);
assert.deepEqual(catalog.entries.map(item=>item.code),Array.from({length:25},(_,index)=>`L${String(index+1).padStart(2,"0")}`));
assert.equal(new Set(catalog.entries.map(item=>item.key)).size,25);
assert.equal(new Set(catalog.entries.map(item=>item.code)).size,25);

const hipPitch=catalog.meta("J_LEG_L_HIP_pitch_RE");
assert.equal(hipPitch.code,"L03");
assert.equal(hipPitch.name,"J2 髋 Pitch 输出副");
assert.equal(hipPitch.display,"L03 J2 髋 Pitch 输出副");
assert.equal(hipPitch.typeLabel,"旋转副");
assert.equal(hipPitch.connection,"leg_l_link2 ↔ leg_l_link3");

const ankleRoll=catalog.meta("J_LEG_L_ANKLE_roll_RE");
assert.equal(ankleRoll.code,"L25");
assert.equal(ankleRoll.name,"J5 踝 Roll 旋转副");

assert.throws(()=>catalog.meta("missing_joint"),/Unknown leg joint/);
console.log("leg joint unified-code regression test passed");
