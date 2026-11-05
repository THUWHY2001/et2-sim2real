import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { createArmRuntime } from "../src/arm-runtime.mjs";

const model = JSON.parse(readFileSync(new URL("../data/arm/model.json", import.meta.url), "utf8"));
const reference = JSON.parse(readFileSync(new URL("../data/arm/pose_reference.json", import.meta.url), "utf8"));
const runtime = createArmRuntime(model, reference);
const standardJ0 = runtime.kinematics(runtime.solveOutput("J0", 0)).jointCentersMm.J0;

for (const joint of ["J0", "J1", "J2", "J3", "J4", "J5", "J6"]) {
  for (const angle of [-10, 10]) {
    const actual = runtime.kinematics(runtime.solveOutput(joint, angle)).jointCentersMm.J0;
    assert.deepEqual(actual, standardJ0, `${joint} excitation must not move the J0 installation point`);
  }
}

const page = readFileSync(new URL("../arm-j0.html", import.meta.url), "utf8");
assert.match(page, /fixedViewCenter/, "the 3D camera must use the standard-pose view center");
assert.match(page, /standardBounds/, "the fixed camera center must come from the standard-pose bounding box so all hardpoints remain visible");
assert.doesNotMatch(page, /const center=points\.reduce/, "the camera must not recenter on the moving mechanism every frame");
for (const label of [
  "肘固定支点",
  "肘输出连接点",
  "肘推杆下端",
  "肘推杆上端",
  "腕左推杆上端",
  "腕左推杆下端",
  "腕右推杆上端",
  "腕右推杆下端",
]) {
  assert.match(page, new RegExp(label), `the 3D mechanism must label ${label}`);
}
assert.match(page, /data-arm-point-key/, "the page must expose a hardpoint-to-joint correspondence list");

console.log("arm fixed J0 installation-point regression test passed");
