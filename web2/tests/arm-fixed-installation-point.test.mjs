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
for (const code of ["E1", "E2", "E3", "E4", "W1", "W2", "W3", "W4", "T1", "T2", "T3"]) {
  assert.match(page, new RegExp(`${code} [^']+`), `the mechanism and contribution names must share code ${code}`);
}
assert.match(page, /data-arm-point-key/, "the page must expose a hardpoint-to-joint correspondence list");

const css = readFileSync(new URL("../site.css", import.meta.url), "utf8");
assert.match(css, /\.arm-unified-geometry text\{[^}]*font-size:9px/, "mechanism labels must use the compact 9 px size");
assert.match(css, /\.arm-hardpoint text\{[^}]*font-size:8px/, "hardpoint labels must use the compact 8 px size");
assert.match(css, /\.arm-line-label\{[^}]*font-size:8px/, "transmission labels must use the compact 8 px size");
assert.match(page, /class="arm-world-axis"/, "mechanism view must include a global XYZ coordinate triad");
assert.match(page, />全局坐标系</, "global coordinate triad must be named in the figure");
assert.match(page, /point=\(p,code\)=>/, "hardpoints in the mechanism figure must show only their short codes");

console.log("arm fixed J0 installation-point regression test passed");
