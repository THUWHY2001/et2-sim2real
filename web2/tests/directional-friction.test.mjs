import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const helperSource = readFileSync(new URL("../src/directional-friction.js", import.meta.url), "utf8");
const context = { window: {} };
vm.runInNewContext(helperSource, context);
const D = context.window.DirectionalFriction;

assert.equal(D.directionKey(2), "positive", "positive local velocity selects the positive branch");
assert.equal(D.directionKey(-2), "negative", "negative local velocity selects the negative branch");
assert.equal(D.directionKey(0, -1), "negative", "zero speed uses the requested start direction");
assert.equal(D.directionalValue({ positive: 3, negative: 5 }, 1), 3);
assert.equal(D.directionalValue({ positive: 3, negative: 5 }, -1), 5);
assert.equal(D.frictionMagnitude({
  positive: { coulomb: 2, viscous: 4 },
  negative: { coulomb: 7, viscous: 3 },
}, 0.5), 4);
assert.equal(D.frictionMagnitude({
  positive: { coulomb: 2, viscous: 4 },
  negative: { coulomb: 7, viscous: 3 },
}, -0.5), 8.5);

for (const file of ["j2.html", "j3.html", "j4.html", "arm-j3.html"]) {
  const page = readFileSync(new URL(`../dist/${file}`, import.meta.url), "utf8");
  assert.match(page, /data-start-direction/, `${file} must expose an equivalent-axis start direction selector`);
  assert.match(page, /正向/, `${file} must label positive-direction inputs`);
  assert.match(page, /反向/, `${file} must label negative-direction inputs`);
  assert.match(page, /directional-friction\.js/, `${file} must load the shared direction model`);
}

console.log("directional friction regression test passed");
