import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

const page = readFileSync(new URL("../dist/j2.html", import.meta.url), "utf8");

assert.match(page, /<title>腿足 J2 Hip Pitch 阻力分析/);
assert.doesNotMatch(page, /<iframe|data-srcdoc=/, "J2 must render directly instead of decoding a nested srcdoc iframe");
assert.match(page, /data-leg-full-joint="J2"/);
assert.match(page, /20 个运动刚体/);
assert.match(page, /23 条硬点记录/);
assert.match(page, /50 条运动副 Marker/);
assert.match(page, /M\.solveOutput\(joint,value\)/);
assert.doesNotMatch(page, /directAngles\(/, "J2 page must not directly drive the passive output angle");

console.log("J2 full closed-chain direct-page regression test passed");
