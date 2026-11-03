import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

const page = readFileSync(new URL("../dist/j2.html", import.meta.url), "utf8");

assert.match(page, /<title>J2 三维机构简图/);
assert.doesNotMatch(page, /<iframe|data-srcdoc=/, "J2 must render directly instead of decoding a nested srcdoc iframe");
assert.match(page, /id="leg-2d-mechanism"/);
assert.match(page, /src="j2-3d-view\.js\?/);
assert.match(page, /id="j2-3d-canvas"/);
assert.match(page, /data-j2-view="side"/);
assert.match(page, /data-j2-view="reset"/);
assert.doesNotMatch(page, /aria-labelledby="leg-viz-title leg-viz-desc"/, "old main 2D SVG must be replaced");
for (const key of ["j2Axis3","axis1Axis2","axis2Axis3","axis3Lower","point2Lower"]) {
  assert.match(page, new RegExp(`data-j2-length="${key}"`));
}
assert.match(page, /id="j2-length-reset"/);
assert.match(page, /id="j2-length-status"/);
assert.ok(Buffer.byteLength(page) < 80_000, "J2 direct page should stay below 80 KB");

console.log("J2 direct-page regression test passed");
