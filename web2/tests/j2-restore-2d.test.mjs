import assert from "node:assert/strict";
import {readFileSync} from "node:fs";

const page=readFileSync(new URL("../j2.html",import.meta.url),"utf8");
assert.match(page,/J2 二维机构简图（X–Z 投影）/);
assert.match(page,/aria-labelledby="leg-viz-title leg-viz-desc"/);
assert.doesNotMatch(page,/j2-3d-canvas|j2-3d-view\.js/);

console.log("J2 2D restoration test passed");
