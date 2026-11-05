import assert from "node:assert/strict";
import {readFileSync} from "node:fs";

const index=readFileSync("index.html","utf8");
const page=readFileSync("arm-j3.html","utf8");

assert.match(index,/id="arm"[\s\S]*href="arm-j3\.html"[\s\S]*手臂 J3/);
assert.match(page,/<title>手臂 J3/);
assert.match(page,/src="directional-friction\.js\?v=[^"]+"/);
assert.match(page,/src="arm-j3-model\.js\?v=[^"]+"/);
assert.match(page,/data-arm-static="p-positive"/);
assert.match(page,/data-arm-static="p-negative"/);
assert.match(page,/data-arm-static="rod-positive"/);
assert.match(page,/data-arm-static="rod-negative"/);
assert.match(page,/data-arm-run="b-negative-c"/);
assert.match(page,/1 Hz[\s\S]*±10°/);
assert.match(page,/data-chart-panel/);
assert.match(page,/data-chart-action="in"/);
assert.match(page,/data-chart-action="fullscreen"/);
assert.match(page,/P副[\s\S]*Q副[\s\S]*R副[\s\S]*B端鱼眼[\s\S]*推杆/);
assert.match(page,/手臂J3全局X-Z投影机构图/);
assert.match(page,/>Z \/ mm</);
assert.match(page,/>X \/ mm</);
assert.doesNotMatch(page,/id="arm-j3-axis"|\.arm-axis\{/);
assert.doesNotMatch(page,/局部 U|局部 V|J3法向局部平面/);
assert.match(page,/A端Q→B端鱼眼/);
assert.doesNotMatch(page,/Q→T|>T<|arm-j3-body/);

console.log("arm J3 page regression test passed");
