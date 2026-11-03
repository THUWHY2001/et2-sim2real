import assert from "node:assert/strict";
import {readFileSync} from "node:fs";

const j2=readFileSync("src/j2.fragment.html","utf8");
const clearance=readFileSync("src/j2-clearance.fragment.html","utf8");
const j3=readFileSync("src/j3.fragment.html","utf8");
const j4=readFileSync("src/j4.fragment.html","utf8");

assert.match(j2,/推杆上端滑动轴承/);
assert.match(j2,/推杆下端滑动轴承/);
assert.match(clearance,/推杆上端滑动轴承/);
assert.match(clearance,/推杆下端滑动轴承/);
assert.match(j3,/A端滑动轴承/);
assert.match(j3,/B端滑动轴承/);
assert.match(j3,/data-load="b-positive"/);
assert.match(j3,/data-run="b-positive-c"/);
assert.match(j3,/rates=\{[\s\S]*?b:/);
assert.match(j4,/A端滑动轴承/);
assert.match(j4,/B端滑动轴承/);
for(const source of [j2,clearance,j3,j4]) assert.doesNotMatch(source,/鱼眼|滑套轴承/);

assert.match(clearance,/主机构图保持真实几何/);
assert.doesNotMatch(clearance,/ghost\.axis3/);
assert.doesNotMatch(clearance,/posV=/);

console.log("Leg pushrod sleeve-bearing regression test passed");
