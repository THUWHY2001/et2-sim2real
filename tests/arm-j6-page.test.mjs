import assert from "node:assert/strict";
import {existsSync,readFileSync} from "node:fs";

assert.ok(existsSync("arm-j6.html"),"build produces arm J6 page");
const page=readFileSync("arm-j6.html","utf8"),index=readFileSync("index.html","utf8"),j5=readFileSync("arm-j5.html","utf8");
assert.match(page,/<title>手臂 J6/);
assert.match(page,/src="arm-j6-model\.js\?v=[^"]+"/);
assert.match(page,/\(25\.282,328\.524,−106\.969\)—\(25\.869,355\.606,−96\.874\)/);
assert.match(page,/∠J6A-下1-上1/);
assert.match(page,/∠J6B-下2-上2/);
assert.match(page,/两根推杆反向伸缩/);
assert.match(page,/href="arm-j6\.html"[^>]*aria-current="page"[^>]*>J6</);
assert.match(j5,/href="arm-j6\.html"[^>]*>J6</);
assert.match(index,/href="arm-j6\.html"[\s\S]*手臂 J6/);
console.log("arm J6 page regression test passed");
