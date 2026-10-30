import assert from "node:assert/strict";
import {existsSync,readFileSync} from "node:fs";

assert.ok(existsSync("arm-j5.html"),"build must produce a standalone arm J5 page");
const page=readFileSync("arm-j5.html","utf8"),index=readFileSync("index.html","utf8"),armJ3=readFileSync("arm-j3.html","utf8");

assert.match(page,/<title>手臂 J5/);
assert.match(page,/src="arm-j5-model\.js\?v=[^"]+"/);
assert.match(page,/手臂J5全局X-Z投影机构图/);
for(const text of ["J5 等效轴","连杆1","连杆2","推杆1","推杆2","上端鱼眼1","下端鱼眼1","上端鱼眼2","下端鱼眼2"])
  assert.match(page,new RegExp(text),`page exposes ${text}`);
for(const key of ["upper1","lower1","upper2","lower2","rod1","rod2"]){
  assert.match(page,new RegExp(`data-arm-j5-static="${key}-positive"`),`${key} has forward static input`);
  assert.match(page,new RegExp(`data-arm-j5-static="${key}-negative"`),`${key} has reverse static input`);
  assert.match(page,new RegExp(`data-arm-j5-run="${key}-positive-c"`),`${key} has running input`);
}
assert.match(page,/运行阻力（1 Hz、±10°）/);
assert.match(page,/连杆长度调节模块/);
assert.match(page,/data-link-length="link1"/);
assert.match(page,/data-link-length="link2"/);
assert.match(page,/id="arm-j5-tolerance-charts"/);
assert.match(page,/data-chart-panel/);
assert.match(index,/href="arm-j5\.html"[\s\S]*手臂 J5/);
assert.match(armJ3,/href="arm-j5\.html"[^>]*>J5</);
assert.match(page,/href="arm-j3\.html"[^>]*>J3</);
assert.match(page,/href="arm-j5\.html"[^>]*aria-current="page"[^>]*>J5</);

console.log("arm J5 page regression test passed");
