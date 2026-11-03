import assert from "node:assert/strict";
import {existsSync,readFileSync} from "node:fs";

assert.ok(existsSync("j2-clearance.html"),"build produces an independent J2 clearance page");
const page=readFileSync("j2-clearance.html","utf8"),resistance=readFileSync("j2.html","utf8");
assert.match(page,/<title>J2 间隙与死区分析/);
assert.match(page,/src="j2-clearance-model\.js\?v=20261103-2"/);
assert.match(page,/href="j2\.html"[^>]*>阻力贡献<\/a>/);
assert.match(page,/href="j2-clearance\.html"[^>]*aria-current="page"[^>]*>间隙\/死区<\/a>/);
assert.match(resistance,/href="j2-clearance\.html"[^>]*>间隙\/死区<\/a>/);
for(const key of ["axis1","axis2","axis3","point2"]){
  for(const side of ["radial-positive","radial-negative","axial-positive","axial-negative"])
    assert.match(page,new RegExp(`data-j2-gap="${key}-${side}"`),`${key} exposes ${side}`);
}
assert.match(page,/data-j2-gap="rod-positive"/);assert.match(page,/data-j2-gap="rod-negative"/);
assert.match(page,/data-j2-gap="j2-positive"/);assert.match(page,/data-j2-gap="j2-negative"/);
for(const id of ["j2-gap-current","j2-gap-bias","j2-gap-peak","j2-gap-chart","j2-gap-ranking"])
  assert.match(page,new RegExp(`id="${id}"`));
assert.doesNotMatch(page,/理想 X–Z 二维模型中/);
assert.doesNotMatch(page,/为什么上端滑套、下端滑套与推杆背隙可能同灵敏度/);
assert.doesNotMatch(page,/δL = c<sub>r<\/sub>/);
for(const key of ["j2","axis1","axis2","axis3","point2","rod"])
  assert.match(page,new RegExp(`data-gap-demo="${key}"`),`${key} can be selected in the mechanism demonstration`);
for(const id of ["j2-gap-mechanism","j2-gap-magnifier","j2-gap-demo-contact","j2-gap-demo-difference"])
  assert.match(page,new RegExp(`id="${id}"`),`${id} is available to the live diagram`);
assert.match(page,/间隙位移为放大示意，数值计算仍使用真实输入/);
for(const label of ["反向贴合 q−","过渡空行程","正向贴合 q\+"])
  assert.match(page,new RegExp(label),`${label} is explained in the three-state graphic`);
assert.match(page,/J2等效旋转轴/);
assert.match(page,/line\(g\.point2,g\.lower/,'mechanism diagram draws the rigid point2-to-lower member');
for(const id of ["j2-screening-summary","j2-screening-sensitivity","j2-screening-contribution","j2-screening-curves","j2-screening-table"])
  assert.match(page,new RegExp(`id="${id}"`),`${id} is present in the leadership decision view`);
assert.match(page,/每次只改变一个连接副/);
assert.match(page,/<details[^>]*id="j2-gap-animation-details"(?![^>]*\sopen)/,'dynamic mechanism explanation is collapsed by default');
console.log("J2 clearance page regression test passed");
