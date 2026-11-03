import assert from "node:assert/strict";
import {existsSync,readFileSync} from "node:fs";

for(const file of ["neck-j0.html","neck-j1.html","neck-model.js"])assert.ok(existsSync(new URL(`../dist/${file}`,import.meta.url)),`${file} must be built`);
const j0=readFileSync(new URL("../dist/neck-j0.html",import.meta.url),"utf8");
const j1=readFileSync(new URL("../dist/neck-j1.html",import.meta.url),"utf8");
const index=readFileSync(new URL("../dist/index.html",import.meta.url),"utf8");

assert.match(index,/href="neck-j0\.html"/);
assert.match(index,/href="neck-j1\.html"/);
for(const [page,joint,motion,view] of [[j0,"J0","Roll","Y–Z"],[j1,"J1","Pitch","X–Z"]]){
  assert.match(page,new RegExp(`头颈 ${joint} ${motion}`));
  assert.match(page,/src="neck-model\.js\?/);
  assert.match(page,new RegExp(`${view} 投影`));
  assert.match(page,/重力矩/);
  assert.match(page,/摩擦贡献度/);
  assert.match(page,/预紧力仅记录实测工况/);
  assert.match(page,/载荷修正系数 k_N（N·m\/N）/);
  assert.match(page,/data-neck-kn=/);
  assert.doesNotMatch(page,/<iframe|data-srcdoc=/);
}
assert.match(j0,/R1/);assert.match(j0,/R2/);
assert.match(j1,/P1/);assert.match(j1,/P2/);

console.log("neck page regression test passed");
