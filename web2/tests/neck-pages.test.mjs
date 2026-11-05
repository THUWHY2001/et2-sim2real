import assert from "node:assert/strict";
import {existsSync,readFileSync} from "node:fs";

for(const file of ["neck-j0.html","neck-j1.html","neck-model.js","neck-3d-view.js"])assert.ok(existsSync(new URL(`../dist/${file}`,import.meta.url)),`${file} must be built`);
const j0=readFileSync(new URL("../dist/neck-j0.html",import.meta.url),"utf8");
const j1=readFileSync(new URL("../dist/neck-j1.html",import.meta.url),"utf8");
const index=readFileSync(new URL("../dist/index.html",import.meta.url),"utf8");

assert.match(index,/href="neck-j0\.html"/);
assert.match(index,/href="neck-j1\.html"/);
for(const [page,joint,motion] of [[j0,"J0","Roll"],[j1,"J1","Pitch"]]){
  assert.match(page,new RegExp(`头颈 ${joint} ${motion}`));
  assert.match(page,/src="neck-model\.js\?/);
  assert.match(page,/src="neck-3d-view\.js\?/);
  assert.match(page,/三维硬点视图/);
  assert.match(page,/拖拽旋转 · 滚轮缩放/);
  assert.match(page,/data-neck-reset-view/);
  assert.match(page,/data-neck-axis=/);
  assert.match(page,/'X','#c43b32'/);
  assert.match(page,/'Y','#2f7d4a'/);
  assert.match(page,/'Z','#235ea7'/);
  assert.match(page,/pointerdown/);
  assert.match(page,/pointermove/);
  assert.match(page,/wheel/);
  assert.match(page,/重力矩/);
  assert.match(page,/摩擦贡献度/);
  assert.match(page,/预紧力仅记录实测工况/);
  assert.match(page,/载荷修正系数 k_N（N·m\/N）/);
  assert.match(page,/data-neck-kn=/);
  assert.doesNotMatch(page,/<iframe|data-srcdoc=/);
}
assert.match(j0,/R1/);assert.match(j0,/R2/);
assert.match(j0,/R1 \(-52,0,479\.66\) → R2 \(-40,0,479\.66\)/);
assert.match(j1,/P1/);assert.match(j1,/P2/);

console.log("neck page regression test passed");
