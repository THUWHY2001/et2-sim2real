import assert from "node:assert/strict";
import {existsSync,readFileSync} from "node:fs";

for(const file of ["neck-j0-tolerance.html","neck-j1-tolerance.html","neck-tolerance-model.js"]){
  assert.ok(existsSync(new URL(`../dist/${file}`,import.meta.url)),`${file} must be built`);
}
const j0=readFileSync(new URL("../dist/neck-j0-tolerance.html",import.meta.url),"utf8");
const j1=readFileSync(new URL("../dist/neck-j1-tolerance.html",import.meta.url),"utf8");
const baseJ0=readFileSync(new URL("../dist/neck-j0.html",import.meta.url),"utf8");
const baseJ1=readFileSync(new URL("../dist/neck-j1.html",import.meta.url),"utf8");

assert.match(baseJ0,/href="neck-j0-tolerance\.html"/);
assert.match(baseJ1,/href="neck-j1-tolerance\.html"/);
for(const [page,joint] of [[j0,"J0"],[j1,"J1"]]){
  assert.match(page,new RegExp(`头颈 ${joint} 硬点公差分析`));
  assert.match(page,/src="neck-model\.js\?/);
  assert.match(page,/src="neck-tolerance-model\.js\?/);
  assert.match(page,/src="neck-3d-view\.js\?/);
  assert.match(page,/半径 1 mm/);
  assert.match(page,/data-tolerance-point="\$\{key\}"/);
  for(const point of ["lower1","upper1","lower2","upper2","R1","R2","P1","P2"])assert.match(page,new RegExp(`${point}:`));
  assert.match(page,/data-reset-all/);
  assert.match(page,/data-project-offset/);
  assert.match(page,/data-find-worst/);
  assert.match(page,/静摩擦启动力矩/);
  assert.match(page,/动态运行阻力/);
  assert.match(page,/最敏感两点组合/);
  assert.match(page,/参数区间鲁棒排名/);
  assert.match(page,/第一名概率/);
  assert.match(page,/前三名概率/);
  assert.match(page,/data-range-lower/);
  assert.match(page,/data-range-upper/);
  assert.match(page,/基础阻力输入/);
  assert.match(page,/data-base-load=/);
  assert.match(page,/推杆轴向参数/);
  assert.match(page,/data-rod-parameter=/);
  assert.match(page,/标零后实际长度作为无轴向预载基准/);
  assert.match(page,/自定义公差状态分析/);
  assert.match(page,/data-run-custom/);
  assert.match(page,/data-custom-state/);
  assert.match(page,/data-custom-chart/);
  assert.match(page,/data-custom-static/);
  assert.match(page,/data-custom-dynamic/);
  assert.match(page,/data-custom-utilization/);
  assert.match(page,/data-custom-interaction/);
  assert.match(page,/请先设置至少一个非零硬点偏移/);
  assert.match(page,/完整动态周期分析/);
  assert.match(page,/data-run-dynamic/);
  assert.match(page,/data-cycle-state/);
  assert.match(page,/data-cycle-static-chart/);
  assert.match(page,/data-cycle-dynamic-chart/);
  assert.match(page,/data-cycle-static-peak/);
  assert.match(page,/data-cycle-dynamic-peak/);
  assert.match(page,/data-cycle-static-ranking/);
  assert.match(page,/data-cycle-dynamic-ranking/);
  assert.match(page,/最大阻力增量时刻/);
  assert.match(page,/标零销参数接口（预留）/);
  assert.match(page,/disabled[^>]*data-zero-pin/);
  assert.doesNotMatch(page,/新旧版本|旧版对比|贡献度差值|排名变化/);
  for(const [,script] of page.matchAll(/<script>([\s\S]*?)<\/script>/g))assert.doesNotThrow(()=>new Function(script));
}

console.log("neck tolerance page regression test passed");
