import assert from "node:assert/strict";
import {readFileSync} from "node:fs";

const build=readFileSync(new URL("../scripts/build-pages.mjs",import.meta.url),"utf8"),fragment=readFileSync(new URL("../src/leg-pin-planar-friction.fragment.html",import.meta.url),"utf8");
assert.match(build,/id:"leg-friction"/);assert.match(build,/leg-pin-planar-friction-page\.mjs/);assert.match(build,/leg-pin-planar-friction\.mjs/);
for(const text of ["腿足理论摩擦圆分析","重力准静态","不包含 1 Hz 惯性","物理接触界面映射","real_joint_type_unverified","real_friction_interface_unverified","演示参数","不是工程参数","连接副合反力","导向接触法向载荷","假设黏着状态","恢复参数为空","输出轴等效阻力","耗散功率","导出 CSV"])assert.match(fragment,new RegExp(text));
const page=readFileSync(new URL("../src/leg-pin-planar-friction-page.mjs",import.meta.url),"utf8");assert.match(page,/software_verification_example_not_measurement/);assert.match(page,/CSV 已导出/);
for(const marker of ["data-leg-friction-output","data-leg-friction-phase","data-leg-friction-contact-table","data-leg-friction-summary","data-leg-friction-bars","data-leg-friction-circle","data-leg-friction-cycle-chart","data-leg-friction-export"])assert.match(fragment,new RegExp(marker));
assert.match(fragment,/leg-pin-planar-friction-page\.mjs/);assert.doesNotMatch(fragment,/真实整腿总运行阻力/);
const home=readFileSync(new URL("../index.html",import.meta.url),"utf8");assert.match(home,/href="leg-friction\.html"[^>]*>[\s\S]*理论摩擦/);
console.log("leg pin planar friction page regression test passed");
