import assert from "node:assert/strict";
import {readFileSync} from "node:fs";

const page=readFileSync(new URL("../j2.html",import.meta.url),"utf8");
assert.match(page,/ET2 左腿完整三推杆闭链/);
assert.match(page,/data-leg-full-svg/);
assert.match(page,/拖拽旋转，滚轮缩放/);
assert.doesNotMatch(page,/J2 二维机构简图（X–Z 投影）/);

console.log("J2 full 3D replacement test passed");
