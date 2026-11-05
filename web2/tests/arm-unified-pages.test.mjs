import assert from 'node:assert/strict';
import {existsSync,readFileSync} from 'node:fs';

for(const file of ['arm-runtime.mjs','left-arm-blackbox.mjs','data/arm/model.json','data/arm/pose_reference.json'])assert.ok(existsSync(new URL(`../dist/${file}`,import.meta.url)),`${file} must be built`);
const pages={};
for(let index=0;index<=6;index++){
  const id=`J${index}`,file=`arm-j${index}.html`;
  assert.ok(existsSync(new URL(`../dist/${file}`,import.meta.url)),`${file} must be built`);
  const page=readFileSync(new URL(`../dist/${file}`,import.meta.url),'utf8');pages[id]=page;
  assert.match(page,new RegExp(`手臂 ${id}`));
  for(let tab=0;tab<=6;tab++)assert.match(page,new RegExp(`href="arm-j${tab}\\.html"[^>]*>J${tab}<`));
  for(const marker of ['data-arm-svg','data-arm-output','data-arm-status','data-arm-residual','data-arm-active-inputs','data-arm-loads','data-arm-live-bars','data-arm-cycle-bars','data-arm-torque-composition','data-arm-mass-table'])assert.match(page,new RegExp(marker),`${id} requires ${marker}`);
  for(const label of ['1 Hz、±10°','闭链残差','静摩擦启动力矩','运行阻力','完整周期贡献度','连接副阻力','重力矩','惯性力矩','总力矩','18 个左臂刚体','2026-07-28 CMD'])assert.match(page,new RegExp(label));
  assert.match(page,/createArmRuntime/);
  assert.doesNotMatch(page,/src="arm-j[356]-model\.js\?/,'unified pages must not load legacy arm models');
  assert.doesNotMatch(page,/Hand_L[^<]*kg|waist_link3_3_shoulder_L[^<]*kg/);
}
for(const id of ['J0','J1','J2','J4'])assert.match(pages[id],/真实旋转轴主动/);
assert.match(pages.J3,/肘部单推杆主动/);assert.match(pages.J3,/输出轴被动/);assert.match(pages.J3,/肘推杆行程/);
for(const id of ['J5','J6']){assert.match(pages[id],/腕部双推杆主动/);assert.match(pages[id],/输出轴被动/);assert.match(pages[id],/左腕推杆行程/);assert.match(pages[id],/右腕推杆行程/)}
const index=readFileSync(new URL('../dist/index.html',import.meta.url),'utf8');
assert.match(index,/id="arm"[\s\S]*href="arm-j0\.html"/);
assert.match(index,/已建立 J0–J6/);
console.log('arm unified page regression test passed');
