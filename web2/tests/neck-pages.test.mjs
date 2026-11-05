import assert from 'node:assert/strict';
import {existsSync,readFileSync} from 'node:fs';

for(const file of ['neck-j0.html','neck-j1.html','neck-j2.html','neck-combined.html','neck-runtime.mjs','neck-blackbox.mjs','data/neck/model.json','neck-3d-view.js'])
  assert.ok(existsSync(new URL(`../dist/${file}`,import.meta.url)),`${file} must be built`);
const pages=Object.fromEntries(['j0','j1','j2','combined'].map(id=>[id,readFileSync(new URL(`../dist/neck-${id}.html`,import.meta.url),'utf8')]));
const index=readFileSync(new URL('../dist/index.html',import.meta.url),'utf8');
for(const id of ['j0','j1','j2','combined'])assert.match(index,new RegExp(`href="neck-${id}\\.html"`));

for(const [id,motion] of [['j0','Roll'],['j1','Pitch'],['j2','Yaw']]){
  const page=pages[id];
  assert.match(page,new RegExp(`头颈 J${id.at(-1)} ${motion} 阻力分析`));
  for(const marker of ['data-neck-svg','data-neck-output','data-neck-play','data-neck-status','data-neck-residual','data-neck-left-extension','data-neck-right-extension','data-neck-loads','data-neck-live','data-neck-cycle','data-neck-load-settings','data-neck-loaded-live','data-neck-loaded-cycle','data-neck-torque-composition','data-neck-mass-table'])
    assert.match(page,new RegExp(marker),`${id} requires ${marker}`);
  for(const label of ['左推杆行程','右推杆行程','被动输出','实时贡献度','完整周期贡献度','加载后实时贡献度','加载后完整周期贡献度','总力矩组成','连接副阻力','重力矩','惯性力矩','总力矩','临时简化'])assert.match(page,new RegExp(label));
  assert.match(page,/createNeckRuntime/);
  assert.match(page,/solveOutput|solveInputs/);
  assert.doesNotMatch(page,/src="neck-model\.js\?/,'main neck pages must not load the legacy simplified model');
  assert.doesNotMatch(page,/<iframe|data-srcdoc=/);
}
for(const marker of ['data-neck-active-yaw','data-neck-active-left','data-neck-active-right','data-neck-passive-roll','data-neck-passive-pitch','data-neck-residual'])assert.match(pages.combined,new RegExp(marker));
assert.match(pages.combined,/三个主动输入/);
assert.match(pages.combined,/Roll\/Pitch 被动输出/);
console.log('neck page regression test passed');
