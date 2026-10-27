import assert from "node:assert/strict";
import {readFileSync} from "node:fs";

const cases=[
  {file:"src/j3.fragment.html",keys:["JP","PA","AQ","JQ"],id:"j3"},
  {file:"src/j4.fragment.html",keys:["JB"],id:"j4"},
  {file:"src/arm-j3.fragment.html",keys:["L","Z","K"],id:"arm-j3"},
];

for(const page of cases){
  const source=readFileSync(page.file,"utf8");
  assert.match(source,/连杆长度调节模块/,`${page.id} exposes the adjustment module`);
  assert.match(source,new RegExp(`id="${page.id}-length-status"`),`${page.id} exposes variation status`);
  assert.match(source,new RegExp(`id="${page.id}-length-reset"`),`${page.id} exposes reset`);
  for(const key of page.keys)assert.match(source,new RegExp(`data-link-length="${key}"`),`${page.id} exposes ${key} delta input`);
  assert.match(source,/LinkLengthControls\.mount/,`${page.id} mounts the shared controller`);
  assert.match(source,/M\.solve\([^\n]*currentLengths\(\)/,`${page.id} solves its mechanism with active lengths`);
}
console.log("joint link length page regression test passed");
