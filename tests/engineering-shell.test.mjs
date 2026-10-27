import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

const read = name => readFileSync(new URL(`../dist/${name}`, import.meta.url), "utf8");
const index = read("index.html");

assert.match(index, /模块连接副-等效旋转轴的灵敏度分析/);
assert.match(index, /class="site-nav"/);
assert.match(index, /腿足/);
assert.match(index, /头颈/);
assert.match(index, /躯干/);
assert.match(index, /手臂/);
assert.match(index, /class="joint-options"[\s\S]*J2[\s\S]*J3[\s\S]*J4/);
assert.match(index, /id="arm"[\s\S]*href="arm-j3\.html"/);
assert.match(index, /href="site\.css\?v=[^"]+"/, "published pages must version the stylesheet so structural CSS updates are not served from stale cache");
assert.match(index, /href="site\.css\?v=20261027-1"/, "link-length controls must publish with the new stylesheet version");
assert.doesNotMatch(index, /radial-gradient|Sim2Real · 理论计算模型|ChatGPT|人工智能|>AI</i);

for (const [file, label] of [["j2.html", "J2"], ["j3.html", "J3"], ["j4.html", "J4"]]) {
  const page = read(file);
  assert.match(page, /class="site-nav"/);
  assert.match(page, /aria-current="page">腿足</);
  assert.match(page, new RegExp(`class="joint-tab"[^>]*aria-current="page">${label}<`));
  assert.doesNotMatch(page, /<iframe|data-srcdoc=/);
  assert.match(page, /src="link-length-controls\.js\?v=20261027-1"/, `${file} loads the versioned link-length controller`);
  assert.ok(Buffer.byteLength(page) < 80_000, `${file} should be a lean direct page`);
}

const arm = read("arm-j3.html");
assert.match(arm, /aria-current="page">手臂</);
assert.match(arm, /class="joint-tab"[^>]*aria-current="page">J3</);
assert.doesNotMatch(arm, /<iframe|data-srcdoc=/);
assert.ok(Buffer.byteLength(arm) < 80_000, "arm-j3.html should be a lean direct page");

console.log("engineering shell regression test passed");
