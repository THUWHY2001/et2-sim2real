import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

for (const file of ["j2.html", "j3.html", "j4.html", "arm-j3.html"]) {
  const page = readFileSync(new URL(`../dist/${file}`, import.meta.url), "utf8");
  assert.match(page, /data-chart-panel/, `${file} must expose a resizable contribution chart panel`);
  for (const action of ["out", "reset", "in", "fullscreen"]) {
    assert.match(page, new RegExp(`data-chart-action="${action}"`), `${file} must expose the ${action} chart control`);
  }
  assert.match(page, /data-chart-scale/, `${file} must show the current chart scale`);
}

console.log("chart resize controls regression test passed");
