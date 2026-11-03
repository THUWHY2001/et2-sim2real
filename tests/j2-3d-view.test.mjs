import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const source = readFileSync(new URL("../src/j2-3d-view.js", import.meta.url), "utf8");
const window = {};
vm.runInNewContext(source, {window, Math, devicePixelRatio: 1});

assert.equal(typeof window.J2ThreeView?.create, "function");
assert.equal(typeof window.J2ThreeView?.projectPoint, "function");
assert.equal(window.J2ThreeView.clampZoom(0.1), 0.55);
assert.equal(window.J2ThreeView.clampZoom(9), 2.4);

const center = {x: 100, y: 20, z: -200};
const projected = window.J2ThreeView.projectPoint(center, {
  yaw: 0,
  pitch: 0,
  zoom: 1,
  width: 800,
  height: 520,
  center,
});
assert.equal(projected.x, 400);
assert.equal(projected.y, 260);
assert.ok(Number.isFinite(projected.depth));

const listeners = new Map();
const context = new Proxy({}, {get: (target, key) => target[key] || (() => {})});
const canvas = {
  width: 0,
  height: 0,
  style: {},
  clientWidth: 800,
  clientHeight: 520,
  getContext: () => context,
  getBoundingClientRect: () => ({width: 800, height: 520}),
  addEventListener: (name, handler) => listeners.set(name, handler),
  removeEventListener: (name) => listeners.delete(name),
  setPointerCapture: () => {},
};
const view = window.J2ThreeView.create(canvas);
for (const method of ["render", "sideView", "resetView", "destroy"]) {
  assert.equal(typeof view[method], "function");
}
assert.ok(listeners.has("pointerdown"));
assert.ok(listeners.has("wheel"));
view.destroy();
assert.equal(listeners.size, 0);

console.log("J2 3D view API test passed");
