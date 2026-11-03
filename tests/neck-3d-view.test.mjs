import assert from "node:assert/strict";
import {existsSync,readFileSync} from "node:fs";
import vm from "node:vm";

const sourcePath=new URL("../src/neck-3d-view.js",import.meta.url);
assert.ok(existsSync(sourcePath),"neck 3D camera module must exist");
const window={};
vm.runInNewContext(readFileSync(sourcePath,"utf8"),{window,Math,Object});
const V=window.Neck3DView;
const camera=V.createCamera({width:600,height:500,center:{x:0,y:0,z:430},scale:2});

const initial=camera.project({x:40,y:25,z:480});
assert.ok(Number.isFinite(initial.x)&&Number.isFinite(initial.y)&&Number.isFinite(initial.depth));
camera.rotate(80,-35);
const rotated=camera.project({x:40,y:25,z:480});
assert.notDeepEqual({...rotated},{...initial},"drag rotation must change the screen projection");

camera.zoom(100);
assert.equal(camera.state().zoom,2.4,"zoom must clamp at the upper limit");
camera.zoom(-100);
assert.equal(camera.state().zoom,.55,"zoom must clamp at the lower limit");
camera.reset();
assert.deepEqual({...camera.state()},{yaw:-35,pitch:-20,zoom:1});
assert.deepEqual({...camera.project({x:40,y:25,z:480})},{...initial});

camera.rotate(0,100000);
const extreme=camera.project({x:-62,y:-27,z:371.06});
assert.ok(Number.isFinite(extreme.x)&&Number.isFinite(extreme.y)&&Number.isFinite(extreme.depth));

const build=readFileSync(new URL("../scripts/build-pages.mjs",import.meta.url),"utf8");
assert.match(build,/src", "neck-3d-view\.js"/);

console.log("neck 3D camera regression test passed");
