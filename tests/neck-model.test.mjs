import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

const source=readFileSync(new URL("../src/neck-model.js",import.meta.url),"utf8");
const window={};
vm.runInNewContext(source,{window,Math});
const M=window.NeckModel;

assert.equal(M.points.upper2.y,-32);
assert.deepEqual({...M.points.com},{x:-9.851,y:.204,z:408.752});

const j0=M.solve("J0",5),j0zero=M.solve("J0",0);
const j1=M.solve("J1",5),j1zero=M.solve("J1",0);
const j0d1=j0.lengths.rod1-j0zero.lengths.rod1;
const j0d2=j0.lengths.rod2-j0zero.lengths.rod2;
const j1d1=j1.lengths.rod1-j1zero.lengths.rod1;
const j1d2=j1.lengths.rod2-j1zero.lengths.rod2;
assert.ok(j0d1*j0d2<0,"J0 rods must change oppositely");
assert.ok(j1d1*j1d2>0,"J1 rods must change synchronously");
assert.ok(Math.abs(j1d1-j1d2)<1e-6,"symmetric J1 rods must have equal stroke");

assert.equal(j0.rates.R1,1);
assert.equal(j0.rates.P1,0);
assert.equal(j1.rates.P1,1);
assert.equal(j1.rates.R1,0);
assert.ok(Number.isFinite(j0.gravityTorque));
assert.ok(Number.isFinite(j1.gravityTorque));
for(const value of Object.values(j0.supportLoads))assert.ok(Number.isFinite(value)&&value>=0);

const rotary=()=>({static:{positive:1,negative:1.2},dynamic:{positive:.7,negative:.8},kN:0,preload:0});
const rod=()=>({static:{positive:10,negative:11},dynamic:{positive:7,negative:8}});
const loads={};
for(const key of M.keys.J0.rotary)loads[key]=rotary();
for(const key of M.keys.J0.rods)loads[key]=rod();
const result=M.equivalent("J0",j0zero,loads,1,"static");
assert.ok(result.frictionTotal>0);
assert.ok(Number.isFinite(result.driveTorque));
assert.ok(Math.abs(Object.values(result.contribution).reduce((a,b)=>a+b,0)-100)<1e-9);

const cycle=M.cycle("J0",loads);
assert.ok(cycle.peakAbsoluteTorque>0);
assert.ok(cycle.frictionEnergy>0);

console.log("neck model regression test passed");
