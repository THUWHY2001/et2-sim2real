import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import {createLegFullRuntime} from "../src/leg-full-runtime.mjs";

const model=JSON.parse(readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8"));
const leg=createLegFullRuntime(model);
const solution=leg.solveOutput("J2",0);
const loads=Object.fromEntries(leg.rateKeys.map(key=>[key,{static:{positive:key.endsWith("_T")?10:1,negative:key.endsWith("_T")?10:1},dynamic:{positive:key.endsWith("_T")?7:.7,negative:key.endsWith("_T")?7:.7}}]));

const rates=leg.rates("J2",0);
assert.equal(rates.complete,true);
assert.equal(Object.keys(rates.values).length,leg.rateKeys.length);
assert.ok(Object.values(rates.values).every(Number.isFinite));

const base=leg.equivalentResistance("J2",solution,loads,1,"static");
assert.equal(base.complete,true);
assert.ok(Number.isFinite(base.total)&&base.total>0);

const nominalLoads=leg.nominalLoads(solution,{});
assert.equal(nominalLoads.complete,true);
assert.ok(Object.values(nominalLoads.loadsN).every(value=>Number.isFinite(value)&&value>=0));
assert.ok(nominalLoads.knownWeightN>120&&nominalLoads.knownWeightN<130);

const zeroLoaded=leg.loadedResistance("J2",solution,loads,1,"static",{coefficients:{},preloads:{}});
assert.equal(zeroLoaded.complete,true);
assert.ok(Math.abs(zeroLoaded.loadedTotal-base.total)<1e-12);
for(const key of leg.rateKeys)assert.ok(Math.abs(zeroLoaded.items[key].deltaNm)<1e-12);

const changedKey=leg.rateKeys.find(key=>Math.abs(base.rates[key])>1e-8);
const changed=leg.loadedResistance("J2",solution,loads,1,"static",{coefficients:{[changedKey]:.001},preloads:{}});
assert.ok(changed.items[changedKey].deltaNm>0);
for(const key of leg.rateKeys)if(key!==changedKey)assert.ok(Math.abs(changed.items[key].deltaNm)<1e-12,`${key} was contaminated by ${changedKey}`);

const gravity=leg.gravityHoldingTorque("J2",0),inertia=leg.effectiveInertia("J2",0),trajectory=leg.trajectoryDynamics("J2",5,{direction:1});
assert.equal(gravity.complete,true);assert.ok(Number.isFinite(gravity.torqueNm));
assert.equal(inertia.complete,true);assert.ok(inertia.kgM2>0);
assert.equal(trajectory.complete,true);assert.ok(Number.isFinite(trajectory.totalTorqueNm));

const cycle=leg.loadedCycle("J2",loads,{coefficients:{[changedKey]:.001},preloads:{}},{samples:21});
assert.equal(cycle.complete,true);
assert.equal(cycle.samples.length,21);
assert.equal(cycle.rankings.static.length,leg.rateKeys.length);

console.log("leg full dynamics regression test passed");
