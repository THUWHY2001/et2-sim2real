import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const source=readFileSync(new URL('../src/torso-dynamics-data.js',import.meta.url),'utf8');
const window={};
vm.runInNewContext(source,{window});
const D=window.TorsoDynamicsData;

assert.deepEqual([...D.gravity],[0,0,-9.80665]);
assert.equal(D.knownMassKg,11.3106595);
assert.equal(D.explicitBodies.length,9);
assert.equal(D.mergedAttachments.length,8);
assert.equal(D.explicitBodies.some(body=>body.partName==='base_link'),false,'base_link must stay outside the waist module');

assert.deepEqual(Array.from(D.explicitBodies,body=>body.partName),[
  'waist_link1','waist_link2','waist_link3','waist_link4','waist_ball_link5',
  'waist_link6','waist_ball_link7','waist_link8','waist_link9',
]);
assert.deepEqual(Array.from(D.mergedAttachments,body=>body.partName),[
  'waist_link3_1_L_link','waist_link3_1_R_link','waist_link3_2',
  'waist_link3_3_shoulder_L','waist_link3_3_shoulder_R',
  'waist_link3_4_waist_L','waist_link3_4_waist_link','waist_link3_4_waist_R',
]);
for(const attachment of D.mergedAttachments){
  assert.equal(attachment.attachedTo,'waist_link3');
  assert.equal(attachment.massKg,null);
}

assert.deepEqual({...D.partMapping},{
  waist_link1:'yaw',waist_link2:'roll',waist_link3:'platform',
  waist_link4:'crankL',waist_ball_link5:'crankL',
  waist_link6:'crankR',waist_ball_link7:'crankR',
  waist_link8:'rodL',waist_link9:'rodR',
});
const link3=D.explicitBodies.find(body=>body.partName==='waist_link3');
assert.equal(link3.massKg,10.2);
assert.deepEqual([...link3.comM],[-.0185177,-.0004286,.2350651]);
assert.equal(link3.inertia.ixx,.0929389);
assert.equal(link3.inertia.izx,.0050274);
assert.equal(link3.status,'临时简化，待核实');

const raw=JSON.parse(readFileSync(new URL('../data/ET2_waist_dynamics_addon.json',import.meta.url),'utf8'));
assert.equal(raw.module_scope.known_explicit_waist_mass_sum_kg,D.knownMassKg);
assert.deepEqual(raw.gravity.vector_m_s2,[...D.gravity]);

console.log('torso dynamics data regression test passed');
