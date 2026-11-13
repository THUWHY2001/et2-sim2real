import assert from "node:assert/strict";
import fs from "node:fs";
import {createLegPinPlanarModel} from "../src/leg-pin-planar-model.mjs";
import {createLegPinPlanarFrictionCatalog,validateContactConfig} from "../src/leg-pin-planar-friction-catalog.mjs";

const legacy=JSON.parse(fs.readFileSync(new URL("../data/leg-full-package/model.json",import.meta.url),"utf8"));
const model=createLegPinPlanarModel(legacy),catalog=createLegPinPlanarFrictionCatalog(model);
assert.equal(catalog.groups.length,25);
assert.deepEqual(new Set(catalog.groups.map(item=>item.joint_id)),new Set(Object.keys(model.joints)));
assert.equal(catalog.groups.filter(item=>item.classification==="pin_bushing_candidate").length,13);
assert.equal(new Set(catalog.contacts.map(item=>item.physical_contact_id)).size,catalog.contacts.length);
for(const id of ["J_LEG_L_P010_S_L09_L03","J_LEG_L_P018_S_L17_L04"]){const item=catalog.byJointId.get(id)[0];assert.equal(item.verification_status,"real_joint_type_unverified");assert.equal(item.enabled,false);assert.equal(item.include_in_official_total,false)}
for(const id of ["J_LEG_L_HIP_pitch_T","J_LEG_L_KNEE_pitch_T","J_LEG_L_ANKLE_pitch_T"]){const item=catalog.byJointId.get(id)[0];assert.equal(item.contact_model,"prismatic_guide_coulomb");assert.equal(item.load_status,"guide_contact_distribution_unresolved");assert.equal(item.include_in_official_total,false)}
for(const id of ["J_LEG_L_P013_F_L8_L12","J_LEG_L_P016_F_L16_B15","J_LEG_L_P021_F_L06_B20"])assert.equal(catalog.byJointId.get(id)[0].contact_model,"none_fixed");
const p13=catalog.groups.filter(item=>item.joint_id.includes("P013"));assert.equal(p13.length,3);assert.equal(new Set(p13.flatMap(item=>item.contacts.map(contact=>contact.physical_contact_id))).size,3);
const p16=catalog.groups.filter(item=>item.joint_id.includes("P016"));assert.equal(p16.length,3);assert.equal(new Set(p16.flatMap(item=>item.contacts.map(contact=>contact.physical_contact_id))).size,3);
const split=structuredClone(catalog.byJointId.get("J_LEG_L_P011_RE_L10_L03")[0]);split.contacts=[{...split.contacts[0],physical_contact_id:"leg.hip_rod.p11.side_a",load_fraction:.4},{...split.contacts[0],physical_contact_id:"leg.hip_rod.p11.side_b",load_fraction:.6}];assert.equal(validateContactConfig(split).valid,true);
split.contacts[1].load_fraction=.7;assert.equal(validateContactConfig(split).valid,false);
console.log("leg pin planar friction contact catalog regression test passed");
