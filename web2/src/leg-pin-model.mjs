import {createCompleteLeftLeg} from "../data/leg-full-package/leg_full.mjs";

export const PIN_JOINT_MAPPINGS=Object.freeze([
  {rod:"hip",end:"upper",hardpoint:"P11",legacy_joint_id:"J_LEG_L_P011_S_L10_L03",pin_joint_id:"J_LEG_L_P011_RE_L10_L03"},
  {rod:"hip",end:"lower",hardpoint:"P13",legacy_joint_id:"J_LEG_L_P013_S_L12_L11",pin_joint_id:"J_LEG_L_P013_RE_L12_L11"},
  {rod:"knee",end:"upper",hardpoint:"P14",legacy_joint_id:"J_LEG_L_P014_S_L03_L13",pin_joint_id:"J_LEG_L_P014_RE_L03_L13"},
  {rod:"knee",end:"lower",hardpoint:"P16",legacy_joint_id:"J_LEG_L_P016_S_B15_L14",pin_joint_id:"J_LEG_L_P016_RE_B15_L14"},
  {rod:"ankle",end:"upper",hardpoint:"P19",legacy_joint_id:"J_LEG_L_P019_S_L18_L04",pin_joint_id:"J_LEG_L_P019_RE_L18_L04"},
  {rod:"ankle",end:"lower",hardpoint:"P21",legacy_joint_id:"J_LEG_L_P021_S_B20_L19",pin_joint_id:"J_LEG_L_P021_RE_B20_L19"},
]);

const transpose=matrix=>matrix[0].map((_,column)=>matrix.map(row=>row[column]));
const matrixVector=(matrix,vector)=>matrix.map(row=>row.reduce((sum,value,index)=>sum+value*vector[index],0));

export function createPinJointModel(legacyModel){
  const model=structuredClone(legacyModel),nominal=createCompleteLeftLeg(legacyModel).solve({}),worldY=[0,1,0],nominalWorldRotations=Object.fromEntries(Object.entries(nominal.world_transforms).map(([body,transform])=>[body,transform.R.map(row=>row.slice())]));
  model.meta={...model.meta,model_id:"pin-revolute-v1",parent_model_id:"legacy-ball",pin_axis_assumption:"robot_world_+Y_pending_CAD_verification",nominal_world_rotations:nominalWorldRotations};
  for(const mapping of PIN_JOINT_MAPPINGS){
    const legacy=model.joints[mapping.legacy_joint_id];
    if(!legacy||legacy.type!=="spherical")throw new Error(`legacy spherical joint missing: ${mapping.legacy_joint_id}`);
    const iAxisLocal=matrixVector(transpose(nominalWorldRotations[legacy.i_part]),worldY),jAxisLocal=matrixVector(transpose(nominalWorldRotations[legacy.j_part]),worldY);
    const pin={...legacy,type:"revolute",i_marker:`${legacy.i_part}.${mapping.pin_joint_id}.I`,j_marker:`${legacy.j_part}.${mapping.pin_joint_id}.J`,i_marker_axis_global:iAxisLocal.slice(),j_marker_axis_global:jAxisLocal.slice(),i_marker_axis_local:iAxisLocal.slice(),j_marker_axis_local:jAxisLocal.slice(),assumed_axis_world_nominal:worldY.slice(),axis_source:"geometry_assumption_pending_cad",legacy_joint_id:mapping.legacy_joint_id,hardpoint:mapping.hardpoint,rod:mapping.rod,end:mapping.end};
    delete model.joints[mapping.legacy_joint_id];model.joints[mapping.pin_joint_id]=pin;
  }
  return model;
}
