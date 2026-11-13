const PIN_IDS=new Set([
  "J_LEG_L_HIP_pitch_RE","J_LEG_L_KNEE_pitch_RE","J_LEG_L_ANKLE_pitch_RE",
  "J_LEG_L_P009_RE_L02_L08","J_LEG_L_P013_RE_L08_L09","J_LEG_L_P016_RE_L16_L17","J_LEG_L_P017_RE_L03_L16",
  "J_LEG_L_P011_RE_L10_L03","J_LEG_L_P013_RE_L12_L11","J_LEG_L_P014_RE_L03_L13","J_LEG_L_P016_RE_B15_L14","J_LEG_L_P019_RE_L18_L04","J_LEG_L_P021_RE_B20_L19"
]);
const UNVERIFIED=new Set(["J_LEG_L_P010_S_L09_L03","J_LEG_L_P018_S_L17_L04"]);
const NEW_PIN_AXIS_ASSUMPTION=new Set(["J_LEG_L_P011_RE_L10_L03","J_LEG_L_P013_RE_L12_L11","J_LEG_L_P014_RE_L03_L13","J_LEG_L_P016_RE_B15_L14","J_LEG_L_P019_RE_L18_L04","J_LEG_L_P021_RE_B20_L19"]);
const HELD=new Set(["J_LEG_L_HIP_roll_RE","J_LEG_L_HIP_yaw_RE","J_LEG_L_ANKLE_roll_RE","J_LEG_L_TOE_pitch_RE"]);
const PHYSICAL_IDS={
  J_LEG_L_HIP_roll_RE:"leg.hip_roll.axis",J_LEG_L_HIP_yaw_RE:"leg.hip_yaw.axis",J_LEG_L_HIP_pitch_RE:"leg.j2.output_pin",J_LEG_L_KNEE_pitch_RE:"leg.j3.output_pin",J_LEG_L_ANKLE_pitch_RE:"leg.j4.output_pin",J_LEG_L_TOE_pitch_RE:"leg.toe_pitch.axis",
  J_LEG_L_P009_RE_L02_L08:"leg.hip_rocker.p09",J_LEG_L_P010_S_L09_L03:"leg.p10.unverified",J_LEG_L_HIP_pitch_T:"leg.hip_rod.guide",J_LEG_L_P013_F_L8_L12:"leg.hip_rod.p13.carrier_fixed",J_LEG_L_P013_RE_L08_L09:"leg.hip_rocker.p13",J_LEG_L_KNEE_pitch_T:"leg.knee_rod.guide",J_LEG_L_P016_F_L16_B15:"leg.knee_rod.p16.carrier_fixed",J_LEG_L_P016_RE_L16_L17:"leg.knee_rocker.p16",J_LEG_L_P017_RE_L03_L16:"leg.knee_rocker.p17",J_LEG_L_P018_S_L17_L04:"leg.p18.unverified",J_LEG_L_ANKLE_pitch_T:"leg.ankle_rod.guide",J_LEG_L_P021_F_L06_B20:"leg.ankle_rod.p21.carrier_fixed",J_LEG_L_ANKLE_roll_RE:"leg.j5.roll_axis",
  J_LEG_L_P011_RE_L10_L03:"leg.hip_rod.p11.pin_bushing",J_LEG_L_P013_RE_L12_L11:"leg.hip_rod.p13.pin_bushing",J_LEG_L_P014_RE_L03_L13:"leg.knee_rod.p14.pin_bushing",J_LEG_L_P016_RE_B15_L14:"leg.knee_rod.p16.pin_bushing",J_LEG_L_P019_RE_L18_L04:"leg.ankle_rod.p19.pin_bushing",J_LEG_L_P021_RE_B20_L19:"leg.ankle_rod.p21.pin_bushing"
};
const DISPLAY={
  J_LEG_L_HIP_pitch_RE:"J2 髋 Pitch 输出销轴",J_LEG_L_KNEE_pitch_RE:"J3 膝 Pitch 输出销轴",J_LEG_L_ANKLE_pitch_RE:"J4 踝 Pitch 输出销轴",
  J_LEG_L_P010_S_L09_L03:"P10 连接副（真实类型待核实）",J_LEG_L_P018_S_L17_L04:"P18 连接副（真实类型待核实）",
  J_LEG_L_P011_RE_L10_L03:"髋推杆 P11 销轴—轴套",J_LEG_L_P013_RE_L12_L11:"髋推杆 P13 销轴—轴套",J_LEG_L_P014_RE_L03_L13:"膝推杆 P14 销轴—轴套",J_LEG_L_P016_RE_B15_L14:"膝推杆 P16 销轴—轴套",J_LEG_L_P019_RE_L18_L04:"踝推杆 P19 销轴—轴套",J_LEG_L_P021_RE_B20_L19:"踝推杆 P21 销轴—轴套"
};

const parameterTemplate=()=>({mu_static:null,mu_kinetic:null,effective_radius_m:null,provenance:"user_input"});
const surface=(physical_contact_id)=>({physical_contact_id,load_fraction:1,parameters:parameterTemplate()});

function classify(jointId,joint){
  if(PIN_IDS.has(jointId))return{classification:"pin_bushing_candidate",contact_model:"pin_bushing_coulomb",verification_status:NEW_PIN_AXIS_ASSUMPTION.has(jointId)?"geometry_axis_assumption_pending_cad":joint.axis_source||"source_joint_verified",enabled:true,include_in_official_total:true,load_status:"planar_reaction_available"};
  if(UNVERIFIED.has(jointId))return{classification:"real_joint_type_unverified",contact_model:"unverified",verification_status:"real_joint_type_unverified",enabled:false,include_in_official_total:false,load_status:"planar_reaction_available"};
  if(joint.planar_type==="prismatic")return{classification:"prismatic_interface",contact_model:"prismatic_guide_coulomb",verification_status:"guide_geometry_required",enabled:false,include_in_official_total:false,load_status:"guide_contact_distribution_unresolved",required_geometry:["support_spacing_m","guide_direction","preload_N"]};
  if(joint.source_type==="fixed")return{classification:"fixed_connection",contact_model:"none_fixed",verification_status:"not_applicable",enabled:false,include_in_official_total:false,load_status:"not_applicable"};
  if(HELD.has(jointId))return{classification:"held_revolute",contact_model:"pin_bushing_coulomb",verification_status:"zero_relative_motion_in_current_boundary",enabled:false,include_in_official_total:false,load_status:"planar_reaction_available"};
  throw new Error(`unclassified planar joint ${jointId}`);
}

export function validateContactConfig(group){
  const errors=[],contacts=Array.isArray(group?.contacts)?group.contacts:[];
  if(!group?.joint_id)errors.push("joint_id_required");
  if(!contacts.length)errors.push("physical_contact_required");
  if(new Set(contacts.map(item=>item.physical_contact_id)).size!==contacts.length)errors.push("duplicate_physical_contact_id");
  const sum=contacts.reduce((value,item)=>value+Number(item.load_fraction),0);if(Math.abs(sum-1)>1e-9)errors.push("load_fraction_sum_must_equal_one");
  for(const item of contacts){if(!item.physical_contact_id)errors.push("physical_contact_id_required");if(!(Number(item.load_fraction)>=0))errors.push("invalid_load_fraction");const p=item.parameters||{};for(const key of ["mu_static","mu_kinetic","effective_radius_m"])if(p[key]!=null&&!(Number(p[key])>=0))errors.push(`invalid_${key}`)}
  return{valid:errors.length===0,errors};
}

export function createLegPinPlanarFrictionCatalog(model){
  if(model?.meta?.model_id!=="pin-planar-ideal-v1")throw new Error("pin-planar-ideal-v1 required");
  const groups=Object.entries(model.joints).map(([jointId,joint])=>{const meta=classify(jointId,joint),physicalId=PHYSICAL_IDS[jointId];if(!physicalId)throw new Error(`physical contact id missing: ${jointId}`);return Object.freeze({joint_id:jointId,display_name:DISPLAY[jointId]||jointId,body_i:joint.i_part,body_j:joint.j_part,source_joint_type:joint.source_type,planar_joint_type:joint.planar_type,...meta,contacts:[surface(`${physicalId}.surface_unspecified`)],load:{mode:"automatic",source:"planar_gravity_estimate",manual_override_N:null}})});
  const contacts=groups.flatMap(group=>group.contacts.map(contact=>({...contact,joint_id:group.joint_id,contact_model:group.contact_model,verification_status:group.verification_status,enabled:group.enabled,include_in_official_total:group.include_in_official_total}))),byJointId=new Map(groups.map(group=>[group.joint_id,[group]])),byContactId=new Map(contacts.map(contact=>[contact.physical_contact_id,contact]));
  const audit={joint_count:groups.length,physical_contact_count:contacts.length,pin_candidate_count:groups.filter(item=>item.classification==="pin_bushing_candidate").length,unverified_joint_ids:groups.filter(item=>item.classification==="real_joint_type_unverified").map(item=>item.joint_id),official_contact_ids:contacts.filter(item=>item.include_in_official_total).map(item=>item.physical_contact_id)};
  return Object.freeze({groups,contacts,byJointId,byContactId,audit});
}
