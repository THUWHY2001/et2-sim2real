import {createLegPinRuntime} from "./leg-pin-runtime.mjs";

const HELD=new Set(["J_LEG_L_HIP_roll_RE","J_LEG_L_HIP_yaw_RE","J_LEG_L_ANKLE_roll_RE","J_LEG_L_TOE_pitch_RE"]);
const ROD_BY_JOINT={J_LEG_L_HIP_pitch_T:"hip",J_LEG_L_KNEE_pitch_T:"knee",J_LEG_L_ANKLE_pitch_T:"ankle"};
const norm=value=>Math.hypot(...value);
const unit=value=>value.map(item=>item/norm(value));
const sub=(a,b)=>a.map((value,index)=>value-b[index]);
const thetaFromR=R=>Math.atan2(R[0][2],R[0][0]);
const rotateInverse=(theta,vector)=>[Math.cos(theta)*vector[0]-Math.sin(theta)*vector[1],Math.sin(theta)*vector[0]+Math.cos(theta)*vector[1]];
const angleDeg=(a,b)=>Math.acos(Math.max(-1,Math.min(1,Math.abs(a[0]*b[0]+a[1]*b[1]))))*180/Math.PI;

export function createLegPinPlanarModel(legacyModel){
  const runtime=createLegPinRuntime(legacyModel),pin=runtime.model,nominal=runtime.solveOutput("J2",0),referenceY=93.00025,bodies=Object.keys(pin.parts),poses=Object.fromEntries(bodies.map(body=>{const transform=nominal.frame.world_transforms[body];return[body,{t_xz_mm:[transform.t_mm[0],transform.t_mm[2]],theta_rad:thetaFromR(transform.R)}]})),joints={},prismaticAxes=[];
  for(const [jointId,source] of Object.entries(pin.joints)){
    const iTheta=source.i_part==="base_link"?0:poses[source.i_part].theta_rad,jTheta=source.j_part==="base_link"?0:poses[source.j_part].theta_rad,planarType=source.type==="translational"?"prismatic":source.type==="fixed"||HELD.has(jointId)?"fixed":"revolute",joint={joint_id:jointId,source_type:source.type,planar_type:planarType,i_part:source.i_part,j_part:source.j_part,initial_center_xz_mm:[source.initial_center_global_mm[0],source.initial_center_global_mm[2]],nominal_relative_angle_rad:jTheta-iTheta,held_reason:HELD.has(jointId)?"J5=0 planar boundary":null};
    if(planarType==="prismatic"){
      const rod=ROD_BY_JOINT[jointId],input=nominal.frame.active_rod_inputs[rod],idealWorld=unit([input.anchor2_xyz_mm[0]-input.anchor1_xyz_mm[0],input.anchor2_xyz_mm[2]-input.anchor1_xyz_mm[2]]),marker=nominal.frame.all_joint_marker_sides.find(item=>item.joint===jointId&&item.marker_side==="I"),sourceWorld=unit([marker.axis_xyz[0],marker.axis_xyz[2]]);joint.rod=rod;joint.ideal_axis_i_local_xz=rotateInverse(iTheta,idealWorld);joint.nominal_axis_world_xz=idealWorld;
      prismaticAxes.push({joint_id:jointId,rod,source_axis_world_xyz:marker.axis_xyz,source_axis_xz:sourceWorld,ideal_axis_xz:idealWorld,axis_change_deg:angleDeg(sourceWorld,idealWorld),source_y_component:marker.axis_xyz[1],source_nominal_lateral_gap_mm:nominal.frame.joint_constraint_checks.find(item=>item.joint===jointId).lateral_gap_mm});
    }
    joints[jointId]=joint;
  }
  const hardpointProjection=Object.entries(legacyModel.hardpoints).map(([hardpoint,item])=>({hardpoint,source_xyz_mm:item.xyz_mm,ideal_xyz_mm:[item.xyz_mm[0],referenceY,item.xyz_mm[2]],delta_y_mm:referenceY-item.xyz_mm[1]}));
  const parts=Object.fromEntries(Object.entries(pin.parts).map(([name,part])=>[name,{mass_kg:part.mass_kg,com_initial_xz_mm:[part.com_initial_global_mm[0],part.com_initial_global_mm[2]],com_source_y_mm:part.com_initial_global_mm[1]}]));
  const model={meta:{model_id:"pin-planar-ideal-v1",parent_model_id:"pin-revolute-v1",source_model_id:"legacy-ball",plane:"X-Z",reference_y_mm:referenceY,J5_deg:0,assumptions:["all body motion is planar","six endpoint pin axes are world Y","three prismatic axes follow exact endpoint lines","J0/J1/J5/toe are position-held"]},bodies,parts,joints,nominal_body_poses:poses,idealization:{hardpoint_y_projection:hardpointProjection,prismatic_axes:prismaticAxes}};
  Object.defineProperty(model,"legacyModel",{value:legacyModel,enumerable:false});
  return model;
}
