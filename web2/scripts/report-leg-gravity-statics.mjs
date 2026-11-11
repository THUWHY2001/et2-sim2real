import fs from "node:fs";
import {fileURLToPath} from "node:url";
import {createLegFullRuntime} from "../src/leg-full-runtime.mjs";
import {createLegGravityStatics} from "../src/leg-gravity-statics.mjs";

const dataUrl=new URL("../data/leg-full-package/",import.meta.url);
export const reportPaths={
  json:fileURLToPath(new URL("nominal_gravity_statics_report.json",dataUrl)),
  csv:fileURLToPath(new URL("nominal_gravity_statics_report.csv",dataUrl)),
};
const model=JSON.parse(fs.readFileSync(new URL("model.json",dataUrl),"utf8"));
const boundary={
  worldInterface:"J_LEG_L_HIP_roll_RE",footContact:"none",
  heldRevolutes:["J_LEG_L_HIP_roll_RE","J_LEG_L_HIP_yaw_RE","J_LEG_L_ANKLE_roll_RE","J_LEG_L_TOE_pitch_RE"],
  toeConstraint:"position-held-unverified",loadCase:"gravity-quasi-static",
};
const compact=quantity=>({status:quantity.status,value:quantity.value,null_projection:quantity.nullProjection??null});

export function buildNominalGravityStaticsReport(){
  const runtime=createLegFullRuntime(model),statics=createLegGravityStatics(model,runtime),result=statics.solveNominal(boundary),virtualWork=["J2","J3","J4"].map(joint=>statics.verifyGravityVirtualWork(result,joint));
  if(virtualWork.some(item=>!item.passed))throw new Error("nominal gravity statics report blocked: virtual-work acceptance failed");
  const joints=result.jointReactions.map(joint=>{
    const components=[];
    for(const [name,quantity] of Object.entries(joint.passive_constraint_wrench.components))components.push({name:`passive_${name}`,unit:name.includes("Nm")?"N.m":"N",...compact(quantity)});
    for(const [name,quantity] of Object.entries(joint.total_joint_wrench.components))components.push({name:`total_${name}`,unit:name.includes("Nm")?"N.m":"N",...compact(quantity)});
    for(const [name,quantity] of Object.entries(joint.force_decomposition))components.push({name:`total_${name}`,unit:"N",...compact(quantity)});
    if(joint.actuator_axial_force)components.push({name:"actuator_axial_force",unit:"N",...compact(joint.actuator_axial_force)});
    return{
      joint_id:joint.joint_id,joint_type:joint.joint_type,body_i:joint.body_i,body_j:joint.body_j,motion_state:joint.motion_state,
      contact_load_mapping:joint.contact_load_mapping,geometry:joint.geometry,
      uniqueness:{unique_components:components.filter(item=>item.status==="unique"),indeterminate_components:components.filter(item=>item.status!=="unique")},
    };
  });
  return{
    schema:"et2-leg-nominal-gravity-statics-v1",
    generated_at:new Date().toISOString(),
    scope:{stage:"nominal-pose-only",pose_degrees:result.pose.output_degrees,all_output_degrees:result.pose.all_output_degrees,boundary:{world_interface:boundary.worldInterface,foot_contact:boundary.footContact,held_revolutes:boundary.heldRevolutes,toe_constraint:boundary.toeConstraint,test_rig_correspondence:"unverified"},load_definition:"重力准静态理论载荷",excluded:["inertia","assembly_preload","friction_feedback","foot_contact","full_1_hz_cycle"]},
    matrix:{rows:result.assembly.A.length,columns:result.assembly.A[0].length,rank:result.matrix.rank,augmented_rank:result.matrix.augmented_rank,nullity:result.matrix.nullity,condition_number:result.matrix.condition_number,consistent:result.matrix.consistent,svd_converged:result.matrix.svd.converged},
    residuals:{matrix_norm:result.matrix.residual_norm,max_body_norm:result.residuals.max_body_norm,by_body:result.residuals.byBody,internal_action_reaction:result.internalWrenchBalance,global_balance:result.globalBalance,relative_equilibrium_tolerance:result.acceptance.relative_equilibrium_tolerance},
    actuator_forces:result.actuatorForces.map(item=>({joint_id:item.joint_id,...compact(item)})),
    holding_torques:result.holdingTorques.map(item=>({joint_id:item.joint_id,...compact(item)})),
    virtual_work:virtualWork.map(item=>({output_joint:item.output_joint,passed:item.passed,total_generalized_actuator_torque:compact(item.total_generalized_actuator_torque),gravity_potential_derivative:item.gravity_potential_derivative,absolute_residual_Nm:item.absolute_residual_Nm,relative_residual:item.relative_residual,actuators:item.actuators.map(actuator=>({joint_id:actuator.joint_id,rate_m_per_rad:actuator.rate_m_per_rad,force:compact(actuator.force),generalized_torque:compact(actuator.generalized_torque)}))})),
    joints,contact_load_qualification:"not_mapped",
    interpretation:"运动副合约束反力尚未映射为销轴、球铰、轴承或移动副的摩擦接触法向载荷。",
  };
}

const csvCell=value=>{const text=value===null||value===undefined?"":String(value);return /[",\n]/.test(text)?`"${text.replaceAll('"','""')}"`:text};
export function serializeJointCsv(report){
  const rows=[["joint_id","joint_type","body_i","body_j","quantity","unit","status","value","load_category","contact_load_mapping"]];
  for(const joint of report.joints)for(const group of [joint.uniqueness.unique_components,joint.uniqueness.indeterminate_components])for(const item of group){
    const category=item.name.startsWith("passive_")?"passive_constraint_wrench":item.name==="actuator_axial_force"?"actuator_axial_force":"total_joint_wrench";
    rows.push([joint.joint_id,joint.joint_type,joint.body_i,joint.body_j,item.name,item.unit,item.status,item.value,category,joint.contact_load_mapping]);
  }
  return rows.map(row=>row.map(csvCell).join(",")).join("\n")+"\n";
}

export function writeNominalGravityStaticsReport(){
  const report=buildNominalGravityStaticsReport(),csv=serializeJointCsv(report);
  fs.writeFileSync(reportPaths.json,JSON.stringify(report,null,2)+"\n");fs.writeFileSync(reportPaths.csv,csv);
  const indeterminate=report.joints.reduce((sum,joint)=>sum+joint.uniqueness.indeterminate_components.length,0),unique=report.joints.reduce((sum,joint)=>sum+joint.uniqueness.unique_components.length,0);
  console.log(`ET2 左腿基准重力静力验收：rank ${report.matrix.rank}/${report.matrix.columns}, nullity ${report.matrix.nullity}, residual ${report.residuals.matrix_norm}`);
  console.log(`25 个运动副分量：unique ${unique}, indeterminate ${indeterminate}`);
  for(const force of report.actuator_forces)console.log(`${force.joint_id}: ${force.status} ${force.value??"null"} N`);
  for(const item of report.virtual_work)console.log(`${item.output_joint}: ${item.passed?"PASS":"FAIL"}, residual ${item.absolute_residual_Nm} N.m`);
  return report;
}

if(process.argv[1]&&fileURLToPath(import.meta.url)===fileURLToPath(new URL(`file:///${process.argv[1].replaceAll("\\","/")}`)))writeNominalGravityStaticsReport();

