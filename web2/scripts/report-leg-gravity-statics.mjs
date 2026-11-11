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
  const zero=createLegGravityStatics(model,runtime,{gravityScale:0}).solveNominal(boundary),doubled=createLegGravityStatics(model,runtime,{gravityScale:2}).solveNominal(boundary),zeroValues=[...zero.actuatorForces,...zero.holdingTorques].filter(item=>item.status==="unique").map(item=>Math.abs(item.value)),doubleErrors=result.actuatorForces.map((item,index)=>item.status==="unique"?Math.abs(doubled.actuatorForces[index].value-2*item.value):0),poseChecks=[];
  for(const output of ["J2","J3","J4"])for(const delta of [-.1,.1]){const perturbed=statics.solve(runtime.solveOutput(output,delta),boundary);poseChecks.push({output_joint:output,delta_degrees:delta,rank:perturbed.matrix.rank,nullity:perturbed.matrix.nullity,residual_norm:perturbed.matrix.residual_norm,residual_ratio:perturbed.matrix.residual_norm/result.matrix.residual_norm})}
  const yawAudit=statics.auditYawGravity(result),perturbationTests={
    zero_gravity:{passed:zero.matrix.residual_norm<1e-12&&Math.max(...zeroValues)<1e-10,matrix_residual_norm:zero.matrix.residual_norm,max_unique_reaction_abs:Math.max(...zeroValues)},
    double_gravity:{passed:Math.max(...doubleErrors)<1e-7&&Math.abs(doubled.matrix.residual_norm-2*result.matrix.residual_norm)<1e-10,max_actuator_linearity_error_N:Math.max(...doubleErrors),residual_ratio:doubled.matrix.residual_norm/result.matrix.residual_norm},
    pose_rank_stability:{passed:poseChecks.every(item=>item.rank===result.matrix.rank&&item.nullity===result.matrix.nullity&&Math.abs(item.residual_ratio-1)<.02),samples:poseChecks},
  };
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
    matrix:{rows:result.assembly.A.length,columns:result.assembly.A[0].length,rank:result.matrix.rank,augmented_rank:result.matrix.augmented_rank,nullity:result.matrix.nullity,left_nullity:result.unbalancedModes.length,condition_number:result.matrix.condition_number,consistent:result.matrix.consistent,numerical_consistency_tolerance:result.acceptance.relative_equilibrium_tolerance,exact_equilibrium_passed:result.acceptance.physical_static_equilibrium_passed,solution_class:"tolerance-qualified least-squares; physical acceptance blocked",svd_converged:result.matrix.svd.converged},
    residuals:{matrix_norm:result.matrix.residual_norm,max_body_norm:result.residuals.max_body_norm,max_force_norm_N:result.residuals.max_force_norm_N,max_moment_norm_Nm:result.residuals.max_moment_norm_Nm,normalized:result.residuals.normalized,by_body:result.residuals.byBody,by_body_physical:result.residuals.byBodyPhysical,internal_action_reaction:result.internalWrenchBalance,global_balance:result.globalBalance,relative_equilibrium_tolerance:result.acceptance.relative_equilibrium_tolerance,note:"matrix_norm and max_body_norm combine force and moment components and are retained only for numerical traceability; physical acceptance uses separated force and moment residuals."},
    actuator_forces:result.actuatorForces.map(item=>({joint_id:item.joint_id,...compact(item)})),
    holding_torques:result.holdingTorques.map(item=>({joint_id:item.joint_id,...compact(item)})),
    virtual_work:virtualWork.map(item=>({output_joint:item.output_joint,passed:item.passed,total_generalized_actuator_torque:compact(item.total_generalized_actuator_torque),gravity_potential_derivative:item.gravity_potential_derivative,absolute_residual_Nm:item.absolute_residual_Nm,relative_residual:item.relative_residual,acceptance:item.acceptance,actuators:item.actuators.map(actuator=>({joint_id:actuator.joint_id,rate_m_per_rad:actuator.rate_m_per_rad,force:compact(actuator.force),generalized_torque:compact(actuator.generalized_torque)}))})),
    unbalanced_modes:result.unbalancedModes,yaw_gravity_audit:yawAudit,perturbation_tests:perturbationTests,
    physical_acceptance:{passed:false,suitable_for_friction_torque_estimation:false,status:"blocked",blockers:["Three unrestrained rod-body axial-spin modes have nonzero gravity generalized residuals.","J1 vertical-axis yaw changes potential energy through the unresolved leg_l_link18 orientation gauge.","Static equilibrium has nonzero moment residuals concentrated in rod bodies leg_l_link18 and leg_l_link19."],required_resolution:"Confirm the physical anti-rotation/orientation constraint of each pushrod assembly or provide the intended rod-body rotational gauge before using reaction loads for friction torque estimation."},
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
  rows.push(["__AUDIT__","audit","","","max_force_residual","N","observed",report.residuals.max_force_norm_N,"physical_residual","not_applicable"]);
  rows.push(["__AUDIT__","audit","","","max_moment_residual","N.m","observed",report.residuals.max_moment_norm_Nm,"physical_residual","not_applicable"]);
  rows.push(["__AUDIT__","audit","","","J1_gravity_potential_derivative","N.m",report.yaw_gravity_audit.passed?"passed":"failed",report.yaw_gravity_audit.potential_derivative_Nm,"yaw_gravity_audit","not_applicable"]);
  rows.push(["__AUDIT__","audit","","","physical_acceptance","",report.physical_acceptance.status,"","acceptance_gate","not_applicable"]);
  return rows.map(row=>row.map(csvCell).join(",")).join("\n")+"\n";
}

export function writeNominalGravityStaticsReport(){
  const report=buildNominalGravityStaticsReport(),csv=serializeJointCsv(report);
  fs.writeFileSync(reportPaths.json,JSON.stringify(report,null,2)+"\n");fs.writeFileSync(reportPaths.csv,csv);
  const indeterminate=report.joints.reduce((sum,joint)=>sum+joint.uniqueness.indeterminate_components.length,0),unique=report.joints.reduce((sum,joint)=>sum+joint.uniqueness.unique_components.length,0);
  console.log(`ET2 左腿基准重力静力复核：${report.physical_acceptance.passed?"PASS":"BLOCKED"}; rank ${report.matrix.rank}/${report.matrix.columns}, nullity ${report.matrix.nullity}, residual ${report.residuals.matrix_norm}`);
  console.log(`分单位残差：force ${report.residuals.max_force_norm_N} N, moment ${report.residuals.max_moment_norm_Nm} N.m; J1 yaw audit ${report.yaw_gravity_audit.passed?"PASS":"FAIL"}`);
  console.log(`25 个运动副分量：unique ${unique}, indeterminate ${indeterminate}`);
  for(const force of report.actuator_forces)console.log(`${force.joint_id}: ${force.status} ${force.value??"null"} N`);
  for(const item of report.virtual_work)console.log(`${item.output_joint}: ${item.passed?"PASS":"FAIL"}, residual ${item.absolute_residual_Nm} N.m`);
  return report;
}

if(process.argv[1]&&fileURLToPath(import.meta.url)===fileURLToPath(new URL(`file:///${process.argv[1].replaceAll("\\","/")}`)))writeNominalGravityStaticsReport();
