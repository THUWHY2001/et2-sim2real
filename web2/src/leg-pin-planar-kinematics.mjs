import {createLegPinRuntime} from "./leg-pin-runtime.mjs";
import {__test as svdTest,solveRankRevealing} from "./rank-revealing-svd.mjs";

const OUTPUTS=["J2","J3","J4"],RODS=["hip","knee","ankle"];
const dot=(a,b)=>a.reduce((sum,value,index)=>sum+value*b[index],0);
const norm=value=>Math.hypot(...value);
const unit=value=>value.map(item=>item/norm(value));
const sub=(a,b)=>a.map((value,index)=>value-b[index]);
const thetaFromR=R=>Math.atan2(R[0][2],R[0][0]);
const rotate=(theta,vector)=>[Math.cos(theta)*vector[0]+Math.sin(theta)*vector[1],-Math.sin(theta)*vector[0]+Math.cos(theta)*vector[1]];
const wrap=value=>Math.atan2(Math.sin(value),Math.cos(value));
const determinant3=M=>M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])-M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])+M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);

function addPointRow(row,bodies,body,point,com,direction,sign){if(body==="base_link")return;const index=bodies.indexOf(body),relative=sub(point,com);row[3*index]+=sign*direction[0];row[3*index+1]+=sign*direction[1];row[3*index+2]+=sign*(direction[0]*relative[1]-direction[1]*relative[0])}
function addAngular(row,bodies,body,value){if(body!=="base_link")row[3*bodies.indexOf(body)+2]+=value}

export function createLegPinPlanarKinematics(model){
  if(model?.meta?.model_id!=="pin-planar-ideal-v1"||!model.legacyModel)throw new Error("pin-planar-ideal-v1 model with source data required");
  const runtime=createLegPinRuntime(model.legacyModel),nominal={...runtime.nominal};
  function solveAngles(angles={}){
    const target={...nominal,J2:Number(angles.J2??nominal.J2),J3:Number(angles.J3??nominal.J3),J4:Number(angles.J4??nominal.J4)},direct=runtime.legacy.blackbox.directAngles(target),solution=runtime.solveRods({hip_mm:direct.active_rod_inputs.hip.delta_vs_nominal_mm,knee_mm:direct.active_rod_inputs.knee.delta_vs_nominal_mm,ankle_mm:direct.active_rod_inputs.ankle.delta_vs_nominal_mm,J0_deg:0,J1_deg:0,J5_deg:0}),poses={},com={};
    for(const body of model.bodies){const transform=solution.frame.world_transforms[body];poses[body]={theta_rad:thetaFromR(transform.R),t_xz_mm:[transform.t_mm[0],transform.t_mm[2]]};const source=solution.frame.mass.COM_world_mm[body];com[body]=[source[0],source[2]]}
    const markerPairs=new Map();for(const marker of solution.frame.all_joint_marker_sides){if(!markerPairs.has(marker.joint))markerPairs.set(marker.joint,{});markerPairs.get(marker.joint)[marker.marker_side]=[marker.xyz_mm[0],marker.xyz_mm[2]]}
    const rows=[],rowMetadata=[],residuals=[];
    for(const [jointId,joint] of Object.entries(model.joints)){
      const pair=markerPairs.get(jointId),iPoint=pair.I,jPoint=pair.J,gap=sub(jPoint,iPoint),iTheta=joint.i_part==="base_link"?0:poses[joint.i_part].theta_rad,jTheta=joint.j_part==="base_link"?0:poses[joint.j_part].theta_rad;
      if(joint.planar_type==="revolute"||joint.planar_type==="fixed")for(const [component,direction] of [["x",[1,0]],["z",[0,1]]]){const row=Array(3*model.bodies.length).fill(0);addPointRow(row,model.bodies,joint.i_part,iPoint,joint.i_part==="base_link"?[0,0]:com[joint.i_part],direction,-1);addPointRow(row,model.bodies,joint.j_part,jPoint,joint.j_part==="base_link"?[0,0]:com[joint.j_part],direction,1);rows.push(row);rowMetadata.push({joint_id:jointId,constraint:`point_${component}`});residuals.push({joint_id:jointId,constraint:`point_${component}`,value_mm:gap[component==="x"?0:1]})}
      if(joint.planar_type==="prismatic"){
        const axis=unit(rotate(iTheta,joint.ideal_axis_i_local_xz)),normal=[axis[1],-axis[0]],row=Array(3*model.bodies.length).fill(0);addPointRow(row,model.bodies,joint.i_part,iPoint,com[joint.i_part],normal,-1);addPointRow(row,model.bodies,joint.j_part,jPoint,com[joint.j_part],normal,1);addAngular(row,model.bodies,joint.i_part,-dot(axis,gap));rows.push(row);rowMetadata.push({joint_id:jointId,constraint:"prismatic_lateral"});residuals.push({joint_id:jointId,constraint:"prismatic_lateral",value_mm:dot(normal,gap)});
      }
      if(joint.planar_type==="fixed"||joint.planar_type==="prismatic"){
        const row=Array(3*model.bodies.length).fill(0);addAngular(row,model.bodies,joint.i_part,-1);addAngular(row,model.bodies,joint.j_part,1);rows.push(row);rowMetadata.push({joint_id:jointId,constraint:"relative_angle"});residuals.push({joint_id:jointId,constraint:"relative_angle",value_rad:wrap((jTheta-iTheta)-joint.nominal_relative_angle_rad)});
      }
    }
    const rank=solveRankRevealing(rows,Array(rows.length).fill(0),{relativeTolerance:1e-10}),positionResiduals=residuals.filter(item=>item.value_mm!==undefined),angleResiduals=residuals.filter(item=>item.value_rad!==undefined),rodLengths=Object.fromEntries(RODS.map(rod=>{const input=solution.frame.active_rod_inputs[rod];return[rod,norm(sub(input.anchor2_xyz_mm,input.anchor1_xyz_mm))/1000]}));
    return{complete:solution.complete&&rank.rank===57,model_id:model.meta.model_id,outputs:target,poses,com_world_xz_mm:com,rod_lengths_m:rodLengths,constraint_jacobian:{matrix:rows,row_metadata:rowMetadata,rows:rows.length,columns:rows[0].length,rank:rank.rank,mobility:rows[0].length-rank.rank,condition_number:rank.conditionNumber,singular_values:rank.singularValues},constraint_residuals:residuals,max_constraint_residual_mm:Math.max(0,...positionResiduals.map(item=>Math.abs(item.value_mm))),max_angular_residual_rad:Math.max(0,...angleResiduals.map(item=>Math.abs(item.value_rad))),source_solution:solution};
  }
  const solveOutput=(output,delta)=>{if(!OUTPUTS.includes(output))throw new Error(`unsupported planar output ${output}`);return solveAngles({...nominal,[output]:nominal[output]+Number(delta)})};
  function lengthJacobian(outputs={}){const h=.001,J=RODS.map(()=>Array(3).fill(0));for(let column=0;column<OUTPUTS.length;column++){const output=OUTPUTS[column],lo=solveAngles({...outputs,[output]:Number(outputs[output]??nominal[output])-h}),hi=solveAngles({...outputs,[output]:Number(outputs[output]??nominal[output])+h}),denominator=2*h*Math.PI/180;for(let row=0;row<RODS.length;row++)J[row][column]=(hi.rod_lengths_m[RODS[row]]-lo.rod_lengths_m[RODS[row]])/denominator}const solved=solveRankRevealing(J,[0,0,0],{relativeTolerance:1e-10}),physical=svdTest.jacobiSvd(J),singular=physical.singularValues,condition=singular[0]/singular[2];return{rows:RODS,columns:OUTPUTS,matrix:J,rank:solved.rank,determinant_m3_per_rad3:determinant3(J),condition_number:condition,singular_values:singular}}
  return Object.freeze({model,nominal,solveAngles,solveOutput,lengthJacobian,runtime});
}
