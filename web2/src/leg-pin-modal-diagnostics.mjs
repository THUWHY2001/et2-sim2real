import {__test as svdTest, solveRankRevealing} from "./rank-revealing-svd.mjs";

const OUTPUT_IDS={J2:"J_LEG_L_HIP_pitch_RE",J3:"J_LEG_L_KNEE_pitch_RE",J4:"J_LEG_L_ANKLE_pitch_RE"};
const ROD_IDS={hip:"J_LEG_L_HIP_pitch_T",knee:"J_LEG_L_KNEE_pitch_T",ankle:"J_LEG_L_ANKLE_pitch_T"};
const dot=(a,b)=>a.reduce((sum,value,index)=>sum+value*b[index],0);
const norm=value=>Math.hypot(...value);
const scale=(value,factor)=>value.map(item=>item*factor);
const add=(a,b)=>a.map((value,index)=>value+b[index]);
const sub=(a,b)=>a.map((value,index)=>value-b[index]);
const transpose=matrix=>matrix[0].map((_,column)=>matrix.map(row=>row[column]));
const multiply=(a,b)=>a.map(row=>b[0].map((_,column)=>row.reduce((sum,value,index)=>sum+value*b[index][column],0)));
const unit=value=>scale(value,1/norm(value));
const angleDeg=(a,b)=>Math.acos(Math.max(-1,Math.min(1,Math.abs(dot(unit(a),unit(b))))))*180/Math.PI;

function scaledSvd(A){
  const rowScales=A.map(row=>1/(norm(row)||1)),rowScaled=A.map((row,index)=>scale(row,rowScales[index])),columnScales=Array.from({length:A[0].length},(_,column)=>Math.hypot(...rowScaled.map(row=>row[column]))||1),scaled=rowScaled.map(row=>row.map((value,column)=>value/columnScales[column])),svd=svdTest.jacobiSvd(scaled);
  return{...svd,rowScales,columnScales,physicalRightVectors:svd.rightVectors.map(vector=>unit(vector.map((value,column)=>value/columnScales[column])))};
}

function outputRows(assembly){
  return Object.fromEntries(Object.entries(OUTPUT_IDS).map(([output,jointId])=>{
    const joint=assembly.joints.find(item=>item.joint_id===jointId),row=Array(assembly.equations.length).fill(0);
    for(const [body,sign] of [[joint.body_i,-1],[joint.body_j,1]]){const index=assembly.bodies.indexOf(body);if(index>=0)for(let axis=0;axis<3;axis++)row[6*index+3+axis]+=sign*joint.axis_world_xyz[axis]}
    return[output,row];
  }));
}

function bodyTwists(vector,bodies){return Object.fromEntries(bodies.map((body,index)=>[body,{linear_m_per_output_rad:vector.slice(6*index,6*index+3),angular_rad_per_output_rad:vector.slice(6*index+3,6*index+6)}]))}

function finiteDifferenceVector(runtime,output,bodies,hDeg=.001){
  const lo=runtime.solveOutput(output,-hDeg),center=runtime.solveOutput(output,0),hi=runtime.solveOutput(output,hDeg),denominator=2*hDeg*Math.PI/180,vector=[];
  for(const body of bodies){
    const lowCom=lo.frame.mass.COM_world_mm[body],highCom=hi.frame.mass.COM_world_mm[body];vector.push(...highCom.map((value,index)=>(value-lowCom[index])/1000/denominator));
    const R=center.frame.world_transforms[body].R,Rlo=lo.frame.world_transforms[body].R,Rhi=hi.frame.world_transforms[body].R,Rdot=Rhi.map((row,i)=>row.map((value,j)=>(value-Rlo[i][j])/denominator)),W=multiply(Rdot,transpose(R));vector.push((W[2][1]-W[1][2])/2,(W[0][2]-W[2][0])/2,(W[1][0]-W[0][1])/2);
  }
  return{vector,lo,center,hi};
}

function residualRanking(assembly,passiveColumns,velocity){
  const grouped=new Map();
  for(const column of passiveColumns){const unknown=assembly.unknowns[column],value=dot(assembly.A.map(row=>row[column]),velocity);if(!grouped.has(unknown.joint_id))grouped.set(unknown.joint_id,[]);grouped.get(unknown.joint_id).push({basis_label:unknown.basis_label,value,absolute_value:Math.abs(value)})}
  return [...grouped.entries()].map(([joint_id,components])=>({joint_id,norm:Math.hypot(...components.map(item=>item.value)),components:components.sort((a,b)=>b.absolute_value-a.absolute_value)})).sort((a,b)=>b.norm-a.norm);
}

function prismaticAudit(model,runtime,assembly){
  const nominal=runtime.solveOutput("J2",0),rodNames=Object.keys(ROD_IDS),items=[];
  for(const rod of rodNames){
    const jointId=ROD_IDS[rod],joint=assembly.joints.find(item=>item.joint_id===jointId),input=nominal.frame.active_rod_inputs[rod],endpointDirection=unit(sub(input.anchor2_xyz_mm,input.anchor1_xyz_mm)),modelJoint=model.joints[jointId],marker=nominal.frame.all_joint_marker_sides.find(item=>item.joint===jointId&&item.marker_side==="I").xyz_mm,fromUpper=sub(marker,input.anchor1_xyz_mm),markerOffset=norm(sub(fromUpper,scale(endpointDirection,dot(fromUpper,endpointDirection))));let maxLateral=0;
    for(const output of ["J2","J3","J4"])for(let index=0;index<=120;index++){const solution=runtime.solveOutput(output,-10+20*index/120),check=solution.frame.joint_constraint_checks.find(item=>item.joint===jointId);maxLateral=Math.max(maxLateral,check.lateral_gap_mm)}
    items.push({rod,joint_id:jointId,axis_world_xyz:joint.axis_world_xyz,endpoint_direction_world_xyz:endpointDirection,axis_to_endpoint_angle_deg:angleDeg(joint.axis_world_xyz,endpointDirection),initial_marker_offset_from_endpoint_line_mm:markerOffset,nominal_lateral_gap_mm:joint.geometry.lateral_offset_mm,max_lateral_gap_mm:maxLateral,i_axis_local:modelJoint.i_marker_axis_global,j_axis_local:modelJoint.j_marker_axis_global});
  }
  return items;
}

export function analyzePinNearModes(model,runtime,statics,boundary){
  const nominal=runtime.solveOutput("J2",0),assembly=statics.assemble(nominal,boundary),passiveColumns=assembly.unknowns.map((item,index)=>item.category==="passive_constraint_wrench"?index:-1).filter(index=>index>=0),passiveTranspose=passiveColumns.map(column=>assembly.A.map(row=>row[column])),decomposition=scaledSvd(passiveTranspose),nearIndices=[117,118,119],rows=outputRows(assembly),rawModes=nearIndices.map((index,modeIndex)=>{const velocity=decomposition.physicalRightVectors[index],outputRates=Object.fromEntries(Object.entries(rows).map(([key,row])=>[key,dot(row,velocity)]));return{mode_index:modeIndex,singular_index:index,singular_value:decomposition.singularValues[index],output_rates:outputRates,constraint_residual_norm:norm(passiveTranspose.map(row=>dot(row,velocity))),body_twists:bodyTwists(velocity,assembly.bodies)}}),basis=rawModes.map((_,index)=>decomposition.physicalRightVectors[nearIndices[index]]),projection=Object.values(rows).map(row=>basis.map(vector=>dot(row,vector))),projectionSolve=solveRankRevealing(projection,[0,0,0],{relativeTolerance:1e-12}),aligned={};
  for(const [targetIndex,output] of Object.keys(rows).entries()){
    const target=[0,0,0];target[targetIndex]=1;const coefficients=solveRankRevealing(projection,target,{relativeTolerance:1e-12}).particular,velocity=Array(assembly.equations.length).fill(0);for(let mode=0;mode<basis.length;mode++)for(let index=0;index<velocity.length;index++)velocity[index]+=coefficients[mode]*basis[mode][index];aligned[output]={coefficients,output_rates:Object.fromEntries(Object.entries(rows).map(([key,row])=>[key,dot(row,velocity)])),constraint_residual_norm:norm(passiveTranspose.map(row=>dot(row,velocity))),body_twists:bodyTwists(velocity,assembly.bodies)};
  }
  const finiteDifference={};
  for(const output of Object.keys(rows)){const fd=finiteDifferenceVector(runtime,output,assembly.bodies),ranking=residualRanking(assembly,passiveColumns,fd.vector);finiteDifference[output]={step_deg:.001,body_twist_dimension:fd.vector.length,output_rates:Object.fromEntries(Object.entries(rows).map(([key,row])=>[key,dot(row,fd.vector)])),constraint_residual_norm:Math.hypot(...ranking.flatMap(item=>item.components.map(component=>component.value))),ranking,body_twists:bodyTwists(fd.vector,assembly.bodies)}}
  return{model_id:model.meta.model_id,strict_acceptance_changed:false,passive_jacobian:{rows:passiveTranspose.length,columns:passiveTranspose[0].length,strict_rank:120},near_modes:rawModes,output_projection_matrix:projection,output_projection_rank:projectionSolve.rank,output_aligned_modes:aligned,next_non_near_singular_value:decomposition.singularValues[116],spectral_gap_ratio:decomposition.singularValues[116]/decomposition.singularValues[117],finite_difference:finiteDifference,prismatic_audit:prismaticAudit(model,runtime,assembly),interpretation:"The three SVD vectors are an arbitrary mixed basis. Their span has full rank in J2/J3/J4 output coordinates; output_aligned_modes are linear combinations for interpretation only. No SVD threshold is changed for statics acceptance."};
}
