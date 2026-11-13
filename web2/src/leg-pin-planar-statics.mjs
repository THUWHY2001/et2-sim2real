import {evaluateLinearQuantity,solveRankRevealing} from "./rank-revealing-svd.mjs";

const G=9.80665,OUTPUTS=["J2","J3","J4"],ROD_JOINTS={hip:"J_LEG_L_HIP_pitch_T",knee:"J_LEG_L_KNEE_pitch_T",ankle:"J_LEG_L_ANKLE_pitch_T"};
const dot=(a,b)=>a.reduce((sum,value,index)=>sum+value*b[index],0);
const norm=value=>Math.hypot(...value);
const unit=value=>value.map(item=>item/norm(value));
const add=(a,b)=>a.map((value,index)=>value+b[index]);
const scale=(a,factor)=>a.map(value=>value*factor);
const transpose=matrix=>matrix[0].map((_,column)=>matrix.map(row=>row[column]));

export function createLegPinPlanarStatics(model,kinematics,options={}){
  if(model?.meta?.model_id!=="pin-planar-ideal-v1")throw new Error("pin-planar-ideal-v1 required");
  const solverOptions={relativeTolerance:Number(options.relativeTolerance??1e-10),consistencyTolerance:Number(options.consistencyTolerance??1e-9),uniquenessTolerance:Number(options.uniquenessTolerance??1e-8)},gravityScale=Number(options.gravityScale??1),gravity=G*gravityScale;
  function markerPairs(solution){const map=new Map();for(const marker of solution.source_solution.frame.all_joint_marker_sides){if(!map.has(marker.joint))map.set(marker.joint,{});map.get(marker.joint)[marker.marker_side]=[marker.xyz_mm[0]/1000,marker.xyz_mm[2]/1000]}return map}
  function assemble(solution){
    if(!solution?.complete)throw new Error("complete planar kinematic solution required");const pairs=markerPairs(solution),equations=model.bodies.flatMap(body=>["Fx","Fz","My"].map(component=>({body,component}))),bodyIndex=new Map(model.bodies.map((body,index)=>[body,index])),unknowns=[];
    for(const [jointId,joint] of Object.entries(model.joints)){
      const pair=pairs.get(jointId),reference=scale(add(pair.I,pair.J),.5),basis=[];
      if(joint.planar_type==="revolute")basis.push({category:"passive_constraint_wrench",label:"Fx",force:[1,0],moment:0},{category:"passive_constraint_wrench",label:"Fz",force:[0,1],moment:0});
      else if(joint.planar_type==="fixed")basis.push({category:"passive_constraint_wrench",label:"Fx",force:[1,0],moment:0},{category:"passive_constraint_wrench",label:"Fz",force:[0,1],moment:0},{category:"passive_constraint_wrench",label:"My",force:[0,0],moment:1});
      else{
        const theta=solution.poses[joint.i_part].theta_rad,axis=unit([Math.cos(theta)*joint.ideal_axis_i_local_xz[0]+Math.sin(theta)*joint.ideal_axis_i_local_xz[1],-Math.sin(theta)*joint.ideal_axis_i_local_xz[0]+Math.cos(theta)*joint.ideal_axis_i_local_xz[1]]),normal=[axis[1],-axis[0]];
        basis.push({category:"passive_constraint_wrench",label:"F_normal",force:normal,moment:0},{category:"passive_constraint_wrench",label:"My",force:[0,0],moment:1},{category:"actuator_axial_force",label:"F_axial",force:axis,moment:0});
      }
      for(const item of basis){const contributions={};for(const [body,sign] of [[joint.i_part,-1],[joint.j_part,1]])if(body!=="base_link")contributions[body]={force_N:scale(item.force,sign),moment_at_reference_Nm:sign*item.moment,reference_world_m:reference};unknowns.push({id:`${jointId}:${item.category}:${item.label}`,joint_id:jointId,category:item.category,basis_label:item.label,force_basis:item.force,moment_basis:item.moment,reference_world_m:reference,body_contributions:contributions})}
    }
    const A=Array.from({length:equations.length},()=>Array(unknowns.length).fill(0));
    for(let column=0;column<unknowns.length;column++)for(const [body,item] of Object.entries(unknowns[column].body_contributions)){const index=bodyIndex.get(body),com=solution.com_world_xz_mm[body].map(value=>value/1000),relative=[item.reference_world_m[0]-com[0],item.reference_world_m[1]-com[1]];A[3*index][column]=item.force_N[0];A[3*index+1][column]=item.force_N[1];A[3*index+2][column]=item.moment_at_reference_Nm+relative[1]*item.force_N[0]-relative[0]*item.force_N[1]}
    const b=Array(equations.length).fill(0);for(let index=0;index<model.bodies.length;index++)b[3*index+1]=(Number(model.parts[model.bodies[index]].mass_kg)||0)*gravity;
    return{A,b,equations,unknowns,bodies:model.bodies.slice(),outputs:{...solution.outputs}};
  }
  const quantity=(solved,assembly,jointId,kind)=>{const coefficients=assembly.unknowns.map(item=>item.joint_id!==jointId?0:kind==="Fx"?item.force_basis[0]:kind==="Fz"?item.force_basis[1]:kind==="My"?item.moment_basis:0);return evaluateLinearQuantity(solved,coefficients,solverOptions)};
  const potentialEnergy=solution=>model.bodies.reduce((sum,body)=>sum+Number(model.parts[body].mass_kg)*gravity*solution.com_world_xz_mm[body][1]/1000,0);
  function gravityTorque(outputs){const h=.001,result={};for(const output of OUTPUTS){const lo=kinematics.solveAngles({...outputs,[output]:outputs[output]-h}),hi=kinematics.solveAngles({...outputs,[output]:outputs[output]+h});result[output]=(potentialEnergy(hi)-potentialEnergy(lo))/(2*h*Math.PI/180)}return result}
  function virtualWork(solution){
    const length=kinematics.lengthJacobian(solution.outputs),torque=gravityTorque(solution.outputs),system=transpose(length.matrix),rhs=OUTPUTS.map(output=>torque[output]),solved=solveRankRevealing(system,rhs,solverOptions),pushrodForces=Object.entries(ROD_JOINTS).map(([rod,joint_id],index)=>{const coefficients=[0,0,0];coefficients[index]=1;return{rod,joint_id,...evaluateLinearQuantity(solved,coefficients,solverOptions)}}),reconstructed=system.map(row=>dot(row,solved.particular)),residual=reconstructed.map((value,index)=>value-rhs[index]);return{length_jacobian:length,gravity_torque_Nm:torque,pushrod_forces:pushrodForces,residual_Nm:residual,residual_norm_Nm:norm(residual),matrix:{rank:solved.rank,condition_number:solved.conditionNumber}}
  }
  function solve(solution){
    const assembly=assemble(solution),linear=solveRankRevealing(assembly.A,assembly.b,solverOptions),unknownValues=assembly.unknowns.map((item,index)=>{const coefficients=Array(assembly.unknowns.length).fill(0);coefficients[index]=1;return{joint_id:item.joint_id,category:item.category,basis_label:item.basis_label,...evaluateLinearQuantity(linear,coefficients,solverOptions)}}),jointReactions=Object.keys(model.joints).map(jointId=>{const jointType=model.joints[jointId].planar_type,Fx=quantity(linear,assembly,jointId,"Fx"),Fz=quantity(linear,assembly,jointId,"Fz"),My=jointType==="revolute"?{status:"not_transmitted",value:null}:quantity(linear,assembly,jointId,"My"),forceUnique=Fx.status==="unique"&&Fz.status==="unique";return{joint_id:jointId,joint_type:jointType,components:{Fx_N:Fx,Fz_N:Fz,My_Nm:My},force_magnitude_N:forceUnique?Math.hypot(Fx.value,Fz.value):null,force_direction_deg:forceUnique?Math.atan2(Fz.value,Fx.value)*180/Math.PI:null}}),actuatorForces=unknownValues.filter(item=>item.category==="actuator_axial_force"),byBody={};for(let index=0;index<model.bodies.length;index++){const vector=linear.residualVector.slice(3*index,3*index+3);byBody[model.bodies[index]]={force_N:vector.slice(0,2),moment_Nm:vector[2],force_norm_N:norm(vector.slice(0,2)),moment_abs_Nm:Math.abs(vector[2])}}
    const work=virtualWork(solution);return{complete:linear.consistent&&solution.complete,model_id:model.meta.model_id,boundary:{load:"gravity-quasi-static-planar",J5_deg:0,excluded:["inertia","assembly_preload","friction","3d_support_load_distribution","J5_roll"]},assembly,matrix:{rows:assembly.A.length,columns:assembly.A[0].length,rank:linear.rank,right_nullity:linear.nullity,consistent:linear.consistent,condition_number:linear.conditionNumber},residuals:{norm:linear.residualNorm,by_body:byBody,max_force_norm_N:Math.max(...Object.values(byBody).map(item=>item.force_norm_N)),max_moment_abs_Nm:Math.max(...Object.values(byBody).map(item=>item.moment_abs_Nm))},gravity_torque_Nm:work.gravity_torque_Nm,length_jacobian:work.length_jacobian,virtual_work:work,actuator_forces:actuatorForces,joint_reactions:jointReactions,unknown_values:unknownValues,potential_energy_J:potentialEnergy(solution)}
  }
  return Object.freeze({assemble,solve,gravityTorque,virtualWork,potentialEnergy});
}
