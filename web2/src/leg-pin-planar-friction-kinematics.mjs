import {solveRankRevealing} from "./rank-revealing-svd.mjs";

const OUTPUTS=["J2","J3","J4"];
const dot=(a,b)=>a.reduce((sum,value,index)=>sum+value*b[index],0);
const norm=value=>Math.hypot(...value);
const wrap=value=>Math.atan2(Math.sin(value),Math.cos(value));
const multiply=(matrix,vector)=>matrix.map(row=>dot(row,vector));

export function createLegPinPlanarFrictionKinematics(model,kinematics,catalog){
  if(model?.meta?.model_id!=="pin-planar-ideal-v1")throw new Error("pin-planar-ideal-v1 required");
  const groupByJoint=new Map(catalog.groups.map(item=>[item.joint_id,item]));
  const theta=(solution,body)=>body==="base_link"?0:solution.poses[body].theta_rad;
  const relativeAngle=(solution,group)=>wrap(theta(solution,group.body_j)-theta(solution,group.body_i));
  const solvePerturbed=(solution,output,deltaDeg)=>kinematics.solveAngles({...solution.outputs,[output]:solution.outputs[output]+deltaDeg});

  function perturbations(solution,stepDeg){return Object.fromEntries(OUTPUTS.map(output=>[output,{lo:solvePerturbed(solution,output,-stepDeg),hi:solvePerturbed(solution,output,stepDeg)}]))}
  function jacobianFromPerturbations(group,sets,stepDeg){const denominator=2*stepDeg*Math.PI/180,values={};for(const output of OUTPUTS){const {lo,hi}=sets[output];if(group.planar_joint_type==="prismatic"){const rod=model.joints[group.joint_id].rod;values[output]=(hi.rod_lengths_m[rod]-lo.rod_lengths_m[rod])/denominator}else values[output]=wrap(relativeAngle(hi,group)-relativeAngle(lo,group))/denominator}return{joint_id:group.joint_id,rate_type:group.planar_joint_type==="prismatic"?"linear":"angular",units:group.planar_joint_type==="prismatic"?"m_per_rad":"rad_per_rad",step_deg:stepDeg,values}}

  function relativeRateJacobian(solution,group,options={}){
    const stepDeg=Number(options.step_deg??.001);return jacobianFromPerturbations(group,perturbations(solution,stepDeg),stepDeg);
  }

  function allRelativeRateJacobians(solution,options={}){const stepDeg=Number(options.step_deg??.001),sets=perturbations(solution,stepDeg);return new Map(catalog.groups.map(group=>[group.joint_id,jacobianFromPerturbations(group,sets,stepDeg)]))}

  function relativeVelocity(solution,group,outputRates,options={}){const jacobian=relativeRateJacobian(solution,group,options),value=OUTPUTS.reduce((sum,output)=>sum+jacobian.values[output]*Number(outputRates[output]??0),0);return{joint_id:group.joint_id,jacobian,...(jacobian.rate_type==="angular"?{value_rad_s:value}:{value_m_s:value})}}

  function bodyTwist(solution,output,stepDeg=.001){const lo=solvePerturbed(solution,output,-stepDeg),hi=solvePerturbed(solution,output,stepDeg),denominator=2*stepDeg*Math.PI/180,vector=[];for(const body of model.bodies){vector.push((hi.com_world_xz_mm[body][0]-lo.com_world_xz_mm[body][0])/denominator,(hi.com_world_xz_mm[body][1]-lo.com_world_xz_mm[body][1])/denominator,wrap(hi.poses[body].theta_rad-lo.poses[body].theta_rad)/denominator)}return vector}
  const omegaFromVector=(vector,body)=>body==="base_link"?0:vector[3*model.bodies.indexOf(body)+2];

  function crossCheckRelativeRates(solution,options={}){
    const stepDeg=Number(options.step_deg??.001),A=solution.constraint_jacobian.matrix,solved=solveRankRevealing(A,Array(A.length).fill(0),{relativeTolerance:1e-10}),columns={};let maxConstraint=0;
    for(const output of OUTPUTS){const finite=bodyTwist(solution,output,stepDeg),residual=norm(multiply(A,finite)),projected=Array(finite.length).fill(0);for(const basis of solved.nullspace){const coefficient=dot(finite,basis);for(let index=0;index<projected.length;index++)projected[index]+=coefficient*basis[index]}columns[output]={finite,projected,constraint_velocity_residual:residual,projection_residual:norm(finite.map((value,index)=>value-projected[index]))};maxConstraint=Math.max(maxConstraint,residual)}
    let maxDifference=0;const contacts=catalog.groups.map(group=>{const primary=relativeRateJacobian(solution,group,{step_deg:stepDeg}),independent={};if(group.planar_joint_type!=="prismatic")for(const output of OUTPUTS){independent[output]=omegaFromVector(columns[output].projected,group.body_j)-omegaFromVector(columns[output].projected,group.body_i);maxDifference=Math.max(maxDifference,Math.abs(primary.values[output]-independent[output]))}return{joint_id:group.joint_id,rate_type:primary.rate_type,primary:primary.values,independent:group.planar_joint_type==="prismatic"?null:independent,status:group.planar_joint_type==="prismatic"?"not_checked_linear_contact":"checked"}});
    return{method:"constraint_nullspace_output_aligned_projection",rank:solved.rank,nullity:solved.nullity,max_constraint_velocity_residual:maxConstraint,max_relative_rate_difference:maxDifference,outputs:columns,contacts};
  }

  return Object.freeze({relativeRateJacobian,allRelativeRateJacobians,relativeVelocity,crossCheckRelativeRates,groupByJoint});
}
