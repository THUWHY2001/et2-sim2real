const GRAVITY_M_S2=9.80665;
const WORLD_AXES=[[1,0,0],[0,1,0],[0,0,1]];
const dot=(a,b)=>a.reduce((sum,value,index)=>sum+value*b[index],0);
const norm=value=>Math.hypot(...value);
const add=(a,b)=>a.map((value,index)=>value+b[index]);
const scale=(value,factor)=>value.map(item=>item*factor);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const unit=value=>{const length=norm(value);if(!length)throw new Error("joint axis must be non-zero");return scale(value,1/length)};
const clamp=(value,min,max)=>Math.max(min,Math.min(max,value));
const toMeters=value=>scale(value,.001);

export function solveAssembledSystem(assembly,options={}){
  if(!assembly?.A||!assembly?.b||!assembly?.unknowns||!assembly?.equations)throw new Error("assembled A, b, unknowns and equations are required");
  const solved=solveRankRevealing(assembly.A,assembly.b,options),unknownValues=assembly.unknowns.map((unknown,index)=>{
    const coefficients=Array(assembly.unknowns.length).fill(0);coefficients[index]=1;
    return{...evaluateLinearQuantity(solved,coefficients,options),id:unknown.id,joint_id:unknown.joint_id,category:unknown.category,basis_label:unknown.basis_label,index};
  });
  const byBody={};
  assembly.equations.forEach((equation,index)=>{if(!byBody[equation.body])byBody[equation.body]=[];byBody[equation.body].push(solved.residualVector[index])});
  return{
    assembly,solution:solved,unknownValues,
    matrix:{consistent:solved.consistent,rank:solved.rank,augmented_rank:solved.augmentedRank,nullity:solved.nullity,condition_number:solved.conditionNumber,residual_norm:solved.residualNorm,svd:solved.svd},
    residuals:{vector:solved.residualVector,byBody,max_body_norm:Math.max(0,...Object.values(byBody).map(value=>norm(value)))},
  };
}

function perpendicularBasis(axis){
  const seed=Math.abs(axis[0])<.8?[1,0,0]:[0,1,0],first=unit(cross(axis,seed)),second=unit(cross(axis,first));
  return[first,second];
}

function validateBoundary(model,boundary){
  const required=["worldInterface","footContact","heldRevolutes","toeConstraint","loadCase"];
  if(!boundary||required.some(key=>boundary[key]===undefined))throw new Error(`boundary fields required: ${required.join(", ")}`);
  if(boundary.footContact!=="none")throw new Error("first-stage boundary requires footContact none");
  if(!Array.isArray(boundary.heldRevolutes))throw new Error("boundary heldRevolutes must be an array");
  const interfaceJoint=model.joints[boundary.worldInterface];
  if(!interfaceJoint||interfaceJoint.i_part!=="base_link")throw new Error("boundary worldInterface must be the existing base_link interface");
}

function markerMap(solution){
  const map=new Map();
  for(const marker of solution.frame.all_joint_marker_sides){
    if(!map.has(marker.joint))map.set(marker.joint,{});
    map.get(marker.joint)[marker.marker_side.toLowerCase()]=marker;
  }
  return map;
}

function prismaticGeometry(iMarker,jMarker,tolerances){
  const iAxis=unit(iMarker.axis_xyz),rawJ=unit(jMarker.axis_xyz),alignment=dot(iAxis,rawJ),jAxis=alignment<0?scale(rawJ,-1):rawJ;
  const separation=iMarker.xyz_mm.map((value,index)=>jMarker.xyz_mm[index]-value),axial=dot(separation,iAxis),lateralVector=add(separation,scale(iAxis,-axial));
  const details={
    axial_separation_mm:axial,
    lateral_offset_mm:norm(lateralVector),
    axis_misalignment_deg:Math.acos(clamp(dot(iAxis,jAxis),-1,1))*180/Math.PI,
    common_axis_xyz:iAxis,
  };
  if(details.lateral_offset_mm>tolerances.prismaticLateralMm||details.axis_misalignment_deg>tolerances.prismaticAxisDeg){
    const error=new Error(`prismatic geometry invalid: lateral ${details.lateral_offset_mm} mm, axis ${details.axis_misalignment_deg} deg`);
    error.code="PRISMATIC_GEOMETRY_INVALID";error.details=details;throw error;
  }
  return details;
}

function makeBasis(jointType,axis,held){
  const [u,v]=perpendicularBasis(axis),zero=[0,0,0],passive=[],actuator=[];
  const force=(direction,label)=>({force:direction,moment:zero,label});
  const moment=(direction,label)=>({force:zero,moment:direction,label});
  if(jointType==="spherical")WORLD_AXES.forEach((direction,index)=>passive.push(force(direction,`F${"xyz"[index]}`)));
  else if(jointType==="revolute"){
    WORLD_AXES.forEach((direction,index)=>passive.push(force(direction,`F${"xyz"[index]}`)));
    (held?WORLD_AXES:[u,v]).forEach((direction,index)=>passive.push(moment(direction,held?`M${"xyz"[index]}`:`M_perp_${index+1}`)));
  }else if(jointType==="translational"){
    [u,v].forEach((direction,index)=>passive.push(force(direction,`F_radial_${index+1}`)));
    WORLD_AXES.forEach((direction,index)=>passive.push(moment(direction,`M${"xyz"[index]}`)));
    actuator.push(force(axis,"F_axial"));
  }else if(jointType==="fixed"){
    WORLD_AXES.forEach((direction,index)=>passive.push(force(direction,`F${"xyz"[index]}`)));
    WORLD_AXES.forEach((direction,index)=>passive.push(moment(direction,`M${"xyz"[index]}`)));
  }else throw new Error(`unsupported joint type ${jointType}`);
  return{passive,actuator};
}

function contribution(force,momentAtReference,reference,marker,sign){
  const signedForce=scale(force,sign),signedMoment=scale(momentAtReference,sign),momentAtMarker=add(signedMoment,cross(add(reference,scale(marker,-1)),signedForce));
  return{
    force_N:signedForce,
    moment_at_marker_Nm:momentAtMarker,
    moment_about_world_origin_Nm:add(signedMoment,cross(reference,signedForce)),
  };
}

export function createLegGravityStatics(model,runtime,options={}){
  if(!model?.parts||!model?.joints)throw new Error("leg model parts and joints are required");
  if(!runtime?.solveOutput)throw new Error("complete leg runtime is required");
  const bodyNames=Object.keys(model.parts),bodyIndex=new Map(bodyNames.map((name,index)=>[name,index]));
  const tolerances={prismaticLateralMm:Number(options.prismaticLateralMm??1e-3),prismaticAxisDeg:Number(options.prismaticAxisDeg??1e-3)};
  const solverOptions={relativeTolerance:Number(options.relativeTolerance??1e-10),consistencyTolerance:Number(options.consistencyTolerance??1e-6),uniquenessTolerance:Number(options.uniquenessTolerance??1e-8)};

  function assemble(solution,boundary){
    validateBoundary(model,boundary);
    if(!solution?.complete||!solution.frame)throw new Error("complete closed-chain solution required");
    const markers=markerMap(solution),equations=[];
    for(const body of bodyNames)for(const component of ["Fx","Fy","Fz","Mx","My","Mz"])equations.push({body,component});
    const unknowns=[],joints=[];
    for(const [jointId,definition] of Object.entries(model.joints)){
      const pair=markers.get(jointId);if(!pair?.i||!pair?.j)throw new Error(`missing Marker pair for ${jointId}`);
      const iPoint=toMeters(pair.i.xyz_mm),jPoint=toMeters(pair.j.xyz_mm),reference=scale(add(iPoint,jPoint),.5),axis=unit(pair.i.axis_xyz);
      const held=definition.type==="revolute"&&boundary.heldRevolutes.includes(jointId),basis=makeBasis(definition.type,axis,held);
      const geometry=definition.type==="translational"?prismaticGeometry(pair.i,pair.j,tolerances):{
        marker_separation_mm:norm(pair.i.xyz_mm.map((value,index)=>pair.j.xyz_mm[index]-value)),
        axis_misalignment_deg:Math.acos(clamp(Math.abs(dot(unit(pair.i.axis_xyz),unit(pair.j.axis_xyz))),-1,1))*180/Math.PI,
      };
      const metadata={
        joint_id:jointId,joint_type:definition.type,body_i:definition.i_part,body_j:definition.j_part,
        motion_state:held?"position-held":definition.type==="revolute"?"free-output":definition.type==="translational"?"actuated-translation":"passive",
        constraint_dimension:basis.passive.length+basis.actuator.length,passive_constraint_dimension:basis.passive.length,actuator_dimension:basis.actuator.length,
        friction_eligible:definition.type!=="fixed",reference_point_world_m:reference,axis_world_xyz:axis,geometry,
      };
      joints.push(metadata);
      for(const [category,items] of [["passive_constraint_wrench",basis.passive],["actuator_axial_force",basis.actuator]])for(const item of items){
        const iContribution=contribution(item.force,item.moment,reference,iPoint,-1),jContribution=contribution(item.force,item.moment,reference,jPoint,1);
        unknowns.push({
          id:`${jointId}:${category}:${item.label}`,joint_id:jointId,category,basis_label:item.label,
          wrench_at_reference:{force_N:item.force,moment_Nm:item.moment},
          body_contributions:{[definition.i_part]:iContribution,[definition.j_part]:jContribution},
        });
      }
    }
    const bodyCom=Object.fromEntries(bodyNames.map(body=>[body,toMeters(solution.frame.mass.COM_world_mm[body])]));
    const A=Array.from({length:equations.length},()=>Array(unknowns.length).fill(0));
    for(let column=0;column<unknowns.length;column++)for(const [body,item] of Object.entries(unknowns[column].body_contributions)){
      const index=bodyIndex.get(body);if(index===undefined)continue;
      for(let component=0;component<3;component++)A[6*index+component][column]=item.force_N[component];
      const momentAboutCom=add(item.moment_about_world_origin_Nm,scale(cross(bodyCom[body],item.force_N),-1));
      for(let component=0;component<3;component++)A[6*index+3+component][column]=momentAboutCom[component];
    }
    const b=Array(equations.length).fill(0);
    for(const [body,index] of bodyIndex){
      const mass=Number(model.parts[body].mass_kg),gravity=[0,0,-mass*GRAVITY_M_S2],moment=[0,0,0];
      for(let component=0;component<3;component++)b[6*index+component]=-gravity[component];
      for(let component=0;component<3;component++)b[6*index+3+component]=-moment[component];
    }
    return{A,b,unknowns,equations,joints,boundary:structuredClone(boundary),bodies:bodyNames.slice(),gravity_m_s2:[0,0,-GRAVITY_M_S2]};
  }

  const wrenchComponents=(solved,assembly,jointId,categories)=>{
    const result={};
    for(const [component,offset] of [["Fx_N",0],["Fy_N",1],["Fz_N",2],["Mx_Nm",3],["My_Nm",4],["Mz_Nm",5]]){
      const coefficients=assembly.unknowns.map(unknown=>{
        if(unknown.joint_id!==jointId||!categories.includes(unknown.category))return 0;
        return offset<3?unknown.wrench_at_reference.force_N[offset]:unknown.wrench_at_reference.moment_Nm[offset-3];
      });
      result[component]=evaluateLinearQuantity(solved,coefficients,solverOptions);
    }
    return result;
  };
  const wrenchResult=(solved,assembly,jointId,categories)=>{
    const components=wrenchComponents(solved,assembly,jointId,categories),force=[components.Fx_N,components.Fy_N,components.Fz_N],moment=[components.Mx_Nm,components.My_Nm,components.Mz_Nm];
    const magnitude=items=>items.every(item=>item.status==="unique")?{status:"unique",value:Math.hypot(...items.map(item=>item.value))}:{status:"indeterminate",value:null};
    return{components,force_magnitude_N:magnitude(force),moment_magnitude_Nm:magnitude(moment)};
  };
  const projectedQuantity=(solved,assembly,jointId,categories,kind,direction)=>{
    const coefficients=assembly.unknowns.map(unknown=>{
      if(unknown.joint_id!==jointId||!categories.includes(unknown.category))return 0;
      return dot(kind==="force"?unknown.wrench_at_reference.force_N:unknown.wrench_at_reference.moment_Nm,direction);
    });
    return evaluateLinearQuantity(solved,coefficients,solverOptions);
  };

  function solve(solution,boundary){
    const assembly=assemble(solution,boundary),base=solveAssembledSystem(assembly,solverOptions),jointReactions=assembly.joints.map(joint=>{
      const passive=wrenchResult(base.solution,assembly,joint.joint_id,["passive_constraint_wrench"]),total=wrenchResult(base.solution,assembly,joint.joint_id,["passive_constraint_wrench","actuator_axial_force"]);
      const actuatorIndex=assembly.unknowns.findIndex(item=>item.joint_id===joint.joint_id&&item.category==="actuator_axial_force");
      const [radial1,radial2]=perpendicularBasis(joint.axis_world_xyz),allCategories=["passive_constraint_wrench","actuator_axial_force"],axial=projectedQuantity(base.solution,assembly,joint.joint_id,allCategories,"force",joint.axis_world_xyz),radialA=projectedQuantity(base.solution,assembly,joint.joint_id,allCategories,"force",radial1),radialB=projectedQuantity(base.solution,assembly,joint.joint_id,allCategories,"force",radial2);
      return{
        ...joint,
        passive_constraint_wrench:passive,
        actuator_axial_force:actuatorIndex<0?null:base.unknownValues[actuatorIndex],
        total_joint_wrench:total,
        force_decomposition:{
          axial_N:axial,radial_1_N:radialA,radial_2_N:radialB,
          radial_magnitude_N:radialA.status==="unique"&&radialB.status==="unique"?{status:"unique",value:Math.hypot(radialA.value,radialB.value)}:{status:"indeterminate",value:null},
        },
        contact_load_mapping:"not_mapped",
      };
    });
    const actuatorForces=jointReactions.filter(item=>item.actuator_axial_force).map(item=>({joint_id:item.joint_id,...item.actuator_axial_force}));
    let maxForce=0,maxMoment=0;
    for(const unknown of assembly.unknowns){
      const items=Object.values(unknown.body_contributions),force=items.reduce((sum,item)=>add(sum,item.force_N),[0,0,0]),moment=items.reduce((sum,item)=>add(sum,item.moment_about_world_origin_Nm),[0,0,0]);
      maxForce=Math.max(maxForce,norm(force));maxMoment=Math.max(maxMoment,norm(moment));
    }
    const holdingTorques=jointReactions.filter(item=>item.motion_state==="position-held").map(item=>({joint_id:item.joint_id,...projectedQuantity(base.solution,assembly,item.joint_id,["passive_constraint_wrench","actuator_axial_force"],"moment",item.axis_world_xyz)}));
    const interfaceReaction=jointReactions.find(item=>item.joint_id===boundary.worldInterface),interfaceComponents=interfaceReaction?.total_joint_wrench.components,interfaceUnique=interfaceComponents&&Object.values(interfaceComponents).every(item=>item.status==="unique");
    let globalBalance={status:"indeterminate",force_residual_N:null,moment_residual_Nm:null};
    if(interfaceUnique){
      const force=[interfaceComponents.Fx_N.value,interfaceComponents.Fy_N.value,interfaceComponents.Fz_N.value],moment=[interfaceComponents.Mx_Nm.value,interfaceComponents.My_Nm.value,interfaceComponents.Mz_Nm.value],reference=interfaceReaction.reference_point_world_m;
      let gravityForce=[0,0,0],gravityMoment=[0,0,0];
      for(const body of bodyNames){const gravity=[0,0,-Number(model.parts[body].mass_kg)*GRAVITY_M_S2],com=toMeters(solution.frame.mass.COM_world_mm[body]);gravityForce=add(gravityForce,gravity);gravityMoment=add(gravityMoment,cross(com,gravity))}
      globalBalance={status:"unique",force_residual_N:add(force,gravityForce),moment_residual_Nm:add(add(moment,cross(reference,force)),gravityMoment)};
    }
    return{
      ...base,
      pose:{output_degrees:{J2:runtime.nominal.J2,J3:runtime.nominal.J3,J4:runtime.nominal.J4}},
      jointReactions,actuatorForces,holdingTorques,globalBalance,
      internalWrenchBalance:{max_force_N:maxForce,max_moment_Nm:maxMoment},
      loadInterpretation:{label:"重力准静态理论约束扳手",contact_normal_load:"not_mapped",limitations:["gravity-only","no-inertia","no-preload","no-friction-feedback","airborne-foot"]},
      acceptance:{relative_equilibrium_tolerance:solverOptions.consistencyTolerance,source_geometry_precision_limited:true},
    };
  }
  const solveNominal=boundary=>solve(runtime.solveOutput("J2",0),boundary);
  return Object.freeze({assemble,solve,solveNominal});
}

import {evaluateLinearQuantity,solveRankRevealing} from "./rank-revealing-svd.mjs";

