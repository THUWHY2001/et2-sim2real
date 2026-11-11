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
        friction_eligible:definition.type!=="fixed",reference_point_world_m:reference,geometry,
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
    const A=Array.from({length:equations.length},()=>Array(unknowns.length).fill(0));
    for(let column=0;column<unknowns.length;column++)for(const [body,item] of Object.entries(unknowns[column].body_contributions)){
      const index=bodyIndex.get(body);if(index===undefined)continue;
      for(let component=0;component<3;component++)A[6*index+component][column]=item.force_N[component];
      for(let component=0;component<3;component++)A[6*index+3+component][column]=item.moment_about_world_origin_Nm[component];
    }
    const b=Array(equations.length).fill(0);
    for(const [body,index] of bodyIndex){
      const mass=Number(model.parts[body].mass_kg),com=toMeters(solution.frame.mass.COM_world_mm[body]),gravity=[0,0,-mass*GRAVITY_M_S2],moment=cross(com,gravity);
      for(let component=0;component<3;component++)b[6*index+component]=-gravity[component];
      for(let component=0;component<3;component++)b[6*index+3+component]=-moment[component];
    }
    return{A,b,unknowns,equations,joints,boundary:structuredClone(boundary),bodies:bodyNames.slice(),gravity_m_s2:[0,0,-GRAVITY_M_S2]};
  }
  return Object.freeze({assemble});
}

