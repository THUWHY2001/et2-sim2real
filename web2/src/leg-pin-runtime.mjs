import {createLegFullRuntime} from "./leg-full-runtime.mjs";
import {createPinJointModel, PIN_JOINT_MAPPINGS} from "./leg-pin-model.mjs";

const ROD_BODIES=Object.freeze({
  hip:{outer:"leg_l_link10",inner:"leg_l_link11",upper:"P11",lower:"P13"},
  knee:{outer:"leg_l_link13",inner:"leg_l_link14",upper:"P14",lower:"P16"},
  ankle:{outer:"leg_l_link18",inner:"leg_l_link19",upper:"P19",lower:"P21"},
});
const PIN_POSITION_TOLERANCE_MM=2e-5;
const PIN_AXIS_TOLERANCE_DEG=2e-6;
const dot=(a,b)=>a.reduce((sum,value,index)=>sum+value*b[index],0);
const add=(a,b)=>a.map((value,index)=>value+b[index]);
const sub=(a,b)=>a.map((value,index)=>value-b[index]);
const scale=(a,factor)=>a.map(value=>value*factor);
const norm=value=>Math.hypot(...value);
const unit=value=>{const length=norm(value);if(length<1e-14)throw new Error("zero-length direction");return scale(value,1/length)};
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const clamp=(value,min,max)=>Math.max(min,Math.min(max,value));
const transpose=matrix=>matrix[0].map((_,column)=>matrix.map(row=>row[column]));
const matrixVector=(matrix,vector)=>matrix.map(row=>dot(row,vector));
const multiply=(a,b)=>a.map(row=>b[0].map((_,column)=>row.reduce((sum,value,index)=>sum+value*b[index][column],0)));
const transformPoint=(transform,point)=>add(matrixVector(transform.R,point),transform.t_mm);
const axisAngleDeg=(a,b)=>Math.acos(clamp(Math.abs(dot(unit(a),unit(b))),-1,1))*180/Math.PI;
const columns=(a,b,c)=>a.map((_,row)=>[a[row],b[row],c[row]]);

function orthogonalFrame(pin,longitudinal){
  const z=unit(longitudinal),y0=sub(pin,scale(z,dot(pin,z))),y=unit(y0),x=unit(cross(y,z));
  return columns(x,y,z);
}

function sideLocalAxis(joint,side){return joint[`${side.toLowerCase()}_marker_axis_local`]??joint[`${side.toLowerCase()}_marker_axis_global`]??[0,0,1]}

function pinJointByHardpoint(model,hardpoint){
  const mapping=PIN_JOINT_MAPPINGS.find(item=>item.hardpoint===hardpoint);
  return model.joints[mapping.pin_joint_id];
}

function structuralSide(joint,rodBody){return joint.i_part===rodBody?"J":"I"}
function rodSide(joint,rodBody){return joint.i_part===rodBody?"I":"J"}

function correctedRodTransforms(frame,model){
  const transforms=structuredClone(frame.world_transforms);
  for(const definition of Object.values(ROD_BODIES)){
    const upperJoint=pinJointByHardpoint(model,definition.upper),lowerJoint=pinJointByHardpoint(model,definition.lower);
    const upperStructuralSide=structuralSide(upperJoint,definition.outer),lowerStructuralSide=structuralSide(lowerJoint,definition.inner);
    const upperStructuralBody=upperJoint[`${upperStructuralSide.toLowerCase()}_part`],lowerStructuralBody=lowerJoint[`${lowerStructuralSide.toLowerCase()}_part`];
    const upper0=upperJoint.initial_center_global_mm,lower0=lowerJoint.initial_center_global_mm;
    const upper=transformPoint(transforms[upperStructuralBody],upper0),lower=transformPoint(transforms[lowerStructuralBody],lower0);
    const pinWorld=matrixVector(transforms[upperStructuralBody].R,sideLocalAxis(upperJoint,upperStructuralSide));
    const upperRodSide=rodSide(upperJoint,definition.outer),pinLocal=sideLocalAxis(upperJoint,upperRodSide);
    const localFrame=orthogonalFrame(pinLocal,sub(lower0,upper0)),worldFrame=orthogonalFrame(pinWorld,sub(lower,upper));
    const R=multiply(worldFrame,transpose(localFrame));
    transforms[definition.outer]={R,t_mm:sub(upper,matrixVector(R,upper0))};
    transforms[definition.inner]={R,t_mm:sub(lower,matrixVector(R,lower0))};
  }
  return transforms;
}

function buildMarkerSides(model,transforms){
  const sides=[];
  for(const [jointId,joint] of Object.entries(model.joints))for(const side of ["I","J"]){
    const lower=side.toLowerCase(),part=joint[`${lower}_part`],transform=transforms[part],initial=joint.initial_center_global_mm;
    sides.push({joint:jointId,marker_side:side,marker:joint[`${lower}_marker`],part,type:joint.type,initial_xyz_mm:initial.slice(),xyz_mm:transformPoint(transform,initial),axis_xyz:unit(matrixVector(transform.R,sideLocalAxis(joint,side))),cmd_line:joint.source_cmd_line});
  }
  return sides;
}

function auditJoints(model,markerSides){
  const byJoint=new Map();for(const marker of markerSides){if(!byJoint.has(marker.joint))byJoint.set(marker.joint,{});byJoint.get(marker.joint)[marker.marker_side]=marker}
  return Object.entries(model.joints).map(([jointId,joint])=>{
    const pair=byJoint.get(jointId),difference=sub(pair.J.xyz_mm,pair.I.xyz_mm),position=norm(difference),axisCross=norm(cross(pair.I.axis_xyz,pair.J.axis_xyz)),base={joint:jointId,type:joint.type,position_residual_mm:position,constraint_residual_mm:position,axis_cross_error:axisCross,axis_line_angle_deg:axisAngleDeg(pair.I.axis_xyz,pair.J.axis_xyz)};
    if(joint.type==="translational"){
      const axial=dot(difference,pair.I.axis_xyz),lateral=norm(sub(difference,scale(pair.I.axis_xyz,axial)));
      return{...base,axial_sliding_signed_mm:axial,lateral_gap_mm:lateral,constraint_residual_mm:lateral};
    }
    return base;
  });
}

function rebuildMass(model,transforms){
  const COM_world_mm={},weighted=[0,0,0];let total_kg=0;
  for(const [name,part] of Object.entries(model.parts)){
    const mass=Number(part.mass_kg)||0,com=transformPoint(transforms[name],part.com_initial_global_mm);COM_world_mm[name]=com;total_kg+=mass;
    for(let index=0;index<3;index++)weighted[index]+=mass*com[index];
  }
  return{total_kg,COM_world_mm,aggregate_COM_world_mm:total_kg?weighted.map(value=>value/total_kg):[0,0,0]};
}

export function validatePinConstraints(frame,model){
  const markerSides=buildMarkerSides(model,frame.world_transforms),checks=auditJoints(model,markerSides).filter(item=>model.joints[item.joint]?.legacy_joint_id).map(item=>({...item,compatible:item.position_residual_mm<=PIN_POSITION_TOLERANCE_MM&&item.axis_line_angle_deg<=PIN_AXIS_TOLERANCE_DEG}));
  return{ok:checks.length===6&&checks.every(item=>item.compatible),checks};
}

export function createLegPinRuntime(legacyModel){
  const legacy=createLegFullRuntime(legacyModel),model=createPinJointModel(legacyModel);
  const correct=solution=>{
    const result=structuredClone(solution),frame=result.frame,worldTransforms=correctedRodTransforms(frame,model);
    frame.world_transforms=worldTransforms;frame.model_id=model.meta.model_id;frame.all_joint_marker_sides=buildMarkerSides(model,worldTransforms);frame.joint_constraint_checks=auditJoints(model,frame.all_joint_marker_sides);frame.mass=rebuildMass(model,worldTransforms);
    const audit=validatePinConstraints(frame,model);frame.pin_constraint_checks=audit.checks;frame.validation={...frame.validation,pin_revolute_compatible:audit.ok,ok:Boolean(frame.validation.ok&&audit.ok)};
    result.complete=Boolean(solution.complete&&audit.ok);result.residualMm=Math.max(...frame.joint_constraint_checks.map(item=>Number(item.constraint_residual_mm)||0));return result;
  };
  const solveRods=inputs=>correct(legacy.solveRods(inputs));
  const solveOutput=(joint,delta)=>correct(legacy.solveOutput(joint,delta));
  const signedRates=(joint,delta)=>{
    const result=legacy.signedRates(joint,delta);
    return{...result,solution:solveOutput(joint,delta),definition:"positive actuator force on J body dotted with signed relative velocity; pin-revolute-v1 geometry"};
  };
  const potentialEnergy=solution=>Object.entries(solution.frame.mass.COM_world_mm).reduce((sum,[name,com])=>sum+(Number(model.parts[name].mass_kg)||0)*9.80665*com[2]/1000,0);
  const gravityHoldingTorque=(joint,delta)=>{const h=.001,lo=solveOutput(joint,delta-h),hi=solveOutput(joint,delta+h);return lo.complete&&hi.complete?{complete:true,torqueNm:(potentialEnergy(hi)-potentialEnergy(lo))/(2*h*Math.PI/180)}:{complete:false,torqueNm:NaN}};
  return Object.freeze({model,nominal:{...legacy.nominal},solveRods,solveOutput,signedRates,gravityHoldingTorque,validatePinConstraints:(frame,override=model)=>validatePinConstraints(frame,override),legacy});
}
