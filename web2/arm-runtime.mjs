import {createLeftArmBlackbox} from './left-arm-blackbox.mjs';

const finite=value=>Number.isFinite(Number(value));
const add=(a,b)=>a.map((v,i)=>v+b[i]),sub=(a,b)=>a.map((v,i)=>v-b[i]),scale=(a,s)=>a.map(v=>v*s),dot=(a,b)=>a.reduce((sum,v,i)=>sum+v*b[i],0),norm=a=>Math.hypot(...a),unit=a=>{const n=norm(a);return n?scale(a,1/n):[0,0,0]},cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const I=()=>[[1,0,0],[0,1,0],[0,0,1]],mul=(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((sum,v,k)=>sum+v*b[k][j],0))),mv=(a,v)=>a.map(row=>dot(row,v)),tr=a=>a[0].map((_,i)=>a.map(row=>row[i]));
const rad=degrees=>degrees*Math.PI/180,clamp=(value,min,max)=>Math.max(min,Math.min(max,value));
const axisAngle=(axis,angle)=>{const [x,y,z]=unit(axis),c=Math.cos(angle),s=Math.sin(angle),t=1-c;return[[t*x*x+c,t*x*y-s*z,t*x*z+s*y],[t*x*y+s*z,t*y*y+c,t*y*z-s*x],[t*x*z-s*y,t*y*z+s*x,t*z*z+c]]};
const rotationFromTo=(from,to)=>{const a=unit(from),b=unit(to),c=clamp(dot(a,b),-1,1);if(c>1-1e-12)return I();if(c<-1+1e-12)return axisAngle(cross(a,Math.abs(a[0])<.8?[1,0,0]:[0,1,0]),Math.PI);const w=cross(a,b);return axisAngle(w,Math.atan2(norm(w),c))};
const identityTransform=()=>({R:I(),t:[0,0,0]}),apply=(T,p)=>add(mv(T.R,p),T.t),compose=(A,B)=>({R:mul(A.R,B.R),t:add(mv(A.R,B.t),A.t)}),about=(pivot,axis,angle)=>{const R=axisAngle(axis,angle);return{R,t:sub(pivot,mv(R,pivot))}},rigidBetween=(anchor0,anchorNow,from,to)=>{const R=rotationFromTo(from,to);return{R,t:sub(anchorNow,mv(R,anchor0))}};
const rotationDistance=(A,B)=>Math.acos(clamp((mul(B,tr(A))[0][0]+mul(B,tr(A))[1][1]+mul(B,tr(A))[2][2]-1)/2,-1,1));
const Rz=a=>{const c=Math.cos(a),s=Math.sin(a);return[[c,-s,0],[s,c,0],[0,0,1]]},Rx=a=>{const c=Math.cos(a),s=Math.sin(a);return[[1,0,0],[0,c,-s],[0,s,c]]},euler313=angles=>mul(mul(Rz(angles[0]),Rx(angles[1])),Rz(angles[2]));
const JOINTS=Object.freeze([
  {id:'J0',name:'Shoulder Pitch',driverType:'revolute',driver:'MOTION_SHOULDER_L_pitch'},
  {id:'J1',name:'Shoulder Roll',driverType:'revolute',driver:'MOTION_SHOULDER_L_roll'},
  {id:'J2',name:'Shoulder Yaw',driverType:'revolute',driver:'MOTION_SHOULDER_L_yaw'},
  {id:'J3',name:'Elbow Pitch',driverType:'singleRod',driver:'MOTION_ELBOW_L_pitch_T'},
  {id:'J4',name:'Elbow Yaw',driverType:'revolute',driver:'MOTION_ELBOW_L_yaw'},
  {id:'J5',name:'Wrist Roll',driverType:'dualRod',driver:['MOTION_WRIST_L_rod1_T','MOTION_WRIST_L_rod2_T']},
  {id:'J6',name:'Wrist Pitch',driverType:'dualRod',driver:['MOTION_WRIST_L_rod1_T','MOTION_WRIST_L_rod2_T']},
]);
const ZERO_INPUTS=Object.freeze({shoulderPitchDeg:0,shoulderRollDeg:0,shoulderYawDeg:0,elbowRodMm:0,elbowYawDeg:0,wristLeftMm:0,wristRightMm:0});

export function createArmRuntime(model,poseReference={}){
  const blackbox=createLeftArmBlackbox(model),baseValidation=blackbox.validate();
  if(!baseValidation.ok)throw new Error('左臂黑盒校验失败');
  const dynamics=blackbox.armDynamicsSummary();
  const bodyNames=Object.keys(model.parts).filter(name=>model.parts[name].role==='arm');
  const jointNames=['J_ARM_L_01_RE','J_ARM_L_02_RE','J_ARM_L_03_RE','J_ARM_L_04_RE','J_ARM_L_05_RE','J_ARM_L_06_RE','J_ARM_L_07_RE'];
  const mainBodies=['arm_l_link1','arm_l_link2','arm_l_link3','arm_l_link4','arm_l_link5','arm_l_link6','arm_l_link7'];
  const rateKeys=Object.keys(model.joints).filter(name=>model.joints[name].type!=='fixed'&&(bodyNames.includes(model.joints[name].body_i)||bodyNames.includes(model.joints[name].body_j)));
  const bodyData=Object.fromEntries(bodyNames.map(name=>[name,blackbox.rigidBodyMassProperties(name)]));

  function solveActive(inputs={}){
    const activeInputs={...ZERO_INPUTS,...inputs};
    if(!Object.values(activeInputs).every(finite))return{complete:false,error:'主动输入必须为有限数值'};
    try{
      const elbow=blackbox.elbowRodToPitch({rodStrokeMm:Number(activeInputs.elbowRodMm)});
      const wrist=blackbox.wristRodsToPose({leftStrokeMm:Number(activeInputs.wristLeftMm),rightStrokeMm:Number(activeInputs.wristRightMm)});
      const outputs={J0:Number(activeInputs.shoulderPitchDeg),J1:Number(activeInputs.shoulderRollDeg),J2:Number(activeInputs.shoulderYawDeg),J3:elbow.deltaPitchDeg,J4:Number(activeInputs.elbowYawDeg),J5:wrist.rollDeg,J6:wrist.pitchDeg};
      return{complete:true,activeInputs,outputs,residualMm:Math.max(elbow.residualMm,wrist.residualMm),iterations:Math.max(elbow.iterations,wrist.iterations),elbow,wrist};
    }catch(error){return{complete:false,error:error instanceof Error?error.message:String(error),activeInputs}}
  }

  function solveOutput(joint,degrees){
    if(!JOINTS.some(item=>item.id===joint))throw new Error(`未知左臂关节 ${joint}`);
    if(!finite(degrees))return{complete:false,error:'输出角必须为有限数值'};
    const target=Number(degrees),inputs={...ZERO_INPUTS};
    if(joint==='J0')inputs.shoulderPitchDeg=target;
    else if(joint==='J1')inputs.shoulderRollDeg=target;
    else if(joint==='J2')inputs.shoulderYawDeg=target;
    else if(joint==='J3')inputs.elbowRodMm=blackbox.elbowPitchToRod({deltaPitchDeg:target}).rodStrokeMm;
    else if(joint==='J4')inputs.elbowYawDeg=target;
    else {
      const rods=blackbox.wristPoseToRods({rollDeg:joint==='J5'?target:0,pitchDeg:joint==='J6'?target:0});
      inputs.wristLeftMm=rods.leftStrokeMm;inputs.wristRightMm=rods.rightStrokeMm;
    }
    const solved=solveActive(inputs);
    return solved.complete?{...solved,targetJoint:joint,targetDegrees:target}:{...solved,targetJoint:joint,targetDegrees:target};
  }

  function validate(){
    const massPropertiesComplete=bodyNames.filter(name=>blackbox.rigidBodyMassProperties(name)).length;
    return{...baseValidation,ok:baseValidation.ok&&bodyNames.length===18&&massPropertiesComplete===18,bodyCount:bodyNames.length,massPropertiesComplete,totalMassKg:dynamics.totalArmMassKg,gravityMps2:dynamics.gravityMps2,excludedInterfaces:[...dynamics.excludedInterfaces]};
  }

  function kinematics(solution){
    if(!solution?.complete)return{complete:false,jointCentersMm:{},bodyTransforms:{}};
    let chain=identityTransform();const bodyTransforms={},jointCentersMm={},jointAxes={};
    for(let index=0;index<jointNames.length;index++){
      const definition=model.joints[jointNames[index]],id=`J${index}`,pivot=apply(chain,definition.xyz_mm),axis=mv(chain.R,definition.axis_i_global_initial);
      jointCentersMm[id]=pivot;jointAxes[id]=axis;
      chain=compose(about(pivot,axis,rad(solution.outputs[id])),chain);
      bodyTransforms[mainBodies[index]]=chain;
      if(index===2)bodyTransforms.__elbowUpstream=chain;
      if(index===4)bodyTransforms.__wristUpstream=chain;
    }

    const elbow0=blackbox.elbowGeometry(),elbow=blackbox.elbowGeometry({deltaPitchDeg:solution.outputs.J3}),elbowUp=bodyTransforms.__elbowUpstream;
    const link11Local=rigidBetween(elbow0.fixedPivot,elbow.fixedPivot,sub(elbow0.coupler,elbow0.fixedPivot),sub(elbow.coupler,elbow.fixedPivot));
    const link12Local=rigidBetween(elbow0.movingPivot,elbow.movingPivot,sub(elbow0.coupler,elbow0.movingPivot),sub(elbow.coupler,elbow.movingPivot));
    const rodLocal=rigidBetween(elbow0.rodUpper,elbow.rodUpper,sub(elbow0.coupler,elbow0.rodUpper),sub(elbow.coupler,elbow.rodUpper));
    const rodLowerLocal=rigidBetween(elbow0.coupler,elbow.coupler,sub(elbow0.coupler,elbow0.rodUpper),sub(elbow.coupler,elbow.rodUpper));
    bodyTransforms.arm_l_link8=compose(elbowUp,rodLocal);
    bodyTransforms.arm_l_link9=compose(elbowUp,rodLowerLocal);
    bodyTransforms.arm_l_ball_link10=compose(elbowUp,link11Local);
    bodyTransforms.arm_l_link11=compose(elbowUp,link11Local);
    bodyTransforms.arm_l_link12=compose(elbowUp,link12Local);

    const wrist0=blackbox.wristGeometry(),wrist=blackbox.wristGeometry({rollDeg:solution.outputs.J5,pitchDeg:solution.outputs.J6}),wristUp=bodyTransforms.__wristUpstream;
    const leftR=rotationFromTo(sub(wrist0.lowerLeft,wrist0.upperLeft),sub(wrist.lowerLeft,wrist.upperLeft)),rightR=rotationFromTo(sub(wrist0.lowerRight,wrist0.upperRight),sub(wrist.lowerRight,wrist.upperRight));
    const leftUpper={R:leftR,t:sub(wrist.upperLeft,mv(leftR,wrist0.upperLeft))},leftLower={R:leftR,t:sub(wrist.lowerLeft,mv(leftR,wrist0.lowerLeft))};
    const rightUpper={R:rightR,t:sub(wrist.upperRight,mv(rightR,wrist0.upperRight))},rightLower={R:rightR,t:sub(wrist.lowerRight,mv(rightR,wrist0.lowerRight))};
    bodyTransforms.arm_l_link13=compose(wristUp,leftUpper);bodyTransforms.arm_l_link14=compose(wristUp,leftLower);bodyTransforms.arm_l_ball_link15=bodyTransforms.arm_l_link7;
    bodyTransforms.arm_l_link16=compose(wristUp,rightUpper);bodyTransforms.arm_l_link17=compose(wristUp,rightLower);bodyTransforms.arm_l_ball_link18=bodyTransforms.arm_l_link7;
    delete bodyTransforms.__elbowUpstream;delete bodyTransforms.__wristUpstream;
    const transformPoints=points=>Object.fromEntries(Object.entries(points).filter(([,value])=>Array.isArray(value)).map(([key,value])=>[key,apply(key.startsWith('axis')||key.startsWith('upper')||key.startsWith('lower')?wristUp:elbowUp,value)]));
    return{complete:true,jointCentersMm,jointAxes,bodyTransforms,elbowPoints:transformPoints(elbow),wristPoints:transformPoints(wrist)};
  }

  function bodyPoses(solution){
    const geometry=kinematics(solution);if(!geometry.complete)return{complete:false,bodies:{},geometry};const bodies={};
    for(const name of bodyNames){const data=bodyData[name],T=geometry.bodyTransforms[name]??identityTransform();bodies[name]={name,massKg:data.massKg,comMm:apply(T,data.comGlobalInitialMm),rotation:T.R,inertia:data.inertiaRawKgM2,inertiaRotation:mul(T.R,euler313(data.inertiaMarkerEuler313Rad)),role:model.parts[name].role}}
    return{complete:true,bodies,geometry};
  }

  const outputOf=(joint,solution)=>solution.outputs[joint];
  const relativeRotation=(poses,bodyI,bodyJ)=>mul(tr(poses[bodyI]?.rotation??I()),poses[bodyJ]?.rotation??I());
  function rates(joint,degrees){
    const h=.001,center=solveOutput(joint,degrees),lo=solveOutput(joint,degrees-h),hi=solveOutput(joint,degrees+h);if(!center.complete||!lo.complete||!hi.complete)return{complete:false,solution:center,values:{}};
    const den=2*rad(h),pl=bodyPoses(lo).bodies,ph=bodyPoses(hi).bodies,values={};
    for(const key of rateKeys){
      if(key==='J_ARM_L_09_T')values[key]=(hi.activeInputs.elbowRodMm-lo.activeInputs.elbowRodMm)/1000/den;
      else if(key==='J_ARM_L_14_T')values[key]=(hi.activeInputs.wristLeftMm-lo.activeInputs.wristLeftMm)/1000/den;
      else if(key==='J_ARM_L_17_T')values[key]=(hi.activeInputs.wristRightMm-lo.activeInputs.wristRightMm)/1000/den;
      else{const definition=model.joints[key],rl=relativeRotation(pl,definition.body_i,definition.body_j),rh=relativeRotation(ph,definition.body_i,definition.body_j);values[key]=rotationDistance(rl,rh)/den}
    }
    return{complete:Object.values(values).every(Number.isFinite),values,solution:center};
  }

  const directionKey=direction=>direction>=0?'positive':'negative',measured=(loads,key,mode,direction)=>Math.max(0,Number(loads?.[key]?.[mode]?.[directionKey(direction)])||0);
  function equivalentResistance(joint,solution,loads={},direction=1,mode='static'){
    const rate=rates(joint,outputOf(joint,solution));if(!solution?.complete||!rate.complete)return{complete:false,total:0,parts:{},contribution:{},rates:{}};
    const parts={},directions={};for(const key of rateKeys){directions[key]=(Math.sign(rate.values[key])||1)*(direction>=0?1:-1);parts[key]=measured(loads,key,mode,directions[key])*Math.abs(rate.values[key])}
    const total=Object.values(parts).reduce((a,b)=>a+b,0),contribution=Object.fromEntries(rateKeys.map(key=>[key,total?100*parts[key]/total:0]));
    return{complete:true,total,parts,contribution,rates:rate.values,directions,mode,direction};
  }

  const potentialEnergy=solution=>Object.values(bodyPoses(solution).bodies).reduce((sum,body)=>sum+body.massKg*Math.abs(dynamics.gravityMps2[2])*body.comMm[2]/1000,0);
  function gravityHoldingTorque(joint,degrees){const h=.001,lo=solveOutput(joint,degrees-h),hi=solveOutput(joint,degrees+h);if(!lo.complete||!hi.complete)return{complete:false,torqueNm:NaN};return{complete:true,torqueNm:(potentialEnergy(hi)-potentialEnergy(lo))/(2*rad(h))}}
  const inertiaMatrix=raw=>[[raw.ixx,raw.ixy,raw.izx],[raw.ixy,raw.iyy,raw.iyz],[raw.izx,raw.iyz,raw.izz]];
  function effectiveInertia(joint,degrees){
    const h=.001,center=solveOutput(joint,degrees),lo=solveOutput(joint,degrees-h),hi=solveOutput(joint,degrees+h);if(!center.complete||!lo.complete||!hi.complete)return{complete:false,kgM2:NaN};
    const pc=bodyPoses(center).bodies,pl=bodyPoses(lo).bodies,ph=bodyPoses(hi).bodies,den=2*rad(h);let kgM2=0;
    for(const name of bodyNames){const b=pc[name],v=scale(sub(ph[name].comMm,pl[name].comMm),1/1000/den),Rdot=ph[name].inertiaRotation.map((row,i)=>row.map((value,j)=>(value-pl[name].inertiaRotation[i][j])/den)),W=mul(Rdot,tr(b.inertiaRotation)),omega=[(W[2][1]-W[1][2])/2,(W[0][2]-W[2][0])/2,(W[1][0]-W[0][1])/2],Iw=mul(mul(b.inertiaRotation,inertiaMatrix(b.inertia)),tr(b.inertiaRotation));kgM2+=b.massKg*dot(v,v)+dot(omega,mv(Iw,omega))}
    return{complete:Number.isFinite(kgM2)&&kgM2>0,kgM2};
  }
  function trajectoryDynamics(joint,degrees,options={}){const amplitude=rad(Number(options.amplitudeDegrees)||10),omega=2*Math.PI*(Number(options.frequencyHz)||1),q=rad(degrees),velocity=(options.direction??1)*amplitude*omega*Math.sqrt(Math.max(0,1-Math.min(1,(q/amplitude)**2))),acceleration=-omega*omega*q,J=effectiveInertia(joint,degrees),G=gravityHoldingTorque(joint,degrees);if(!J.complete||!G.complete)return{complete:false};const inertiaTorqueNm=J.kgM2*acceleration;return{complete:true,velocityRadS:velocity,accelerationRadS2:acceleration,effectiveInertiaKgM2:J.kgM2,gravityTorqueNm:G.torqueNm,inertiaTorqueNm,totalTorqueNm:G.torqueNm+inertiaTorqueNm}}
  function cycle(joint,loads={},options={}){const count=Math.max(5,Math.floor(Number(options.samples)||121)),samples=[];for(let index=0;index<count;index++){const time=index/(count-1),phase=2*Math.PI*time,output=10*Math.sin(phase),direction=Math.cos(phase)>=0?1:-1,solution=solveOutput(joint,output),staticResult=equivalentResistance(joint,solution,loads,direction,'static'),dynamicResult=equivalentResistance(joint,solution,loads,direction,'dynamic'),dynamicsResult=trajectoryDynamics(joint,output,{direction});if(!solution.complete||!staticResult.complete||!dynamicResult.complete||!dynamicsResult.complete)return{complete:false,failingIndex:index,samples};samples.push({time,output,direction,solution,static:staticResult,dynamic:dynamicResult,dynamics:dynamicsResult})}const ranking=mode=>{const totals=Object.fromEntries(rateKeys.map(key=>[key,0])),dt=1/(count-1);for(let i=1;i<count;i++)for(const key of rateKeys)totals[key]+=.5*(samples[i-1][mode].parts[key]+samples[i][mode].parts[key])*dt;const denominator=Object.values(totals).reduce((a,b)=>a+b,0);return rateKeys.map(key=>({key,integral:totals[key],contribution:denominator?100*totals[key]/denominator:0})).sort((a,b)=>b.contribution-a.contribution)};return{complete:true,samples,rankings:{static:ranking('static'),dynamic:ranking('dynamic')}}}

  return Object.freeze({
    validate,
    summary:()=>({bodyCount:bodyNames.length,totalMassKg:dynamics.totalArmMassKg,gravityMps2:[...dynamics.gravityMps2],initialAggregateCOMMm:[...dynamics.initialAggregateCOMMm],excludedInterfaces:[...dynamics.excludedInterfaces],joints:JOINTS.map(item=>({...item})),revisionConflicts:blackbox.versionConflicts()}),
    solveActive,solveOutput,
    bodyNames:[...bodyNames],rateKeys:[...rateKeys],
    body:name=>blackbox.rigidBodyMassProperties(name),
    joint:name=>blackbox.joint(name),
    hardpoints:()=>blackbox.hardpoints(),
    poseReference,kinematics,bodyPoses,rates,equivalentResistance,gravityHoldingTorque,effectiveInertia,trajectoryDynamics,cycle,
    blackbox,
  });
}
