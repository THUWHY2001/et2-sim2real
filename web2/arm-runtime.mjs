import {createLeftArmBlackbox} from './left-arm-blackbox.mjs';

const finite=value=>Number.isFinite(Number(value));
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

  return Object.freeze({
    validate,
    summary:()=>({bodyCount:bodyNames.length,totalMassKg:dynamics.totalArmMassKg,gravityMps2:[...dynamics.gravityMps2],initialAggregateCOMMm:[...dynamics.initialAggregateCOMMm],excludedInterfaces:[...dynamics.excludedInterfaces],joints:JOINTS.map(item=>({...item})),revisionConflicts:blackbox.versionConflicts()}),
    solveActive,solveOutput,
    bodyNames:[...bodyNames],
    body:name=>blackbox.rigidBodyMassProperties(name),
    joint:name=>blackbox.joint(name),
    hardpoints:()=>blackbox.hardpoints(),
    poseReference,
    blackbox,
  });
}
