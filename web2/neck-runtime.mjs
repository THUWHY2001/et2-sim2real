import {makeNeck} from './neck-blackbox.mjs';

const finite=value=>Number.isFinite(Number(value));

export function createNeckRuntime(data){
  const neck=makeNeck(data);
  const validation=neck.validate();
  if(!validation.pass)throw new Error('头颈黑盒校验失败');

  function solveInputs({leftMm=0,rightMm=0,yawDeg=0}={}){
    if(![leftMm,rightMm,yawDeg].every(finite))return{complete:false,error:'输入必须为有限数值'};
    try{
      const pose=neck.rodsToPose(Number(leftMm),Number(rightMm));
      return{complete:true,leftMm:Number(leftMm),rightMm:Number(rightMm),yawDeg:Number(yawDeg),rollDeg:pose.roll_deg,pitchDeg:pose.pitch_deg,residualMm:pose.residual_max_mm,iterations:pose.iterations,branch:pose.branch,pose};
    }catch(error){return{complete:false,error:error instanceof Error?error.message:String(error)}}
  }

  function solveOutput(joint,degrees,yawDeg=0){
    if(!finite(degrees)||!finite(yawDeg))return{complete:false,error:'输出角必须为有限数值'};
    if(joint==='J2')return solveInputs({leftMm:0,rightMm:0,yawDeg:Number(degrees)});
    if(joint!=='J0'&&joint!=='J1')throw new Error(`未知头颈关节 ${joint}`);
    try{
      const target={rollDeg:joint==='J0'?Number(degrees):0,pitchDeg:joint==='J1'?Number(degrees):0};
      const rods=neck.poseToRods(target.rollDeg,target.pitchDeg),roundtrip=solveInputs({leftMm:rods.extension_mm.L,rightMm:rods.extension_mm.R,yawDeg:Number(yawDeg)});
      if(!roundtrip.complete)return roundtrip;
      return{...roundtrip,targetRollDeg:target.rollDeg,targetPitchDeg:target.pitchDeg,ik:rods};
    }catch(error){return{complete:false,error:error instanceof Error?error.message:String(error)}}
  }

  return{
    validate:()=>({...validation}),
    summary:()=>({...neck.summary(),active_motions:['MOTION_neck_L_rod_T','MOTION_neck_R_rod_T','MOTION_neck_yaw'],passive_outputs:['MOTION_neck_roll','MOTION_neck_pitch']}),
    solveInputs,solveOutput,
    body:name=>neck.body(name),joint:name=>neck.joint(name),point:name=>neck.point(name),
    bodies:neck.bodies,joints:neck.joints,motions:neck.motions,hardpoints:neck.hardpoints,externalAnchors:neck.externalAnchors,gravity:neck.gravity,
    blackbox:neck,
  };
}
