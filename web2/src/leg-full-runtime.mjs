import {createCompleteLeftLeg} from "../data/leg-full-package/leg_full.mjs";

const OUTPUT_KEYS={J2:"J2_hip_pitch_deg",J3:"J3_knee_pitch_deg",J4:"J4_ankle_pitch_deg"};
const ROD_KEYS={J2:"hip",J3:"knee",J4:"ankle"};
const GRAVITY=9.80665;
const rad=degrees=>degrees*Math.PI/180;
const dot=(a,b)=>a.reduce((sum,value,index)=>sum+value*b[index],0);
const sub=(a,b)=>a.map((value,index)=>value-b[index]);
const norm=value=>Math.hypot(...value);
const unit=value=>{const length=norm(value);return length?value.map(item=>item/length):[0,0,0]};
const transpose=matrix=>matrix[0].map((_,column)=>matrix.map(row=>row[column]));
const multiply=(a,b)=>a.map(row=>b[0].map((_,column)=>row.reduce((sum,value,index)=>sum+value*b[index][column],0)));
const matrixVector=(matrix,vector)=>matrix.map(row=>dot(row,vector));
const clamp=(value,min,max)=>Math.max(min,Math.min(max,value));
const rotationDistance=(a,b)=>{const relative=multiply(b,transpose(a));return Math.acos(clamp((relative[0][0]+relative[1][1]+relative[2][2]-1)/2,-1,1))};

export function createLegFullRuntime(model){
  const blackbox=createCompleteLeftLeg(model);
  const nominal={...blackbox.nominal};
  const rateKeys=Object.keys(model.joints).filter(key=>model.joints[key].type!=="fixed");
  const partNames=Object.keys(model.parts);
  const massOf=names=>names.reduce((sum,name)=>sum+(model.parts[name]?.mass_kg||0),0);
  const hipMass=massOf(partNames.filter(name=>name.startsWith("leg_l_")));
  const kneeMass=massOf(partNames.filter(name=>/^leg_l_(link([4-9]|1[3-9])|ball_link(15|20))$/.test(name)));
  const ankleMass=massOf(partNames.filter(name=>/^leg_l_(link([5-7]|1[89])|ball_link20)$/.test(name)));

  function normalize(frame){
    const outputs={
      J0:frame.controls.angles_cmd_relative_deg.J0,
      J1:frame.controls.angles_cmd_relative_deg.J1,
      J2:frame.passive_outputs.J2_hip_pitch_deg,
      J3:frame.passive_outputs.J3_knee_pitch_deg,
      J4:frame.passive_outputs.J4_ankle_pitch_deg,
      J5:frame.controls.angles_cmd_relative_deg.J5,
    };
    const motors=frame.controls.motor_inputs??{};
    return{
      complete:frame.validation.ok,
      frame,
      outputs,
      outputDelta:{J2:outputs.J2-nominal.J2,J3:outputs.J3-nominal.J3,J4:outputs.J4-nominal.J4},
      rodStrokes:{hip_mm:Number(motors.hip_rod_mm??frame.active_rod_inputs.hip.delta_vs_nominal_mm),knee_mm:Number(motors.knee_rod_mm??frame.active_rod_inputs.knee.delta_vs_nominal_mm),ankle_mm:Number(motors.ankle_rod_mm??frame.active_rod_inputs.ankle.delta_vs_nominal_mm)},
      residualMm:Math.max(...frame.joint_constraint_checks.map(item=>Number(item.constraint_residual_mm)||0)),
    };
  }

  function solveRods(inputs={}){return normalize(blackbox.solve(inputs))}

  function solveOutput(joint,deltaDegrees){
    if(!OUTPUT_KEYS[joint])throw new Error(`腿足完整闭链仅支持 J2/J3/J4 输出反解，收到 ${joint}`);
    const absolute=nominal[joint]+Number(deltaDegrees),angles={...nominal,[joint]:absolute};
    const direct=blackbox.directAngles(angles),rod=ROD_KEYS[joint],stroke=direct.active_rod_inputs[rod].delta_vs_nominal_mm;
    return solveRods({hip_mm:joint==="J2"?stroke:0,knee_mm:joint==="J3"?stroke:0,ankle_mm:joint==="J4"?stroke:0});
  }

  const measured=(loads,key,mode,direction)=>Math.max(0,Number(loads?.[key]?.[mode]?.[direction>=0?"positive":"negative"])||0);
  const jointCheck=(solution,key)=>solution.frame.joint_constraint_checks.find(item=>item.joint===key);
  const relativeRotation=(solution,key)=>{const definition=model.joints[key],transforms=solution.frame.world_transforms;return multiply(transpose(transforms[definition.i_part].R),transforms[definition.j_part].R)};

  function rates(joint,deltaDegrees){
    const h=.001,center=solveOutput(joint,deltaDegrees),lo=solveOutput(joint,deltaDegrees-h),hi=solveOutput(joint,deltaDegrees+h);
    if(!center.complete||!lo.complete||!hi.complete)return{complete:false,values:{},solution:center};
    const denominator=2*rad(h),values={};
    for(const key of rateKeys){
      if(model.joints[key].type==="translational")values[key]=(jointCheck(hi,key).axial_sliding_signed_mm-jointCheck(lo,key).axial_sliding_signed_mm)/1000/denominator;
      else values[key]=rotationDistance(relativeRotation(lo,key),relativeRotation(hi,key))/denominator;
    }
    return{complete:Object.values(values).every(Number.isFinite),values,solution:center};
  }

  function signedRates(joint,deltaDegrees){
    const h=.001,center=solveOutput(joint,deltaDegrees),lo=solveOutput(joint,deltaDegrees-h),hi=solveOutput(joint,deltaDegrees+h),denominator=2*rad(h),values={};
    if(!center.complete||!lo.complete||!hi.complete)return{complete:false,values,solution:center};
    const angularRate=part=>{const Rc=center.frame.world_transforms[part].R,Rl=lo.frame.world_transforms[part].R,Rh=hi.frame.world_transforms[part].R,Rdot=Rh.map((row,i)=>row.map((value,j)=>(value-Rl[i][j])/denominator)),W=multiply(Rdot,transpose(Rc));return[(W[2][1]-W[1][2])/2,(W[0][2]-W[2][0])/2,(W[1][0]-W[0][1])/2]};
    for(const key of rateKeys){
      const definition=model.joints[key];
      if(definition.type==="translational")values[key]=(jointCheck(hi,key).axial_sliding_signed_mm-jointCheck(lo,key).axial_sliding_signed_mm)/1000/denominator;
      else{
        const wi=angularRate(definition.i_part),wj=angularRate(definition.j_part),relative=sub(wj,wi),marker=center.frame.all_joint_marker_sides.find(item=>item.joint===key&&item.marker_side==="I"),axis=unit(marker.axis_xyz);
        values[key]=definition.type==="revolute"?dot(relative,axis):norm(relative);
      }
    }
    return{complete:Object.values(values).every(Number.isFinite),values,solution:center,definition:"positive actuator force on J body dotted with signed relative velocity"};
  }

  function equivalentResistance(joint,solution,loads={},direction=1,mode="static"){
    const delta=solution.outputDelta[joint],rate=rates(joint,delta);
    if(!solution?.complete||!rate.complete)return{complete:false,total:0,parts:{},contribution:{},rates:{},directions:{}};
    const parts={},directions={};
    for(const key of rateKeys){directions[key]=(Math.sign(rate.values[key])||1)*(direction>=0?1:-1);parts[key]=measured(loads,key,mode,directions[key])*Math.abs(rate.values[key])}
    const total=Object.values(parts).reduce((sum,value)=>sum+value,0),contribution=Object.fromEntries(rateKeys.map(key=>[key,total?100*parts[key]/total:0]));
    return{complete:true,total,parts,contribution,rates:rate.values,directions,mode,direction};
  }

  function nominalLoads(solution,assumptions={}){
    const preload=key=>Math.max(0,Number(assumptions.preloads?.[key])||0),frame=solution.frame;
    const axial=(rod,mass)=>{const item=frame.active_rod_inputs[rod],vertical=Math.abs(unit(sub(item.anchor2_xyz_mm,item.anchor1_xyz_mm))[2]);return mass*GRAVITY/Math.max(.15,vertical)};
    const groupLoad={hip:axial("hip",hipMass),knee:axial("knee",kneeMass),ankle:axial("ankle",ankleMass)},loadsN={};
    for(const key of rateKeys){
      let value=hipMass*GRAVITY;
      if(key.includes("TOE"))value=(model.parts.leg_l_link7?.mass_kg||0)*GRAVITY;
      else if(key.includes("ANKLE")||key.includes("P019")||key.includes("P021"))value=key.endsWith("_T")||key.includes("P019")||key.includes("P021")?groupLoad.ankle:ankleMass*GRAVITY;
      else if(key.includes("KNEE")||key.includes("P014")||key.includes("P016")||key.includes("P017")||key.includes("P018"))value=key.endsWith("_T")||key.includes("P014")||key.includes("P016")||key.includes("P017")||key.includes("P018")?groupLoad.knee:kneeMass*GRAVITY;
      else if(key.includes("P009")||key.includes("P010")||key.includes("P011")||key.includes("P013")||key.includes("HIP_pitch_T"))value=groupLoad.hip;
      loadsN[key]=value+preload(key);
    }
    return{complete:true,loadsN,knownWeightN:frame.mass.total_kg*GRAVITY,groupLoadN:groupLoad};
  }

  function loadedResistance(joint,solution,loads={},direction=1,mode="static",assumptions={}){
    const base=equivalentResistance(joint,solution,loads,direction,mode),bearing=nominalLoads(solution,assumptions);
    if(!base.complete)return{complete:false,items:{}};
    const loadedParts={},coefficient=key=>Math.max(0,Number(assumptions.coefficients?.[key])||0);
    for(const key of rateKeys)loadedParts[key]=(measured(loads,key,mode,base.directions[key])+coefficient(key)*bearing.loadsN[key])*Math.abs(base.rates[key]);
    const loadedTotal=Object.values(loadedParts).reduce((sum,value)=>sum+value,0),items={};
    for(const key of rateKeys){const loadedContribution=loadedTotal?100*loadedParts[key]/loadedTotal:0,baseContribution=base.total?100*base.parts[key]/base.total:0;items[key]={key,basePart:base.parts[key],loadedPart:loadedParts[key],deltaNm:loadedParts[key]-base.parts[key],baseContribution,loadedContribution,deltaPercentagePoints:loadedContribution-baseContribution,sensitivityNmPerN:bearing.knownWeightN?(loadedParts[key]-base.parts[key])/bearing.knownWeightN:0,nominalLoadN:bearing.loadsN[key]}}
    return{complete:true,items,baseTotal:base.total,loadedTotal,deltaTotalNm:loadedTotal-base.total,bearing,mode,direction};
  }

  const potentialEnergy=solution=>Object.entries(solution.frame.mass.COM_world_mm).reduce((sum,[name,com])=>sum+model.parts[name].mass_kg*GRAVITY*com[2]/1000,0);
  function gravityHoldingTorque(joint,deltaDegrees){const h=.001,lo=solveOutput(joint,deltaDegrees-h),hi=solveOutput(joint,deltaDegrees+h);return lo.complete&&hi.complete?{complete:true,torqueNm:(potentialEnergy(hi)-potentialEnergy(lo))/(2*rad(h))}:{complete:false,torqueNm:NaN}}
  const inertiaMatrix=raw=>[[raw.ixx,raw.ixy,raw.izx],[raw.ixy,raw.iyy,raw.iyz],[raw.izx,raw.iyz,raw.izz]];
  function effectiveInertia(joint,deltaDegrees){
    const h=.001,center=solveOutput(joint,deltaDegrees),lo=solveOutput(joint,deltaDegrees-h),hi=solveOutput(joint,deltaDegrees+h);if(!center.complete||!lo.complete||!hi.complete)return{complete:false,kgM2:NaN};
    const denominator=2*rad(h);let kgM2=0;
    for(const name of partNames){const mass=model.parts[name].mass_kg,pc=center.frame.mass.COM_world_mm[name],pl=lo.frame.mass.COM_world_mm[name],ph=hi.frame.mass.COM_world_mm[name],velocity=sub(ph,pl).map(value=>value/1000/denominator),Rc=center.frame.world_transforms[name].R,Rl=lo.frame.world_transforms[name].R,Rh=hi.frame.world_transforms[name].R,Rdot=Rh.map((row,i)=>row.map((value,j)=>(value-Rl[i][j])/denominator)),W=multiply(Rdot,transpose(Rc)),omega=[(W[2][1]-W[1][2])/2,(W[0][2]-W[2][0])/2,(W[1][0]-W[0][1])/2],Iw=multiply(multiply(Rc,inertiaMatrix(model.parts[name].inertia_raw_kg_m2)),transpose(Rc));kgM2+=mass*dot(velocity,velocity)+dot(omega,matrixVector(Iw,omega))}
    return{complete:Number.isFinite(kgM2)&&kgM2>0,kgM2};
  }
  function trajectoryDynamics(joint,deltaDegrees,options={}){const amplitude=rad(Number(options.amplitudeDegrees)||10),omega=2*Math.PI*(Number(options.frequencyHz)||1),q=rad(deltaDegrees),velocity=(options.direction??1)*amplitude*omega*Math.sqrt(Math.max(0,1-Math.min(1,(q/amplitude)**2))),acceleration=-omega*omega*q,inertia=effectiveInertia(joint,deltaDegrees),gravity=gravityHoldingTorque(joint,deltaDegrees);if(!inertia.complete||!gravity.complete)return{complete:false};const inertiaTorqueNm=inertia.kgM2*acceleration;return{complete:true,velocityRadS:velocity,accelerationRadS2:acceleration,effectiveInertiaKgM2:inertia.kgM2,gravityTorqueNm:gravity.torqueNm,inertiaTorqueNm,totalTorqueNm:gravity.torqueNm+inertiaTorqueNm}}

  function loadedCycle(joint,loads={},assumptions={},options={}){
    const count=Math.max(5,Math.floor(Number(options.samples)||121)),samples=[];
    for(let index=0;index<count;index++){const time=index/(count-1),phase=2*Math.PI*time,output=10*Math.sin(phase),direction=Math.cos(phase)>=0?1:-1,solution=solveOutput(joint,output),staticResult=loadedResistance(joint,solution,loads,direction,"static",assumptions),dynamicResult=loadedResistance(joint,solution,loads,direction,"dynamic",assumptions),dynamics=trajectoryDynamics(joint,output,{direction});if(!solution.complete||!staticResult.complete||!dynamicResult.complete||!dynamics.complete)return{complete:false,failingIndex:index,samples};samples.push({time,output,direction,solution,static:staticResult,dynamic:dynamicResult,dynamics})}
    const ranking=mode=>{const loaded=Object.fromEntries(rateKeys.map(key=>[key,0])),base=Object.fromEntries(rateKeys.map(key=>[key,0])),dt=1/(count-1);for(let index=1;index<count;index++)for(const key of rateKeys){loaded[key]+=.5*(samples[index-1][mode].items[key].loadedPart+samples[index][mode].items[key].loadedPart)*dt;base[key]+=.5*(samples[index-1][mode].items[key].basePart+samples[index][mode].items[key].basePart)*dt}const loadedDenominator=Object.values(loaded).reduce((sum,value)=>sum+value,0),baseDenominator=Object.values(base).reduce((sum,value)=>sum+value,0);return rateKeys.map(key=>({key,loadedIntegral:loaded[key],baseIntegral:base[key],loadedContribution:loadedDenominator?100*loaded[key]/loadedDenominator:0,baseContribution:baseDenominator?100*base[key]/baseDenominator:0,deltaNm:loaded[key]-base[key]})).sort((a,b)=>b.loadedContribution-a.loadedContribution)};
    return{complete:true,samples,rankings:{static:ranking("static"),dynamic:ranking("dynamic")}};
  }

  return Object.freeze({
    nominal:{...nominal},
    validate:()=>blackbox.validate(),
    solveRods,
    solveOutput,
    kinematics:solution=>solution?.frame??solution,
    rateKeys:[...rateKeys],
    rates,
    signedRates,
    equivalentResistance,
    nominalLoads,
    loadedResistance,
    gravityHoldingTorque,
    effectiveInertia,
    trajectoryDynamics,
    loadedCycle,
    blackbox,
  });
}
