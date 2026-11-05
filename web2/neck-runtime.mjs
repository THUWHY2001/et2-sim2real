import {makeNeck} from './neck-blackbox.mjs';

const finite=value=>Number.isFinite(Number(value));
const add=(a,b)=>a.map((v,i)=>v+b[i]),sub=(a,b)=>a.map((v,i)=>v-b[i]),scale=(a,s)=>a.map(v=>v*s),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),norm=a=>Math.hypot(...a),unit=a=>{const n=norm(a);return n?scale(a,1/n):[0,0,0]},cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const I=()=>[[1,0,0],[0,1,0],[0,0,1]],mul=(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((s,v,k)=>s+v*b[k][j],0))),mv=(a,v)=>a.map(row=>dot(row,v)),tr=a=>a[0].map((_,i)=>a.map(row=>row[i]));
const Rx=a=>{const c=Math.cos(a),s=Math.sin(a);return[[1,0,0],[0,c,-s],[0,s,c]]},Ry=a=>{const c=Math.cos(a),s=Math.sin(a);return[[c,0,s],[0,1,0],[-s,0,c]]},Rz=a=>{const c=Math.cos(a),s=Math.sin(a);return[[c,-s,0],[s,c,0],[0,0,1]]},rad=d=>d*Math.PI/180;
const axisAngle=(axis,angle)=>{const [x,y,z]=unit(axis),c=Math.cos(angle),s=Math.sin(angle),t=1-c;return[[t*x*x+c,t*x*y-s*z,t*x*z+s*y],[t*x*y+s*z,t*y*y+c,t*y*z-s*x],[t*x*z-s*y,t*y*z+s*x,t*z*z+c]]};
const rotationFromTo=(a,b)=>{const u=unit(a),v=unit(b),c=Math.max(-1,Math.min(1,dot(u,v)));if(c>1-1e-12)return I();if(c<-1+1e-12)return axisAngle(cross(u,Math.abs(u[0])<.8?[1,0,0]:[0,1,0]),Math.PI);const w=cross(u,v);return axisAngle(w,Math.atan2(norm(w),c))};

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

  const point=name=>neck.point(name).xyz_world0_mm;
  const initialPoints={roll:point('Roll旋转中心'),pitch:point('Pitch旋转中心'),yaw:point('Yaw旋转中心'),leftLower:point('左下球铰中心'),leftUpper:point('左上球铰中心'),rightLower:point('右下球铰中心'),rightUpper:point('右上球铰中心')};
  const bodyNames=Object.keys(neck.bodies),bodyData=Object.fromEntries(bodyNames.map(name=>[name,neck.body(name)]));
  const headBodies=new Set(['neck_link3','head_imu_link1','head_camera_link1','head_camera_link2','head_fisheye_camera_link3']);
  const rateKeys=['yaw','rollCenter','rollRear','rollFront','pitchCenter','pitchL','pitchR','leftLowerBall','leftUpperBall','rightLowerBall','rightUpperBall','leftPrismatic','rightPrismatic'];
  const transformAbout=(p,pivot,R)=>add(pivot,mv(R,sub(p,pivot)));
  function kinematics(solution){
    if(!solution?.complete)return{complete:false};
    const Rroll=Rx(rad(solution.rollDeg)),Rplatform=mul(Rroll,Ry(rad(solution.pitchDeg))),Rhead=mul(Rplatform,Rz(rad(solution.yawDeg)));
    const rollPoint=p=>transformAbout(p,initialPoints.roll,Rroll);
    const pitchPivot=rollPoint(initialPoints.pitch),platformPoint=p=>add(pitchPivot,mv(Rplatform,sub(p,initialPoints.pitch)));
    const yawPivot=platformPoint(initialPoints.yaw),headPoint=p=>add(yawPivot,mv(Rhead,sub(p,initialPoints.yaw)));
    const points={roll:initialPoints.roll,pitch:pitchPivot,yaw:yawPivot,leftLower:initialPoints.leftLower,rightLower:initialPoints.rightLower,leftUpper:platformPoint(initialPoints.leftUpper),rightUpper:platformPoint(initialPoints.rightUpper)};
    const rodL=rotationFromTo(sub(initialPoints.leftUpper,initialPoints.leftLower),sub(points.leftUpper,points.leftLower)),rodR=rotationFromTo(sub(initialPoints.rightUpper,initialPoints.rightLower),sub(points.rightUpper,points.rightLower));
    return{complete:true,points,rotations:{roll:Rroll,platform:Rplatform,head:Rhead,rodL,rodR},pointFns:{rollPoint,platformPoint,headPoint},rodLengthsMm:{left:norm(sub(points.leftUpper,points.leftLower)),right:norm(sub(points.rightUpper,points.rightLower))}};
  }
  function bodyPoses(solution){
    const k=kinematics(solution);if(!k.complete)return{complete:false,bodies:{}};const bodies={};
    for(const name of bodyNames){const b=bodyData[name],p=b.com_world0_mm;let comMm,rotation;
      if(name==='neck_link1'){comMm=k.pointFns.rollPoint(p);rotation=k.rotations.roll}
      else if(name==='neck_link2'||name==='neck_ball_link6'||name==='neck_ball_link9'){comMm=k.pointFns.platformPoint(p);rotation=k.rotations.platform}
      else if(headBodies.has(name)){comMm=k.pointFns.headPoint(p);rotation=k.rotations.head}
      else if(name==='neck_link4'){comMm=add(k.points.leftLower,mv(k.rotations.rodL,sub(p,initialPoints.leftLower)));rotation=k.rotations.rodL}
      else if(name==='neck_link5'){comMm=add(k.points.leftUpper,mv(k.rotations.rodL,sub(p,initialPoints.leftUpper)));rotation=k.rotations.rodL}
      else if(name==='neck_link7'){comMm=add(k.points.rightLower,mv(k.rotations.rodR,sub(p,initialPoints.rightLower)));rotation=k.rotations.rodR}
      else if(name==='neck_link8'){comMm=add(k.points.rightUpper,mv(k.rotations.rodR,sub(p,initialPoints.rightUpper)));rotation=k.rotations.rodR}
      else{comMm=[...p];rotation=I()}
      bodies[name]={name,massKg:b.mass_kg,comMm,rotation,inertia:b.inertia_cm_raw_kg_m2,role:b.role};
    }
    return{complete:true,bodies,geometry:k};
  }
  const outputOf=(joint,s)=>joint==='J0'?s.rollDeg:joint==='J1'?s.pitchDeg:s.yawDeg;
  function directions(solution){const k=kinematics(solution),l=unit(sub(k.points.leftUpper,k.points.leftLower)),r=unit(sub(k.points.rightUpper,k.points.rightLower)),Rt=tr(k.rotations.platform);return{leftLowerBall:l,rightLowerBall:r,leftUpperBall:mv(Rt,scale(l,-1)),rightUpperBall:mv(Rt,scale(r,-1))}}
  function rates(joint,degrees){const center=solveOutput(joint,degrees);if(!center.complete)return{complete:false,solution:center};const h=.001,lo=solveOutput(joint,degrees-h),hi=solveOutput(joint,degrees+h);if(!lo.complete||!hi.complete)return{complete:false,solution:center};const den=2*rad(h),values={yaw:rad(hi.yawDeg-lo.yawDeg)/den,rollCenter:rad(hi.rollDeg-lo.rollDeg)/den,rollRear:rad(hi.rollDeg-lo.rollDeg)/den,rollFront:rad(hi.rollDeg-lo.rollDeg)/den,pitchCenter:rad(hi.pitchDeg-lo.pitchDeg)/den,pitchL:rad(hi.pitchDeg-lo.pitchDeg)/den,pitchR:rad(hi.pitchDeg-lo.pitchDeg)/den,leftPrismatic:(hi.leftMm-lo.leftMm)/1000/den,rightPrismatic:(hi.rightMm-lo.rightMm)/1000/den};const dl=directions(lo),dh=directions(hi);for(const key of ['leftLowerBall','leftUpperBall','rightLowerBall','rightUpperBall'])values[key]=norm(sub(dh[key],dl[key]))/den;return{complete:Object.values(values).every(Number.isFinite),values,solution:center}}
  const directionKey=d=>d>=0?'positive':'negative',measured=(load,mode,d)=>Math.max(0,Number(load?.[mode]?.[directionKey(d)])||0);
  function equivalentResistance(joint,solution,loads={},direction=1,mode='static'){
    const rate=rates(joint,outputOf(joint,solution));if(!solution?.complete||!rate.complete)return{complete:false,parts:{},contribution:{},total:0};const parts={},directions={};for(const key of rateKeys){directions[key]=Math.sign(rate.values[key])*(direction>=0?1:-1)||direction;parts[key]=measured(loads[key],mode,directions[key])*Math.abs(rate.values[key])}const total=Object.values(parts).reduce((a,b)=>a+b,0),contribution={};for(const key of rateKeys)contribution[key]=total?100*parts[key]/total:0;return{complete:true,parts,contribution,total,rates:rate.values,directions,mode,direction}
  }
  const knownWeightN=validation.mass_kg*Math.abs(neck.gravity[2]);
  function nominalLoads(solution,assumptions={}){const k=kinematics(solution),pre=key=>Math.max(0,Number(assumptions.preloads?.[key])||0),upperMass=['neck_link2','neck_link3','neck_ball_link6','neck_ball_link9','head_imu_link1','head_camera_link1','head_camera_link2','head_fisheye_camera_link3'].reduce((s,n)=>s+bodyData[n].mass_kg,0),axial=(upper,lower)=>.5*upperMass*Math.abs(neck.gravity[2])/Math.max(.15,Math.abs(unit(sub(upper,lower))[2])),left=axial(k.points.leftUpper,k.points.leftLower),right=axial(k.points.rightUpper,k.points.rightLower),loadsN={yaw:(bodyData.neck_link3.mass_kg+bodyData.head_imu_link1.mass_kg+bodyData.head_camera_link1.mass_kg+bodyData.head_camera_link2.mass_kg+bodyData.head_fisheye_camera_link3.mass_kg)*Math.abs(neck.gravity[2])+pre('yaw')};for(const key of ['rollCenter','rollRear','rollFront'])loadsN[key]=knownWeightN/3+pre(key);for(const key of ['pitchCenter','pitchL','pitchR'])loadsN[key]=upperMass*Math.abs(neck.gravity[2])/3+pre(key);for(const key of ['leftLowerBall','leftUpperBall','leftPrismatic'])loadsN[key]=left+pre(key);for(const key of ['rightLowerBall','rightUpperBall','rightPrismatic'])loadsN[key]=right+pre(key);return{complete:true,loadsN,knownWeightN}}
  function loadedResistance(joint,solution,loads={},direction=1,mode='static',assumptions={}){const base=equivalentResistance(joint,solution,loads,direction,mode),bearing=nominalLoads(solution,assumptions);if(!base.complete)return{complete:false,items:{}};const loadedParts={},coef=key=>Math.max(0,Number(assumptions.coefficients?.[key])||0);for(const key of rateKeys)loadedParts[key]=(measured(loads[key],mode,base.directions[key])+coef(key)*bearing.loadsN[key])*Math.abs(base.rates[key]);const loadedTotal=Object.values(loadedParts).reduce((a,b)=>a+b,0),items={};for(const key of rateKeys){const loadedContribution=loadedTotal?100*loadedParts[key]/loadedTotal:0,baseContribution=base.total?100*base.parts[key]/base.total:0;items[key]={key,basePart:base.parts[key],loadedPart:loadedParts[key],deltaNm:loadedParts[key]-base.parts[key],baseContribution,loadedContribution,deltaPercentagePoints:loadedContribution-baseContribution,sensitivityNmPerN:bearing.knownWeightN?(loadedParts[key]-base.parts[key])/bearing.knownWeightN:0,nominalLoadN:bearing.loadsN[key]}}return{complete:true,items,baseTotal:base.total,loadedTotal,deltaTotalNm:loadedTotal-base.total,bearing,mode,direction}}
  const energy=solution=>Object.values(bodyPoses(solution).bodies).reduce((sum,b)=>sum+b.massKg*Math.abs(neck.gravity[2])*b.comMm[2]/1000,0);
  function gravityHoldingTorque(joint,degrees){const h=.001,lo=solveOutput(joint,degrees-h),hi=solveOutput(joint,degrees+h);if(!lo.complete||!hi.complete)return{complete:false,torqueNm:NaN};return{complete:true,torqueNm:(energy(hi)-energy(lo))/(2*rad(h))}}
  const inertiaMatrix=b=>{const [xx,yy,zz,xy,zx,yz]=b.inertia;return[[xx,xy,zx],[xy,yy,yz],[zx,yz,zz]]};
  function effectiveInertia(joint,degrees){const h=.001,c=solveOutput(joint,degrees),lo=solveOutput(joint,degrees-h),hi=solveOutput(joint,degrees+h);if(!c.complete||!lo.complete||!hi.complete)return{complete:false,kgM2:NaN};const pc=bodyPoses(c).bodies,pl=bodyPoses(lo).bodies,ph=bodyPoses(hi).bodies,den=2*rad(h);let kgM2=0;for(const name of bodyNames){const b=pc[name],v=scale(sub(ph[name].comMm,pl[name].comMm),1/1000/den),Rdot=ph[name].rotation.map((row,i)=>row.map((x,j)=>(x-pl[name].rotation[i][j])/den)),W=mul(Rdot,tr(b.rotation)),omega=[(W[2][1]-W[1][2])/2,(W[0][2]-W[2][0])/2,(W[1][0]-W[0][1])/2],Iw=mul(mul(b.rotation,inertiaMatrix(b)),tr(b.rotation));kgM2+=b.massKg*dot(v,v)+dot(omega,mv(Iw,omega))}return{complete:true,kgM2}}
  function trajectoryDynamics(joint,degrees,options={}){const amplitude=rad(Number(options.amplitudeDegrees)||10),omega=2*Math.PI*(Number(options.frequencyHz)||1),q=rad(degrees),velocity=(options.direction??1)*amplitude*omega*Math.sqrt(Math.max(0,1-Math.min(1,(q/amplitude)**2))),acceleration=-omega*omega*q,J=effectiveInertia(joint,degrees),G=gravityHoldingTorque(joint,degrees);if(!J.complete||!G.complete)return{complete:false};const inertiaTorqueNm=J.kgM2*acceleration;return{complete:true,velocityRadS:velocity,accelerationRadS2:acceleration,effectiveInertiaKgM2:J.kgM2,gravityTorqueNm:G.torqueNm,inertiaTorqueNm,totalTorqueNm:G.torqueNm+inertiaTorqueNm}}
  function cycle(joint,loads={},assumptions={},options={}){const count=Math.max(5,Math.floor(Number(options.samples)||121)),samples=[];for(let i=0;i<count;i++){const time=i/(count-1),phase=2*Math.PI*time,output=10*Math.sin(phase),direction=Math.cos(phase)>=0?1:-1,solution=solveOutput(joint,output),staticResult=loadedResistance(joint,solution,loads,direction,'static',assumptions),dynamicResult=loadedResistance(joint,solution,loads,direction,'dynamic',assumptions),dynamics=trajectoryDynamics(joint,output,{direction});if(!solution.complete||!staticResult.complete||!dynamicResult.complete||!dynamics.complete)return{complete:false,failingIndex:i,samples};samples.push({time,output,direction,solution,static:staticResult,dynamic:dynamicResult,dynamics})}const ranking=mode=>{const totals=Object.fromEntries(rateKeys.map(k=>[k,0])),base=Object.fromEntries(rateKeys.map(k=>[k,0])),dt=1/(count-1);for(let i=1;i<count;i++)for(const key of rateKeys){totals[key]+=.5*(samples[i-1][mode].items[key].loadedPart+samples[i][mode].items[key].loadedPart)*dt;base[key]+=.5*(samples[i-1][mode].items[key].basePart+samples[i][mode].items[key].basePart)*dt}const td=Object.values(totals).reduce((a,b)=>a+b,0),bd=Object.values(base).reduce((a,b)=>a+b,0);return rateKeys.map(key=>({key,loadedIntegral:totals[key],baseIntegral:base[key],loadedContribution:td?100*totals[key]/td:0,baseContribution:bd?100*base[key]/bd:0,deltaNm:totals[key]-base[key]})).sort((a,b)=>b.loadedContribution-a.loadedContribution)};return{complete:true,samples,rankings:{static:ranking('static'),dynamic:ranking('dynamic')}}}

  return{
    validate:()=>({...validation}),
    summary:()=>({...neck.summary(),active_motions:['MOTION_neck_L_rod_T','MOTION_neck_R_rod_T','MOTION_neck_yaw'],passive_outputs:['MOTION_neck_roll','MOTION_neck_pitch']}),
    solveInputs,solveOutput,
    body:name=>neck.body(name),joint:name=>neck.joint(name),point:name=>neck.point(name),
    bodies:neck.bodies,joints:neck.joints,motions:neck.motions,hardpoints:neck.hardpoints,externalAnchors:neck.externalAnchors,gravity:neck.gravity,
    rateKeys,kinematics,bodyPoses,rates,equivalentResistance,nominalLoads,loadedResistance,gravityHoldingTorque,effectiveInertia,trajectoryDynamics,cycle,
    blackbox:neck,
  };
}
