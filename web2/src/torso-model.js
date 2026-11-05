(() => {
  'use strict';

  const massProperties=window.TorsoDynamicsData;
  if(!massProperties)throw new Error('TorsoDynamicsData must load before TorsoModel');

  const points={
    yawPivot:{x:-29.2574,y:.0001,z:-24.0598},
    outputPivot:{x:-29.255,y:0,z:30.8402},
    crankPivotL:{x:-29.255,y:22.6,z:97.8402},
    crankPivotR:{x:-29.255,y:-22.6,z:97.8402},
    upperBallL:{x:-68.4219,y:13.35,z:109.0301},
    upperBallR:{x:-68.4219,y:-13.35,z:109.0301},
    lowerBallL:{x:-73.0189,y:25.7497,z:40.2936},
    lowerBallR:{x:-73.0189,y:-25.7497,z:40.2936},
  };

  const add=(a,b)=>({x:a.x+b.x,y:a.y+b.y,z:a.z+b.z});
  const sub=(a,b)=>({x:a.x-b.x,y:a.y-b.y,z:a.z-b.z});
  const scale=(a,s)=>({x:a.x*s,y:a.y*s,z:a.z*s});
  const dot=(a,b)=>a.x*b.x+a.y*b.y+a.z*b.z;
  const cross=(a,b)=>({x:a.y*b.z-a.z*b.y,y:a.z*b.x-a.x*b.z,z:a.x*b.y-a.y*b.x});
  const norm=a=>Math.hypot(a.x,a.y,a.z);
  const unit=a=>{const length=norm(a);return length?scale(a,1/length):{x:0,y:0,z:0}};
  const distance=(a,b)=>norm(sub(a,b));
  const radians=degrees=>degrees*Math.PI/180;
  const multiply=(a,b)=>[
    [0,1,2].map(j=>a[0][0]*b[0][j]+a[0][1]*b[1][j]+a[0][2]*b[2][j]),
    [0,1,2].map(j=>a[1][0]*b[0][j]+a[1][1]*b[1][j]+a[1][2]*b[2][j]),
    [0,1,2].map(j=>a[2][0]*b[0][j]+a[2][1]*b[1][j]+a[2][2]*b[2][j]),
  ];
  const transform=(matrix,vector)=>({
    x:matrix[0][0]*vector.x+matrix[0][1]*vector.y+matrix[0][2]*vector.z,
    y:matrix[1][0]*vector.x+matrix[1][1]*vector.y+matrix[1][2]*vector.z,
    z:matrix[2][0]*vector.x+matrix[2][1]*vector.y+matrix[2][2]*vector.z,
  });
  const transpose=matrix=>[[matrix[0][0],matrix[1][0],matrix[2][0]],[matrix[0][1],matrix[1][1],matrix[2][1]],[matrix[0][2],matrix[1][2],matrix[2][2]]];
  const identity=()=>[[1,0,0],[0,1,0],[0,0,1]];
  const Rx=angle=>{const c=Math.cos(angle),s=Math.sin(angle);return[[1,0,0],[0,c,-s],[0,s,c]]};
  const Ry=angle=>{const c=Math.cos(angle),s=Math.sin(angle);return[[c,0,s],[0,1,0],[-s,0,c]]};
  const Rz=angle=>{const c=Math.cos(angle),s=Math.sin(angle);return[[c,-s,0],[s,c,0],[0,0,1]]};
  const rotateAbout=(point,pivot,matrix)=>add(pivot,transform(matrix,sub(point,pivot)));
  const axisAngle=(axis,angle)=>{const u=unit(axis),c=Math.cos(angle),s=Math.sin(angle),t=1-c;return[
    [t*u.x*u.x+c,t*u.x*u.y-s*u.z,t*u.x*u.z+s*u.y],
    [t*u.x*u.y+s*u.z,t*u.y*u.y+c,t*u.y*u.z-s*u.x],
    [t*u.x*u.z-s*u.y,t*u.y*u.z+s*u.x,t*u.z*u.z+c],
  ]};
  const rotationFromTo=(from,to)=>{const a=unit(from),b=unit(to),c=Math.max(-1,Math.min(1,dot(a,b)));if(c>1-1e-12)return identity();if(c<-1+1e-12){const trial=Math.abs(a.x)<.8?{x:1,y:0,z:0}:{x:0,y:1,z:0};return axisAngle(cross(a,trial),Math.PI)}const v=cross(a,b),s=norm(v);return axisAngle(scale(v,1/s),Math.atan2(s,c))};
  const toVector=value=>({x:value[0],y:value[1],z:value[2]});
  const mmToM=value=>scale(value,.001);

  const lengths={
    crankL:distance(points.crankPivotL,points.upperBallL),
    crankR:distance(points.crankPivotR,points.upperBallR),
    rodL:distance(points.lowerBallL,points.upperBallL),
    rodR:distance(points.lowerBallR,points.upperBallR),
  };

  const zeroState=()=>({yaw:0,roll:0,pitch:0,crankL:0,crankR:0});

  function worldGeometry(input={}){
    const state={...zeroState(),...input};
    const platform=multiply(Rx(radians(state.roll)),Ry(radians(state.pitch)));
    const pivot=points.outputPivot;
    const crankPivotL=rotateAbout(points.crankPivotL,pivot,platform);
    const crankPivotR=rotateAbout(points.crankPivotR,pivot,platform);
    const upperBallL=add(crankPivotL,transform(multiply(platform,Ry(radians(state.crankL))),sub(points.upperBallL,points.crankPivotL)));
    const upperBallR=add(crankPivotR,transform(multiply(platform,Ry(radians(state.crankR))),sub(points.upperBallR,points.crankPivotR)));
    const localPoints={
      yawPivot:points.yawPivot,
      outputPivot:points.outputPivot,
      crankPivotL,crankPivotR,upperBallL,upperBallR,
      lowerBallL:points.lowerBallL,
      lowerBallR:points.lowerBallR,
    };
    const yaw=Rz(radians(state.yaw)),worldPoints={};
    for(const [key,value] of Object.entries(localPoints))worldPoints[key]=rotateAbout(value,points.yawPivot,yaw);
    const rodLengths={
      L:distance(worldPoints.lowerBallL,worldPoints.upperBallL),
      R:distance(worldPoints.lowerBallR,worldPoints.upperBallR),
    };
    const rodErrors={L:rodLengths.L-lengths.rodL,R:rodLengths.R-lengths.rodR};
    const closureResiduals={
      L:rodLengths.L*rodLengths.L-lengths.rodL*lengths.rodL,
      R:rodLengths.R*rodLengths.R-lengths.rodR*lengths.rodR,
    };
    const platformWorld=multiply(yaw,platform);
    const orientations={base:yaw,roll:multiply(yaw,Rx(radians(state.roll))),platform:platformWorld,crankL:multiply(platformWorld,Ry(radians(state.crankL))),crankR:multiply(platformWorld,Ry(radians(state.crankR)))};
    return{state,points:worldPoints,rodLengths,rodErrors,closureResiduals,platform,orientations};
  }

  const maxAbs=values=>Math.max(...values.map(value=>Math.abs(value)));
  const finite=values=>values.every(Number.isFinite);
  const solve2=(a,b,c,d,e,f)=>{
    const determinant=a*d-b*c;
    if(!Number.isFinite(determinant)||Math.abs(determinant)<1e-12)return null;
    return[(e*d-b*f)/determinant,(a*f-e*c)/determinant];
  };

  function newton(makeState,initial){
    let x=[...initial],lastState=null,lastGeometry=null,lastResidual=Infinity;
    const evaluate=value=>{
      const state=makeState(value),geometry=worldGeometry(state);
      return{state,geometry,residuals:[geometry.rodErrors.L,geometry.rodErrors.R]};
    };
    for(let iteration=0;iteration<=40;iteration++){
      const current=evaluate(x),residual=maxAbs(current.residuals);
      lastState=current.state;lastGeometry=current.geometry;lastResidual=residual;
      if(residual<1e-9)return{converged:true,iterations:iteration,residual,state:current.state,geometry:current.geometry};
      if(iteration===40||!finite(current.residuals))break;
      const h=1e-4,jacobian=[[],[]];
      for(let column=0;column<2;column++){
        const lo=[...x],hi=[...x];lo[column]-=h;hi[column]+=h;
        const rLo=evaluate(lo).residuals,rHi=evaluate(hi).residuals;
        jacobian[0][column]=(rHi[0]-rLo[0])/(2*h);
        jacobian[1][column]=(rHi[1]-rLo[1])/(2*h);
      }
      const delta=solve2(jacobian[0][0],jacobian[0][1],jacobian[1][0],jacobian[1][1],-current.residuals[0],-current.residuals[1]);
      if(!delta||!finite(delta))break;
      const bounded=delta.map(value=>Math.max(-12,Math.min(12,value)));
      let accepted=false;
      for(const factor of [1,.5,.25,.125,.0625,.03125]){
        const trial=[x[0]+bounded[0]*factor,x[1]+bounded[1]*factor];
        const next=evaluate(trial),nextResidual=maxAbs(next.residuals);
        if(Number.isFinite(nextResidual)&&nextResidual<residual){x=trial;accepted=true;break}
      }
      if(!accepted)break;
    }
    return{converged:false,iterations:40,residual:lastResidual,state:null,geometry:null,attemptedState:lastState,attemptedGeometry:lastGeometry};
  }

  function solveJoint(joint,outputDegrees,previousState=null){
    const output=Number(outputDegrees);
    if(!Number.isFinite(output))return{converged:false,iterations:0,residual:Infinity,state:null,geometry:null};
    if(joint==='J0'){
      const state={...zeroState(),yaw:output},geometry=worldGeometry(state);
      return{converged:true,iterations:0,residual:maxAbs(Object.values(geometry.rodErrors)),state,geometry};
    }
    if(joint==='J1'){
      const initial=previousState?[previousState.crankL,previousState.pitch]:[-.6878331741392661*output,0];
      return newton(([crank,pitch])=>({yaw:0,roll:output,pitch,crankL:crank,crankR:-crank}),initial);
    }
    if(joint==='J2'){
      const initial=previousState?[previousState.crankL,previousState.roll]:[-1.1122594417717997*output,0];
      return newton(([crank,roll])=>({yaw:0,roll,pitch:output,crankL:crank,crankR:crank}),initial);
    }
    throw new Error(`未知躯干关节 ${joint}`);
  }

  function solveActive(inputs={},previousState=null){
    const yaw=Number(inputs.yaw??0),crankL=Number(inputs.crankL??0),crankR=Number(inputs.crankR??0);
    if(!finite([yaw,crankL,crankR]))return{converged:false,iterations:0,residual:Infinity,state:null,geometry:null};
    const a=.6878331741392661,b=1.1122594417717997;
    const initial=previousState?[previousState.roll,previousState.pitch]:[(crankR-crankL)/(2*a),-(crankL+crankR)/(2*b)];
    return newton(([roll,pitch])=>({yaw,roll,pitch,crankL,crankR}),initial);
  }

  const rateKeys=['yaw','roll','pitch','crankL','crankR','ballLUpper','ballLLower','ballRUpper','ballRLower'];
  const outputOf=(joint,state)=>joint==='J0'?state.yaw:joint==='J1'?state.roll:state.pitch;
  function articulationDirections(geometry){
    const p=geometry.points,o=geometry.orientations;
    const rodL=unit(sub(p.upperBallL,p.lowerBallL)),rodR=unit(sub(p.upperBallR,p.lowerBallR));
    return{
      ballLLower:transform(transpose(o.base),rodL),
      ballRLower:transform(transpose(o.base),rodR),
      ballLUpper:transform(transpose(o.crankL),scale(rodL,-1)),
      ballRUpper:transform(transpose(o.crankR),scale(rodR,-1)),
    };
  }

  function rates(joint,outputDegrees,previousState=null){
    const center=solveJoint(joint,outputDegrees,previousState);
    if(!center.converged)return{converged:false,solution:center,values:null};
    const h=.001,lo=solveJoint(joint,outputDegrees-h,center.state),hi=solveJoint(joint,outputDegrees+h,center.state);
    if(!lo.converged||!hi.converged)return{converged:false,solution:center,values:null};
    const denominator=2*radians(h),values={};
    for(const key of ['yaw','roll','pitch','crankL','crankR'])values[key]=radians(hi.state[key]-lo.state[key])/denominator;
    const dLo=articulationDirections(lo.geometry),dHi=articulationDirections(hi.geometry);
    for(const key of ['ballLUpper','ballLLower','ballRUpper','ballRLower'])values[key]=norm(sub(dHi[key],dLo[key]))/denominator;
    return{converged:finite(Object.values(values)),solution:center,values};
  }

  const directionKey=direction=>direction>=0?'positive':'negative';
  const measured=(load,mode,direction)=>Math.max(0,Number(load?.[mode]?.[directionKey(direction)])||0);
  function equivalentResistance(joint,solution,loads={},direction=1,mode='static'){
    if(!solution?.converged)return{complete:false,total:0,parts:{},contribution:{}};
    const rate=rates(joint,outputOf(joint,solution.state),solution.state);
    if(!rate.converged)return{complete:false,total:0,parts:{},contribution:{}};
    const parts={},directions={};
    for(const key of rateKeys){
      directions[key]=Math.sign(rate.values[key])*(direction>=0?1:-1)||direction;
      parts[key]=measured(loads[key],mode,directions[key])*Math.abs(rate.values[key]);
    }
    const total=Object.values(parts).reduce((sum,value)=>sum+value,0),contribution={};
    for(const key of rateKeys)contribution[key]=total?100*parts[key]/total:0;
    return{complete:true,total,parts,contribution,rates:rate.values,directions,mode,direction};
  }

  function cycle(joint,loads={},options={}){
    const sampleCount=Math.max(5,Math.floor(Number(options.samples)||121)),samples=[];
    let previous=null;
    for(let index=0;index<sampleCount;index++){
      const time=index/(sampleCount-1),phase=2*Math.PI*time,output=10*Math.sin(phase),direction=Math.cos(phase)>=0?1:-1;
      const solution=solveJoint(joint,output,previous);
      if(!solution.converged)return{complete:false,failingIndex:index,time,solution,samples};
      previous=solution.state;
      const staticResult=equivalentResistance(joint,solution,loads,direction,'static');
      const dynamicResult=equivalentResistance(joint,solution,loads,direction,'dynamic');
      if(!staticResult.complete||!dynamicResult.complete)return{complete:false,failingIndex:index,time,solution,samples};
      samples.push({index,time,output,direction,solution,static:staticResult,dynamic:dynamicResult});
    }
    const integrate=mode=>{
      const totals=Object.fromEntries(rateKeys.map(key=>[key,0]));
      const dt=1/(sampleCount-1);
      for(let index=1;index<samples.length;index++)for(const key of rateKeys)totals[key]+=.5*(Math.abs(samples[index-1][mode].parts[key])+Math.abs(samples[index][mode].parts[key]))*dt;
      const denominator=Object.values(totals).reduce((sum,value)=>sum+value,0);
      return rateKeys.map(key=>({key,integral:totals[key],contribution:denominator?100*totals[key]/denominator:0})).sort((a,b)=>b.contribution-a.contribution);
    };
    return{
      complete:true,samples,
      rankings:{static:integrate('static'),dynamic:integrate('dynamic')},
      peaks:{static:Math.max(...samples.map(sample=>sample.static.total)),dynamic:Math.max(...samples.map(sample=>sample.dynamic.total))},
    };
  }

  const bodyAnchors={
    yaw:['yawPivot','yawPivot'],roll:['outputPivot','outputPivot'],platform:['outputPivot','outputPivot'],
    crankL:['crankPivotL','crankPivotL'],crankR:['crankPivotR','crankPivotR'],
    rodL:['lowerBallL','lowerBallL'],rodR:['lowerBallR','lowerBallR'],
  };
  function bodyPoses(solution){
    if(!solution?.converged)return{complete:false,bodies:{}};
    const geometry=solution.geometry,p=geometry.points,o=geometry.orientations;
    const rotations={
      yaw:o.base,roll:o.roll,platform:o.platform,crankL:o.crankL,crankR:o.crankR,
      rodL:multiply(o.base,rotationFromTo(sub(points.upperBallL,points.lowerBallL),transform(transpose(o.base),sub(p.upperBallL,p.lowerBallL)))),
      rodR:multiply(o.base,rotationFromTo(sub(points.upperBallR,points.lowerBallR),transform(transpose(o.base),sub(p.upperBallR,p.lowerBallR)))),
    };
    const bodies={};
    for(const body of massProperties.explicitBodies){
      const kind=massProperties.partMapping[body.partName],rotation=rotations[kind];
      const [initialKey,currentKey]=bodyAnchors[kind],initialAnchor=mmToM(points[initialKey]),currentAnchor=mmToM(p[currentKey]);
      const comInitial=toVector(body.comM),comM=add(currentAnchor,transform(rotation,sub(comInitial,initialAnchor)));
      bodies[body.partName]={partName:body.partName,kind,massKg:body.massKg,comM,rotation,inertia:body.inertia,status:body.status};
    }
    return{complete:true,bodies,state:solution.state};
  }
  const potentialEnergy=solution=>{
    const poses=bodyPoses(solution);if(!poses.complete)return NaN;
    const g=toVector(massProperties.gravity);
    return Object.values(poses.bodies).reduce((sum,body)=>sum-body.massKg*dot(g,body.comM),0);
  };
  function gravityHoldingTorque(joint,outputDegrees,previousState=null){
    const h=.001,center=solveJoint(joint,outputDegrees,previousState);if(!center.converged)return{complete:false,torqueNm:NaN,solution:center};
    const lo=solveJoint(joint,outputDegrees-h,center.state),hi=solveJoint(joint,outputDegrees+h,center.state);if(!lo.converged||!hi.converged)return{complete:false,torqueNm:NaN,solution:center};
    return{complete:true,torqueNm:(potentialEnergy(hi)-potentialEnergy(lo))/(2*radians(h)),potentialJ:potentialEnergy(center),solution:center};
  }
  const inertiaMatrix=body=>[[body.inertia.ixx,body.inertia.ixy,body.inertia.izx],[body.inertia.ixy,body.inertia.iyy,body.inertia.iyz],[body.inertia.izx,body.inertia.iyz,body.inertia.izz]];
  const matrixSub=(a,b)=>a.map((row,i)=>row.map((value,j)=>value-b[i][j]));
  const matrixScale=(a,s)=>a.map(row=>row.map(value=>value*s));
  function effectiveInertia(joint,outputDegrees,previousState=null){
    const h=.001,center=solveJoint(joint,outputDegrees,previousState);if(!center.converged)return{complete:false,kgM2:NaN,solution:center};
    const lo=solveJoint(joint,outputDegrees-h,center.state),hi=solveJoint(joint,outputDegrees+h,center.state);if(!lo.converged||!hi.converged)return{complete:false,kgM2:NaN,solution:center};
    const centerPoses=bodyPoses(center).bodies,loPoses=bodyPoses(lo).bodies,hiPoses=bodyPoses(hi).bodies,denominator=2*radians(h);let kgM2=0;
    for(const body of massProperties.explicitBodies){
      const c=centerPoses[body.partName],l=loPoses[body.partName],r=hiPoses[body.partName];
      const velocity=scale(sub(r.comM,l.comM),1/denominator),rdot=matrixScale(matrixSub(r.rotation,l.rotation),1/denominator),omegaMatrix=multiply(rdot,transpose(c.rotation));
      const omega={x:(omegaMatrix[2][1]-omegaMatrix[1][2])/2,y:(omegaMatrix[0][2]-omegaMatrix[2][0])/2,z:(omegaMatrix[1][0]-omegaMatrix[0][1])/2};
      const inertiaWorld=multiply(multiply(c.rotation,inertiaMatrix(c)),transpose(c.rotation)),iw=transform(inertiaWorld,omega);
      kgM2+=c.massKg*dot(velocity,velocity)+dot(omega,iw);
    }
    return{complete:true,kgM2,solution:center};
  }
  function trajectoryDynamics(joint,outputDegrees,options={}){
    const amplitude=radians(Number(options.amplitudeDegrees)||10),frequency=Number(options.frequencyHz)||1,omega=2*Math.PI*frequency,q=radians(outputDegrees),ratio=Math.max(0,1-Math.min(1,(q/amplitude)**2));
    const velocityRadS=(options.direction??1)*amplitude*omega*Math.sqrt(ratio),accelerationRadS2=-omega*omega*q,inertia=effectiveInertia(joint,outputDegrees),gravity=gravityHoldingTorque(joint,outputDegrees);
    if(!inertia.complete||!gravity.complete)return{complete:false};
    const h=.01,lo=effectiveInertia(joint,outputDegrees-h,inertia.solution.state),hi=effectiveInertia(joint,outputDegrees+h,inertia.solution.state),dJdq=lo.complete&&hi.complete?(hi.kgM2-lo.kgM2)/(2*radians(h)):0;
    const inertiaTorqueNm=inertia.kgM2*accelerationRadS2+.5*dJdq*velocityRadS*velocityRadS;
    return{complete:true,velocityRadS,accelerationRadS2,gravityTorqueNm:gravity.torqueNm,effectiveInertiaKgM2:inertia.kgM2,inertiaTorqueNm,holdingTorqueNm:gravity.torqueNm+inertiaTorqueNm,solution:inertia.solution};
  }

  window.TorsoModel={points,lengths,rateKeys,massProperties,zeroState,worldGeometry,solveJoint,solveActive,rates,equivalentResistance,cycle,bodyPoses,gravityHoldingTorque,effectiveInertia,trajectoryDynamics};
})();
