(() => {
  'use strict';

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
  const norm=a=>Math.hypot(a.x,a.y,a.z);
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
  const Rx=angle=>{const c=Math.cos(angle),s=Math.sin(angle);return[[1,0,0],[0,c,-s],[0,s,c]]};
  const Ry=angle=>{const c=Math.cos(angle),s=Math.sin(angle);return[[c,0,s],[0,1,0],[-s,0,c]]};
  const Rz=angle=>{const c=Math.cos(angle),s=Math.sin(angle);return[[c,-s,0],[s,c,0],[0,0,1]]};
  const rotateAbout=(point,pivot,matrix)=>add(pivot,transform(matrix,sub(point,pivot)));

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
    return{state,points:worldPoints,rodLengths,rodErrors,closureResiduals,platform};
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

  window.TorsoModel={points,lengths,zeroState,worldGeometry,solveJoint,solveActive};
})();
