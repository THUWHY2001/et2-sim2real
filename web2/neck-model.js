(() => {
  'use strict';

  const points={
    lower1:{x:-62,y:27,z:371.06},upper1:{x:-37.4,y:32,z:486.6},
    lower2:{x:-62,y:-27,z:371.06},upper2:{x:-37.4,y:-32,z:486.6},
    R1:{x:-52,y:0,z:479.66},R2:{x:-40,y:0,z:479.66},
    P1:{x:-16,y:20,z:479.66},P2:{x:-16,y:-20,z:479.66},
    com:{x:-9.851,y:.204,z:408.752},
  };
  const mass=2.447,g=9.80665;
  const keys={
    J0:{axis:['R1','R2'],inactive:['P1','P2'],rotary:['lower1','upper1','lower2','upper2','R1','R2'],rods:['rod1','rod2']},
    J1:{axis:['P1','P2'],inactive:['R1','R2'],rotary:['lower1','upper1','lower2','upper2','P1','P2'],rods:['rod1','rod2']},
  };
  const add=(a,b)=>({x:a.x+b.x,y:a.y+b.y,z:a.z+b.z});
  const sub=(a,b)=>({x:a.x-b.x,y:a.y-b.y,z:a.z-b.z});
  const scale=(a,s)=>({x:a.x*s,y:a.y*s,z:a.z*s});
  const dot=(a,b)=>a.x*b.x+a.y*b.y+a.z*b.z;
  const cross=(a,b)=>({x:a.y*b.z-a.z*b.y,y:a.z*b.x-a.x*b.z,z:a.x*b.y-a.y*b.x});
  const norm=a=>Math.hypot(a.x,a.y,a.z);
  const unit=a=>{const n=norm(a);return n?scale(a,1/n):{x:0,y:0,z:0}};
  const distance=(a,b)=>norm(sub(a,b));
  const axisFor=joint=>{const config=keys[joint];if(!config)throw new Error(`未知头颈关节 ${joint}`);const a=points[config.axis[0]],b=points[config.axis[1]];return{a,b,u:unit(sub(b,a))}};

  function rotateVector(v,axis,radians){const c=Math.cos(radians),s=Math.sin(radians);return add(add(scale(v,c),scale(cross(axis,v),s)),scale(axis,dot(axis,v)*(1-c)))}
  function rotatePoint(point,axis,radians){return add(axis.a,rotateVector(sub(point,axis.a),axis.u,radians))}

  function geometry(joint,degrees){
    const axis=axisFor(joint),radians=degrees*Math.PI/180;
    const upper1=rotatePoint(points.upper1,axis,radians),upper2=rotatePoint(points.upper2,axis,radians),com=rotatePoint(points.com,axis,radians);
    const rod1=unit(sub(upper1,points.lower1)),rod2=unit(sub(upper2,points.lower2));
    return{joint,degrees,radians,axis,lower1:points.lower1,lower2:points.lower2,upper1,upper2,com,rod1,rod2,lengths:{rod1:distance(points.lower1,upper1),rod2:distance(points.lower2,upper2)}};
  }

  function gravityTorqueOf(state){const force={x:0,y:0,z:-mass*g};return dot(cross(sub(state.com,state.axis.a),force),state.axis.u)/1000}
  function vectorRate(a,b,step){return norm(sub(b,a))/(2*step)}

  function solve(joint,degrees){
    const state=geometry(joint,degrees),step=1e-5,delta=step*180/Math.PI,lo=geometry(joint,degrees-delta),hi=geometry(joint,degrees+delta);
    const config=keys[joint],rates={R1:0,R2:0,P1:0,P2:0};
    rates[config.axis[0]]=1;rates[config.axis[1]]=1;
    rates.lower1=vectorRate(lo.rod1,hi.rod1,step);
    rates.lower2=vectorRate(lo.rod2,hi.rod2,step);
    const loUpper1=rotateVector(lo.rod1,state.axis.u,-lo.radians),hiUpper1=rotateVector(hi.rod1,state.axis.u,-hi.radians);
    const loUpper2=rotateVector(lo.rod2,state.axis.u,-lo.radians),hiUpper2=rotateVector(hi.rod2,state.axis.u,-hi.radians);
    rates.upper1=vectorRate(loUpper1,hiUpper1,step);
    rates.upper2=vectorRate(loUpper2,hiUpper2,step);
    state.rates=rates;
    state.rodJacobians={rod1:(hi.lengths.rod1-lo.lengths.rod1)/(2*step),rod2:(hi.lengths.rod2-lo.lengths.rod2)/(2*step)};
    state.gravityTorque=gravityTorqueOf(state);
    const jac1=state.rodJacobians.rod1/1000,jac2=state.rodJacobians.rod2/1000,denominator=jac1*jac1+jac2*jac2;
    const force1=denominator>1e-12?-state.gravityTorque*jac1/denominator:0,force2=denominator>1e-12?-state.gravityTorque*jac2/denominator:0;
    const gravity={x:0,y:0,z:-mass*g};
    const reaction=scale(add(add(gravity,scale(state.rod1,force1)),scale(state.rod2,force2)),-1);
    const radial=norm(sub(reaction,scale(state.axis.u,dot(reaction,state.axis.u))));
    state.actuatorForces={rod1:force1,rod2:force2};
    state.hingeReaction=reaction;
    state.supportLoads={lower1:Math.abs(force1),upper1:Math.abs(force1),lower2:Math.abs(force2),upper2:Math.abs(force2),R1:0,R2:0,P1:0,P2:0};
    state.supportLoads[config.axis[0]]=radial/2;state.supportLoads[config.axis[1]]=radial/2;
    return state;
  }

  const directionKey=direction=>direction>=0?'positive':'negative';
  const measured=(load,mode,direction)=>Math.max(0,Number(load?.[mode]?.[directionKey(direction)])||0);
  function equivalent(joint,state,loads={},direction=1,mode='static'){
    const config=keys[joint],reference=solve(joint,0),parts={};
    for(const key of config.rotary){const load=loads[key]||{},base=measured(load,mode,direction),kN=Math.max(0,Number(load.kN)||0),corrected=Math.max(0,base+kN*(state.supportLoads[key]-reference.supportLoads[key]));parts[key]=corrected*Math.abs(state.rates[key]||0)}
    for(const key of config.rods){const force=measured(loads[key]||{},mode,direction),jacobian=Math.abs(state.rodJacobians[key])/1000;parts[key]=force*jacobian}
    const frictionTotal=Object.values(parts).reduce((sum,value)=>sum+value,0),contribution={};
    for(const [key,value] of Object.entries(parts))contribution[key]=frictionTotal?100*value/frictionTotal:0;
    const holdTorque=-state.gravityTorque,driveTorque=holdTorque+(direction>=0?1:-1)*frictionTotal;
    return{parts,contribution,frictionTotal,gravityTorque:state.gravityTorque,holdTorque,driveTorque,supportLoads:state.supportLoads};
  }

  function cycle(joint,loads={},steps=360){
    const amplitude=10*Math.PI/180,omega=2*Math.PI,dt=1/steps;let peakAbsoluteTorque=0,peakAngle=0,frictionEnergy=0,gravityWork=0;
    for(let index=0;index<steps;index++){const t=index/steps,q=amplitude*Math.sin(omega*t),qdot=amplitude*omega*Math.cos(omega*t),degrees=q*180/Math.PI,direction=qdot>=0?1:-1,state=solve(joint,degrees),result=equivalent(joint,state,loads,direction,'dynamic');const absolute=Math.abs(result.driveTorque);if(absolute>peakAbsoluteTorque){peakAbsoluteTorque=absolute;peakAngle=degrees}frictionEnergy+=result.frictionTotal*Math.abs(qdot)*dt;gravityWork+=result.holdTorque*qdot*dt}
    return{peakAbsoluteTorque,peakAngle,frictionEnergy,gravityWork};
  }

  window.NeckModel={points,mass,g,keys,geometry,solve,equivalent,cycle};
})();
