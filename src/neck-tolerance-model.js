(() => {
  'use strict';

  const M=window.NeckModel;
  if(!M)throw new Error('NeckToleranceModel requires NeckModel');
  const pointKeys=['lower1','upper1','lower2','upper2','R1','R2','P1','P2'];
  const limit=1;
  const add=(a,b)=>({x:a.x+b.x,y:a.y+b.y,z:a.z+b.z});
  const sub=(a,b)=>({x:a.x-b.x,y:a.y-b.y,z:a.z-b.z});
  const scale=(a,s)=>({x:a.x*s,y:a.y*s,z:a.z*s});
  const dot=(a,b)=>a.x*b.x+a.y*b.y+a.z*b.z;
  const cross=(a,b)=>({x:a.y*b.z-a.z*b.y,y:a.z*b.x-a.x*b.z,z:a.x*b.y-a.y*b.x});
  const norm=a=>Math.hypot(a.x,a.y,a.z);
  const unit=a=>{const n=norm(a);return n?scale(a,1/n):{x:0,y:0,z:0}};
  const distance=(a,b)=>norm(sub(a,b));
  const vectorRate=(a,b,step)=>norm(sub(b,a))/(2*step);
  const vector=value=>({x:Number(value?.x)||0,y:Number(value?.y)||0,z:Number(value?.z)||0});
  const cloneOffsets=offsets=>Object.fromEntries(pointKeys.map(key=>[key,vector(offsets?.[key])]));
  const axisFrom=(a,b)=>({a:{...a},b:{...b},u:unit(sub(b,a))});
  const nominalAxis=joint=>{
    const config=M.keys[joint];
    if(!config)throw new Error(`未知头颈关节 ${joint}`);
    return axisFrom(M.points[config.axis[0]],M.points[config.axis[1]]);
  };
  const rotateVector=(v,axis,radians)=>{const c=Math.cos(radians),s=Math.sin(radians);return add(add(scale(v,c),scale(cross(axis,v),s)),scale(axis,dot(axis,v)*(1-c)))};
  const rotatePoint=(point,axis,radians)=>add(axis.a,rotateVector(sub(point,axis.a),axis.u,radians));
  const distanceToAxis=(point,axis)=>norm(sub(sub(point,axis.a),scale(axis.u,dot(sub(point,axis.a),axis.u))));

  function validateOffsets(offsets){
    const invalid=[];
    for(const key of pointKeys){const value=vector(offsets?.[key]);const magnitude=norm(value);if(!Number.isFinite(magnitude)||magnitude>limit+1e-12)invalid.push({key,magnitude})}
    return{valid:invalid.length===0,invalid,limit};
  }

  function projectOffset(offset){
    const value=vector(offset),magnitude=norm(value);
    return magnitude>limit?scale(value,limit/magnitude):value;
  }

  function applyOffset(offsets,key,value){
    if(!pointKeys.includes(key))throw new Error(`未知硬点 ${key}`);
    const next=cloneOffsets(offsets),candidate=vector(value);
    if(norm(candidate)>limit+1e-12)return{accepted:false,offsets:next};
    next[key]=candidate;
    return{accepted:true,offsets:next};
  }

  function geometry(joint,degrees,offsets){
    const checked=validateOffsets(offsets);
    if(!checked.valid)throw new RangeError('硬点偏移超出 1 mm 球形边界');
    const delta=cloneOffsets(offsets),axis=nominalAxis(joint),radians=degrees*Math.PI/180;
    const zeroPoints=Object.fromEntries(pointKeys.map(key=>[key,add(M.points[key],delta[key])]));
    const points={...zeroPoints};
    points.upper1=rotatePoint(zeroPoints.upper1,axis,radians);
    points.upper2=rotatePoint(zeroPoints.upper2,axis,radians);
    const com=rotatePoint(M.points.com,axis,radians);
    const rod1=unit(sub(points.upper1,points.lower1)),rod2=unit(sub(points.upper2,points.lower2));
    const axes={J0:nominalAxis('J0'),J1:nominalAxis('J1')};
    const radialResiduals={};
    for(const key of ['R1','R2'])radialResiduals[key]=distanceToAxis(zeroPoints[key],axes.J0);
    for(const key of ['P1','P2'])radialResiduals[key]=distanceToAxis(zeroPoints[key],axes.J1);
    const nominalRod1=unit(sub(M.points.upper1,M.points.lower1)),nominalRod2=unit(sub(M.points.upper2,M.points.lower2));
    const closureResiduals={
      lower1:norm(cross(delta.lower1,nominalRod1)),upper1:norm(cross(delta.upper1,nominalRod1)),
      lower2:norm(cross(delta.lower2,nominalRod2)),upper2:norm(cross(delta.upper2,nominalRod2)),
    };
    return{joint,degrees,radians,axis,points,com,rod1,rod2,radialResiduals,closureResiduals,lengths:{rod1:distance(points.lower1,points.upper1),rod2:distance(points.lower2,points.upper2)}};
  }

  function createState(joint,degrees,offsets={}){
    const state=geometry(joint,degrees,offsets),step=1e-5,delta=step*180/Math.PI;
    const lo=geometry(joint,degrees-delta,offsets),hi=geometry(joint,degrees+delta,offsets);
    state.rodJacobians={rod1:(hi.lengths.rod1-lo.lengths.rod1)/(2*step),rod2:(hi.lengths.rod2-lo.lengths.rod2)/(2*step)};
    const config=M.keys[joint],rates=Object.fromEntries(pointKeys.map(key=>[key,0]));
    rates[config.axis[0]]=1;rates[config.axis[1]]=1;
    rates.lower1=vectorRate(lo.rod1,hi.rod1,step);
    rates.lower2=vectorRate(lo.rod2,hi.rod2,step);
    const loUpper1=rotateVector(lo.rod1,state.axis.u,-lo.radians),hiUpper1=rotateVector(hi.rod1,state.axis.u,-hi.radians);
    const loUpper2=rotateVector(lo.rod2,state.axis.u,-lo.radians),hiUpper2=rotateVector(hi.rod2,state.axis.u,-hi.radians);
    rates.upper1=vectorRate(loUpper1,hiUpper1,step);
    rates.upper2=vectorRate(loUpper2,hiUpper2,step);
    state.rates=rates;
    const nominal=M.solve(joint,degrees);
    state.lengthErrors={rod1:state.lengths.rod1-nominal.lengths.rod1,rod2:state.lengths.rod2-nominal.lengths.rod2};
    state.gravityTorque=nominal.gravityTorque;
    return state;
  }

  function createDefaultParameters(){
    const points=Object.fromEntries(pointKeys.map(key=>[key,{radialStiffness:500,clearancePositive:.05,clearanceNegative:.05,staticLoadCoefficient:.002,dynamicLoadCoefficient:.001}]));
    const rods={rod1:{axialStiffness:800,staticLoadCoefficient:.02,dynamicLoadCoefficient:.01},rod2:{axialStiffness:800,staticLoadCoefficient:.02,dynamicLoadCoefficient:.01}};
    return{points,rods,servoTorsionalStiffness:{J0:0,J1:0},zeroPin:{J0:null,J1:null}};
  }

  function sideLoad(residual,parameters={},direction=1){
    const stiffness=Math.max(0,Number(parameters.radialStiffness)||0);
    const clearance=Math.max(0,Number(direction>=0?parameters.clearancePositive:parameters.clearanceNegative)||0);
    return stiffness*Math.max(0,Math.max(0,Number(residual)||0)-clearance);
  }

  const directionKey=direction=>direction>=0?'positive':'negative';
  const measured=(load,mode,direction)=>Math.max(0,Number(load?.[mode]?.[directionKey(direction)])||0);
  function equivalentResistance(joint,state,parameters=createDefaultParameters(),loads={},direction=1,mode='static'){
    const config=M.keys[joint],parts=Object.fromEntries([...pointKeys,'rod1','rod2'].map(key=>[key,0])),sideLoads={};
    for(const key of pointKeys){
      const residual=key[0]==='R'||key[0]==='P'?state.radialResiduals[key]:(state.closureResiduals[key]||0);
      const pointParameters=parameters.points?.[key]||{};
      const load=sideLoad(residual,pointParameters,direction);
      sideLoads[key]=load;
      if(config.rotary.includes(key)){
        const coefficient=Math.max(0,Number(pointParameters[mode==='static'?'staticLoadCoefficient':'dynamicLoadCoefficient'])||0);
        const friction=measured(loads[key],mode,direction)+coefficient*load;
        parts[key]=friction*Math.abs(state.rates[key]||0);
      }
    }
    const axialForces={};
    for(const key of ['rod1','rod2']){
      const rodParameters=parameters.rods?.[key]||{};
      const axialForce=Math.max(0,Number(rodParameters.axialStiffness)||0)*Math.abs(state.lengthErrors[key]||0);
      axialForces[key]=axialForce;
      const coefficient=Math.max(0,Number(rodParameters[mode==='static'?'staticLoadCoefficient':'dynamicLoadCoefficient'])||0);
      const resistance=measured(loads[key],mode,direction)+coefficient*axialForce;
      parts[key]=resistance*Math.abs(state.rodJacobians[key]||0)/1000;
    }
    const frictionTotal=Object.values(parts).reduce((sum,value)=>sum+value,0);
    const contribution={};
    for(const [key,value] of Object.entries(parts))contribution[key]=frictionTotal?100*value/frictionTotal:0;
    const holdTorque=-state.gravityTorque,driveTorque=holdTorque+(direction>=0?1:-1)*frictionTotal;
    return{parts,contribution,frictionTotal,gravityTorque:state.gravityTorque,holdTorque,driveTorque,sideLoads,axialForces};
  }

  const defaultDirections=()=>{
    const values=[];
    for(const x of [-1,0,1])for(const y of [-1,0,1])for(const z of [-1,0,1])if(x||y||z)values.push(unit({x,y,z}));
    return values;
  };
  const pairDirections=()=>[{x:1,y:0,z:0},{x:-1,y:0,z:0},{x:0,y:1,z:0},{x:0,y:-1,z:0},{x:0,y:0,z:1},{x:0,y:0,z:-1}];
  const normalizedDirections=values=>(values?.length?values:defaultDirections()).map(value=>unit(vector(value))).filter(value=>norm(value)>0);
  function refineDirections(best){
    const result=[best];
    for(const axis of [{x:1,y:0,z:0},{x:-1,y:0,z:0},{x:0,y:1,z:0},{x:0,y:-1,z:0},{x:0,y:0,z:1},{x:0,y:0,z:-1}])result.push(unit(add(best,scale(axis,.2))));
    return result;
  }

  function responseMetrics(joint,offsets,parameters,loads,steps=72){
    const zero=cloneOffsets(),amplitude=10*Math.PI/180,omega=2*Math.PI,staticDelta=[],dynamicDelta=[];
    let energyDelta=0;
    for(let index=0;index<steps;index++){
      const t=index/steps,q=amplitude*Math.sin(omega*t),qdot=amplitude*omega*Math.cos(omega*t),degrees=q*180/Math.PI,direction=qdot>=0?1:-1,dt=1/steps;
      const baseline=createState(joint,degrees,zero),current=createState(joint,degrees,offsets);
      const baseStatic=equivalentResistance(joint,baseline,parameters,loads,direction,'static');
      const nextStatic=equivalentResistance(joint,current,parameters,loads,direction,'static');
      const baseDynamic=equivalentResistance(joint,baseline,parameters,loads,direction,'dynamic');
      const nextDynamic=equivalentResistance(joint,current,parameters,loads,direction,'dynamic');
      staticDelta.push(nextStatic.frictionTotal-baseStatic.frictionTotal);
      dynamicDelta.push(nextDynamic.frictionTotal-baseDynamic.frictionTotal);
      energyDelta+=(nextDynamic.frictionTotal-baseDynamic.frictionTotal)*Math.abs(qdot)*dt;
    }
    const peak=values=>Math.max(0,...values.map(Math.abs));
    const rms=values=>Math.sqrt(values.reduce((sum,value)=>sum+value*value,0)/Math.max(1,values.length));
    const meanAbsolute=values=>values.reduce((sum,value)=>sum+Math.abs(value),0)/Math.max(1,values.length);
    return{staticPeak:peak(staticDelta),dynamicPeak:peak(dynamicDelta),dynamicRms:rms(dynamicDelta),dynamicMeanAbsolute:meanAbsolute(dynamicDelta),dynamicEnergy:Math.abs(energyDelta)};
  }

  function withContribution(items){
    const total=items.reduce((sum,item)=>sum+item.impact,0);
    return{total,items:items.map(item=>({...item,contribution:total?100*item.impact/total:0})).sort((a,b)=>b.impact-a.impact)};
  }
  function bestForPoint(joint,key,parameters,loads,options){
    const directions=normalizedDirections(options.directions),zero=cloneOffsets();
    let bestStatic=null,bestDynamic=null;
    const consider=direction=>{
      const offsets=cloneOffsets(zero);offsets[key]=scale(direction,limit);
      const metrics=responseMetrics(joint,offsets,parameters,loads,options.steps||72);
      const staticItem={point:key,direction,impact:metrics.staticPeak,metrics};
      const dynamicItem={point:key,direction,impact:metrics.dynamicPeak,metrics};
      if(!bestStatic||staticItem.impact>bestStatic.impact)bestStatic=staticItem;
      if(!bestDynamic||dynamicItem.impact>bestDynamic.impact)bestDynamic=dynamicItem;
    };
    directions.forEach(consider);
    if(options.refine!==false){refineDirections(bestStatic.direction).forEach(consider);refineDirections(bestDynamic.direction).forEach(consider)}
    return{static:bestStatic,dynamic:bestDynamic};
  }
  function rankContributions(joint,parameters=createDefaultParameters(),loads={},options={}){
    const found=pointKeys.map(key=>bestForPoint(joint,key,parameters,loads,options));
    return{static:withContribution(found.map(item=>item.static)),dynamic:withContribution(found.map(item=>item.dynamic))};
  }

  function rankPairContributions(joint,parameters=createDefaultParameters(),loads={},options={}){
    const directions=normalizedDirections(options.directions?.length?options.directions:pairDirections()),zero=cloneOffsets(),staticItems=[],dynamicItems=[];
    for(let a=0;a<pointKeys.length;a++)for(let b=a+1;b<pointKeys.length;b++){
      const first=pointKeys[a],second=pointKeys[b];let bestStatic=null,bestDynamic=null;
      for(const firstDirection of directions)for(const secondDirection of directions){
        const offsets=cloneOffsets(zero);offsets[first]=scale(firstDirection,limit);offsets[second]=scale(secondDirection,limit);
        const metrics=responseMetrics(joint,offsets,parameters,loads,options.steps||72);
        const staticItem={points:[first,second],directions:[firstDirection,secondDirection],impact:metrics.staticPeak,metrics};
        const dynamicItem={points:[first,second],directions:[firstDirection,secondDirection],impact:metrics.dynamicPeak,metrics};
        if(!bestStatic||staticItem.impact>bestStatic.impact)bestStatic=staticItem;
        if(!bestDynamic||dynamicItem.impact>bestDynamic.impact)bestDynamic=dynamicItem;
      }
      staticItems.push(bestStatic);dynamicItems.push(bestDynamic);
    }
    return{static:withContribution(staticItems),dynamic:withContribution(dynamicItems)};
  }

  window.NeckToleranceModel={pointKeys,limit,validateOffsets,projectOffset,applyOffset,createState,createDefaultParameters,sideLoad,equivalentResistance,responseMetrics,rankContributions,rankPairContributions};
})();
