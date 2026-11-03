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
    return state;
  }

  window.NeckToleranceModel={pointKeys,limit,validateOffsets,projectOffset,applyOffset,createState};
})();
