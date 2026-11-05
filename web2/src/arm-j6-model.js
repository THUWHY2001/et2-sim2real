(()=>{
  const nominal={
    upper1:[-3.771,294.982,18.385],lower1:[38.538,319.161,-96.683],
    upper2:[-35.362,298.628,7.536],lower2:[6.947,322.808,-107.532],
    axisA:[25.282,328.524,-106.969],axisB:[25.869,355.606,-96.874],
  };
  const add=(a,b)=>a.map((v,i)=>v+b[i]),sub=(a,b)=>a.map((v,i)=>v-b[i]),scale=(a,s)=>a.map(v=>v*s);
  const dot=(a,b)=>a.reduce((sum,v,i)=>sum+v*b[i],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const norm=a=>Math.hypot(...a),unit=a=>{const n=norm(a);if(!n)throw new Error('零长度方向无效');return scale(a,1/n)},distance=(a,b)=>norm(sub(a,b));
  const axis=unit(sub(nominal.axisB,nominal.axisA));
  const lengths={link1:distance(nominal.upper1,nominal.upper2),link2:distance(nominal.lower1,nominal.lower2)};
  const link1Direction=unit(sub(nominal.upper2,nominal.upper1)),link2Direction=unit(sub(nominal.lower2,nominal.lower1));
  const rotateVector=(vector,radians)=>{const c=Math.cos(radians),s=Math.sin(radians);return add(add(scale(vector,c),scale(cross(axis,vector),s)),scale(axis,dot(axis,vector)*(1-c)))};
  const rotatePoint=(point,radians)=>add(nominal.axisA,rotateVector(sub(point,nominal.axisA),radians));
  const clamp=value=>Math.max(-1,Math.min(1,value));
  const angleBetween=(a,b)=>Math.acos(clamp(dot(unit(a),unit(b))));
  function activeGeometry(overrides={}){
    const active={...lengths,...overrides};
    for(const key of ['link1','link2'])if(!Number.isFinite(active[key])||active[key]<=0)throw new Error(`${key} 长度必须大于0`);
    const upper1=[...nominal.upper1],upper2=add(upper1,scale(link1Direction,active.link1));
    const lower1=[...nominal.lower1],lower2=add(lower1,scale(link2Direction,active.link2));
    return{active,upper1,upper2,lower1,lower2};
  }
  function pose(degrees,overrides){
    const radians=degrees*Math.PI/180,g=activeGeometry(overrides);
    const lower1=rotatePoint(g.lower1,radians),lower2=rotatePoint(g.lower2,radians);
    const direction1=unit(sub(lower1,g.upper1)),direction2=unit(sub(lower2,g.upper2));
    const bodyDirection1=rotateVector(direction1,-radians),bodyDirection2=rotateVector(direction2,-radians);
    const reference1=unit(sub(g.lower1,g.upper1)),reference2=unit(sub(g.lower2,g.upper2));
    return{
      degrees,radians,lengths:g.active,
      points:{upper1:g.upper1,lower1,upper2:g.upper2,lower2,axisA:[...nominal.axisA],axisB:[...nominal.axisB]},
      directions:{direction1,direction2,bodyDirection1,bodyDirection2},
      coordinates:{
        upper1:angleBetween(reference1,direction1),lower1:angleBetween(reference1,bodyDirection1),
        upper2:angleBetween(reference2,direction2),lower2:angleBetween(reference2,bodyDirection2),
        rod1:distance(g.upper1,lower1),rod2:distance(g.upper2,lower2),
      },
    };
  }
  function vectorRate(lo,hi,dr){return norm(scale(sub(hi,lo),1/dr))}
  function solve(degrees,overrides){
    const state=pose(degrees,overrides),epsilon=.001,lo=pose(degrees-epsilon,overrides),hi=pose(degrees+epsilon,overrides),dr=2*epsilon*Math.PI/180;
    state.rates={
      upper1:vectorRate(lo.directions.direction1,hi.directions.direction1,dr),
      lower1:vectorRate(lo.directions.bodyDirection1,hi.directions.bodyDirection1,dr),
      upper2:vectorRate(lo.directions.direction2,hi.directions.direction2,dr),
      lower2:vectorRate(lo.directions.bodyDirection2,hi.directions.bodyDirection2,dr),
      rod1:(hi.coordinates.rod1-lo.coordinates.rod1)/dr,
      rod2:(hi.coordinates.rod2-lo.coordinates.rod2)/dr,
    };
    return state;
  }
  const keys=['upper1','lower1','upper2','lower2','rod1','rod2'];
  function equivalentTorque(state,loads,qDirection=1){
    const D=window.DirectionalFriction,parts={};
    for(const key of keys){const rate=state.rates[key],jac=key.startsWith('rod')?rate/1000:rate;parts[key]=D.directionalValue(loads[key],jac*qDirection,qDirection)*Math.abs(jac)}
    return{parts,total:Object.values(parts).reduce((sum,value)=>sum+value,0)};
  }
  function runningResistance(state,qdot,loads){
    const D=window.DirectionalFriction,parts={},powerParts={},directions={};
    for(const key of keys){const rate=state.rates[key],jac=key.startsWith('rod')?rate/1000:rate,velocity=jac*qdot,friction=D.frictionMagnitude(loads[key],velocity,qdot||1);parts[key]=friction*Math.abs(jac);powerParts[key]=friction*Math.abs(velocity);directions[key]=D.directionKey(velocity,qdot||1)}
    const total=Object.values(parts).reduce((sum,value)=>sum+value,0);
    return{parts,total,signedTotal:-Math.sign(qdot||1)*total,powerParts,power:Object.values(powerParts).reduce((sum,value)=>sum+value,0),directions};
  }
  window.ArmJ6Model={solve,equivalentTorque,runningResistance,nominal:{...nominal,axis:[...axis]},lengths};
})();
