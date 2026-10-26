(()=>{
  const nominal={
    axisA:[-37.709,301.469,107.885],axisB:[-37.047,231.932,82.584],
    P:[-53.292,267.46,92.73],Q:[-77.299,260.564,111.055],
    R:[-30.952,261.768,108.958],S:[-63.991,213.39,240.935],
  };
  const add=(a,b)=>a.map((v,i)=>v+b[i]),sub=(a,b)=>a.map((v,i)=>v-b[i]),scale=(a,s)=>a.map(v=>v*s);
  const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const norm=a=>Math.hypot(...a),unit=a=>{const n=norm(a);if(!n)throw new Error('zero-length vector');return scale(a,1/n)};
  const distance=(a,b)=>norm(sub(a,b)),wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));

  const axis=unit(sub(nominal.axisB,nominal.axisA));
  const O=add(nominal.axisA,scale(axis,dot(sub(nominal.P,nominal.axisA),axis)));
  const ex=unit(sub(nominal.P,O)),ey=unit(cross(axis,ex));
  const local=p=>{const v=sub(p,O);return {x:dot(v,ex),y:dot(v,ey),z:dot(v,axis)}};
  const world=p=>add(O,add(add(scale(ex,p.x),scale(ey,p.y)),scale(axis,p.z)));
  const localNominal=Object.fromEntries(['P','Q','R','S'].map(k=>[k,local(nominal[k])]));
  const lengths={L:distance(O,nominal.P),Z:distance(nominal.P,nominal.Q),K:distance(nominal.Q,nominal.R),pushrodReference:distance(nominal.Q,nominal.S)};
  const planarRadius=(length,dz,label)=>{const squared=length*length-dz*dz;if(squared<-1e-7)throw new Error(`arm J3 ${label} closure has no solution`);return Math.sqrt(Math.max(0,squared))};
  function normalizedLengths(overrides={}){const active={...lengths,...overrides};for(const [key,value] of Object.entries(active))if(!Number.isFinite(value)||value<=0)throw new Error(`${key} 杆长必须大于 0`);return active}

  function circleIntersection(c0,r0,c1,r1,reference){
    const dx=c1.x-c0.x,dy=c1.y-c0.y,d=Math.hypot(dx,dy);
    if(!d||d>r0+r1+1e-7||d<Math.abs(r0-r1)-1e-7)throw new Error('arm J3 closure has no solution');
    const a=(r0*r0-r1*r1+d*d)/(2*d),h2=r0*r0-a*a;if(h2<-1e-7)throw new Error('arm J3 closure has no solution');const h=Math.sqrt(Math.max(0,h2));
    const x=c0.x+a*dx/d,y=c0.y+a*dy/d,rx=-dy*h/d,ry=dx*h/d;
    const candidates=[{x:x+rx,y:y+ry},{x:x-rx,y:y-ry}];
    return candidates.sort((u,v)=>Math.hypot(u.x-reference.x,u.y-reference.y)-Math.hypot(v.x-reference.x,v.y-reference.y))[0];
  }
  const angle2=(a,b)=>Math.atan2(b.y-a.y,b.x-a.x);
  const signedAngle=(a,b)=>Math.atan2(a.x*b.y-a.y*b.x,a.x*b.x+a.y*b.y);
  function base(degrees,overrides){
    const active=normalizedLengths(overrides),theta=degrees*Math.PI/180,p0=localNominal.P,pRadius=planarRadius(active.L,p0.z,'L'),pNominalRadius=Math.hypot(p0.x,p0.y),pScale=pRadius/pNominalRadius;
    const PZero={x:p0.x*pScale,y:p0.y*pScale,z:p0.z};
    const P2={x:PZero.x*Math.cos(theta)-PZero.y*Math.sin(theta),y:PZero.x*Math.sin(theta)+PZero.y*Math.cos(theta),z:PZero.z};
    const zPlan=planarRadius(active.Z,localNominal.Q.z-P2.z,'Z'),kPlan=planarRadius(active.K,localNominal.Q.z-localNominal.R.z,'K');
    const Q2=circleIntersection(P2,zPlan,localNominal.R,kPlan,localNominal.Q);
    Q2.z=localNominal.Q.z;
    const P=world(P2),Q=world(Q2),R=[...nominal.R],S=[...nominal.S];
    const sToQ0={x:localNominal.Q.x-localNominal.S.x,y:localNominal.Q.y-localNominal.S.y};
    const sToQ={x:Q2.x-localNominal.S.x,y:Q2.y-localNominal.S.y};
    const swing=signedAngle(sToQ0,sToQ),rodAxis=unit(sub(Q,S));
    const coordinates={
      p:wrap(angle2(P2,Q2)-angle2({x:0,y:0},P2)),
      q:wrap(angle2(localNominal.R,Q2)-angle2(P2,Q2)),
      r:angle2(localNominal.R,Q2),b:swing,
      rod:distance(S,Q),
    };
    return {degrees,radians:theta,points:{O:[...O],P,Q,R,S},rodAxis,swing,coordinates,lengths:active};
  }

  function solve(degrees,overrides){
    const state=base(degrees,overrides),epsilon=.001,lo=base(degrees-epsilon,overrides),hi=base(degrees+epsilon,overrides),dr=2*epsilon*Math.PI/180,rates={};
    for(const key of ['p','q','r','b'])rates[key]=wrap(hi.coordinates[key]-lo.coordinates[key])/dr;
    rates.rod=(hi.coordinates.rod-lo.coordinates.rod)/dr;
    state.rates=rates;
    return state;
  }

  function equivalentTorque(state,loads,qDirection=1){
    const D=window.DirectionalFriction,parts={};
    for(const key of ['p','q','r','b'])parts[key]=D.directionalValue(loads[key],state.rates[key]*qDirection,qDirection)*Math.abs(state.rates[key]);
    const rodJac=state.rates.rod/1000;
    parts.rod=D.directionalValue(loads.rod,rodJac*qDirection,qDirection)*Math.abs(rodJac);
    return {parts,total:Object.values(parts).reduce((a,b)=>a+b,0)};
  }

  function runningResistance(state,qdot,loads){
    const D=window.DirectionalFriction,parts={},powerParts={},directions={};
    for(const key of ['p','q','r','b']){
      const velocity=state.rates[key]*qdot,friction=D.frictionMagnitude(loads[key],velocity,qdot);
      parts[key]=friction*Math.abs(state.rates[key]);powerParts[key]=friction*Math.abs(velocity);directions[key]=D.directionKey(velocity,qdot);
    }
    const rodJac=state.rates.rod/1000,velocity=rodJac*qdot,friction=D.frictionMagnitude(loads.rod,velocity,qdot);
    parts.rod=friction*Math.abs(rodJac);powerParts.rod=friction*Math.abs(velocity);directions.rod=D.directionKey(velocity,qdot);
    const total=Object.values(parts).reduce((a,b)=>a+b,0);
    return {parts,total,signedTotal:-Math.sign(qdot||1)*total,powerParts,power:Object.values(powerParts).reduce((a,b)=>a+b,0),directions};
  }

  window.ArmJ3Model={solve,equivalentTorque,runningResistance,nominal:{...nominal,O:[...O],axis:[...axis]},lengths};
})();
