/** ET2 left-arm initial-geometry kinematics; body313 axes are provided by model.json.
 *  Sign convention used by geometry API: rod stroke + means increasing ball-to-ball / hinge-to-ball distance,
 *  and output angles are relative perturbations about initial CMD pose (not absolute joint zero).
 *  No gravity/constraint-force/dynamic torque solver is implemented here. */
const D2R = Math.PI / 180;
const EPS = 1e-12;
const sub = (a,b) => a.map((v,i) => v-b[i]);
const add = (a,b) => a.map((v,i) => v+b[i]);
const norm = (a) => Math.hypot(...a);
const dot = (a,b) => a.reduce((s,v,i)=>s+v*b[i],0);
const rotateX = (p,t) => [p[0],Math.cos(t)*p[1]-Math.sin(t)*p[2],Math.sin(t)*p[1]+Math.cos(t)*p[2]];
const rotateY = (p,t) => [Math.cos(t)*p[0]+Math.sin(t)*p[2],p[1],-Math.sin(t)*p[0]+Math.cos(t)*p[2]];

export function createLeftArmBlackbox(model) {
  if (!model?.hardpoints || !model?.joints || !model?.motions) throw new Error('Invalid left-arm model.json');
  const point = (name) => {
    const p = model.hardpoints[name]?.xyz_mm;
    if (!p) throw new Error(`Missing hardpoint: ${name}`);
    return p;
  };
  const [O6,O7,UA,UB,LA,LB] = [
    'arm_l_link6','arm_l_link7','arm_l_link13','arm_l_link16',
    'arm_l_ball_link15','arm_l_ball_link18'
  ].map(point);
  const [E0,EA,ED,EC,EB] = [
    'arm_l_link4','arm_l_link11','arm_l_link12','arm_l_ball_link10','arm_l_link8'
  ].map(point).map(p=>[p[0],p[2]]);

  // Wrist: J_ARM_L_06_RE (+X) -> J_ARM_L_07_RE (+Y), both axes in initial pose.
  const wristLengthsRad = (roll,pitch) => {
    const lower = (p) => add(O6,rotateX(add(sub(O7,O6),rotateY(sub(p,O7),pitch)),roll));
    return [norm(sub(UA,lower(LA))),norm(sub(UB,lower(LB)))];
  };
  const wristL0 = wristLengthsRad(0,0);
  function wristPoseToRods({rollDeg=0,pitchDeg=0}={}) {
    const length = wristLengthsRad(rollDeg*D2R,pitchDeg*D2R);
    return {leftStrokeMm:length[0]-wristL0[0],rightStrokeMm:length[1]-wristL0[1],
            lengthMm:{left:length[0],right:length[1]},referenceLengthMm:{left:wristL0[0],right:wristL0[1]},
            convention:'physical extension positive, NOT CMD T-axis sign'};
  }
  function wristRodsToPose({leftStrokeMm=0,rightStrokeMm=0,initialRollDeg=0,initialPitchDeg=0}={}) {
    const target=[wristL0[0]+leftStrokeMm,wristL0[1]+rightStrokeMm];
    if(target.some(x=>x<=0)) throw new Error('Non-positive wrist rod length');
    let r=initialRollDeg*D2R,p=initialPitchDeg*D2R, normErr=Infinity;
    for(let i=0;i<50;i++) {
      const f=wristLengthsRad(r,p).map((v,k)=>v-target[k]);normErr=Math.hypot(...f);
      if(normErr<1e-7) return {rollDeg:r/D2R,pitchDeg:p/D2R,residualMm:normErr,iterations:i};
      const h=1e-5;
      const frp=wristLengthsRad(r+h,p),frm=wristLengthsRad(r-h,p);
      const fpp=wristLengthsRad(r,p+h),fpm=wristLengthsRad(r,p-h);
      const a=(frp[0]-frm[0])/(2*h),b=(fpp[0]-fpm[0])/(2*h),
            c=(frp[1]-frm[1])/(2*h),d=(fpp[1]-fpm[1])/(2*h);
      const det=a*d-b*c;if(Math.abs(det)<1e-7) throw new Error('Wrist Jacobian singular');
      const dr=(-d*f[0]+b*f[1])/det,dp=(c*f[0]-a*f[1])/det;
      // Damped Newton step to keep the branch close to the provided seed.
      const maxstep=0.16,fac=Math.min(1,maxstep/Math.max(Math.abs(dr),Math.abs(dp),EPS));
      r+=dr*fac;p+=dp*fac;
    }
    throw new Error(`Wrist IK did not converge, residual=${normErr.toFixed(6)} mm`);
  }
  function wristJacobianMmPerRad({rollDeg=0,pitchDeg=0}={}){
    const r=rollDeg*D2R,p=pitchDeg*D2R,h=1e-5;
    const rplus=wristLengthsRad(r+h,p),rminus=wristLengthsRad(r-h,p);
    const pplus=wristLengthsRad(r,p+h),pminus=wristLengthsRad(r,p-h);
    return [[(rplus[0]-rminus[0])/(2*h),(pplus[0]-pminus[0])/(2*h)],
            [(rplus[1]-rminus[1])/(2*h),(pplus[1]-pminus[1])/(2*h)]];
  }

  // Elbow: ideal planar four-bar closure in global X-Z plane at CMD initial geometry.
  // J04 output axis +Y, J11 and J12 pivot about the same planar normal.
  const eR1=norm(sub(EA,EC)),eR2=norm(sub(ED,EC));
  const eL0=norm(sub(EB,EC));
  const orient=(A,B,C)=>(B[0]-A[0])*(C[1]-A[1])-(B[1]-A[1])*(C[0]-A[0]);
  const branchSign=Math.sign(orient(EA,ED,EC))||1;
  function elbowFourbarPoint(deltaRad){
    const d0=sub(ED,E0),t=deltaRad;
    const D=add(E0,[Math.cos(t)*d0[0]+Math.sin(t)*d0[1],-Math.sin(t)*d0[0]+Math.cos(t)*d0[1]]);
    const v=sub(D,EA),sep=norm(v);
    if(sep<1e-9) throw new Error('Elbow four-bar centers coincide');
    const x=(eR1*eR1-eR2*eR2+sep*sep)/(2*sep);
    const h2=eR1*eR1-x*x;
    if(h2<-1e-7) throw new Error('No elbow four-bar closure at requested angle');
    const u=v.map(z=>z/sep),perp=[-u[1],u[0]];
    return add(add(EA,u.map(z=>z*x)),perp.map(z=>z*branchSign*Math.sqrt(Math.max(0,h2))));
  }
  const elbowLengthRad = (q)=>norm(sub(EB,elbowFourbarPoint(q)));
  function elbowPitchToRod({deltaPitchDeg=0}={}){
    const current=elbowLengthRad(deltaPitchDeg*D2R);
    return {rodStrokeMm:current-eL0,rodLengthMm:current,referenceLengthMm:eL0,
            convention:'relative from initial CMD pose, + means physical extension'};
  }
  function elbowRodToPitch({rodStrokeMm=0,initialDeltaPitchDeg=0}={}) {
    const target=eL0+rodStrokeMm;if(target<=0)throw new Error('Non-positive elbow rod length');
    let t=initialDeltaPitchDeg*D2R,err=Infinity;
    for(let i=0;i<55;i++) {
      const f=elbowLengthRad(t)-target;err=Math.abs(f);
      if(err<1e-7)return {deltaPitchDeg:t/D2R,residualMm:err,iterations:i};
      const h=1e-5,d=(elbowLengthRad(t+h)-elbowLengthRad(t-h))/(2*h);
      if(Math.abs(d)<1e-5)throw new Error('Elbow linkage near singular point; inverse is branch ambiguous');
      const step=Math.max(-0.10,Math.min(0.10,-f/d));t+=step;
    }
    throw new Error(`Elbow IK did not converge, residual=${err.toFixed(6)} mm`);
  }
  function elbowJacobianMmPerRad({deltaPitchDeg=0}={}){
    const t=deltaPitchDeg*D2R,h=1e-5;
    return (elbowLengthRad(t+h)-elbowLengthRad(t-h))/(2*h);
  }
  function rigidBodyMassProperties(partName){
    const p=model.parts[partName];
    if(!p || p.role!=='arm') return null;
    if(!p.dynamics_ready) throw new Error(`Missing arm dynamics: ${partName}`);
    return {
      body:partName,massKg:p.mass_kg,comLocalMm:p.com_body_local_initial_mm,
      comGlobalInitialMm:p.com_global_initial_mm,
      inertiaRawKgM2:p.inertia_raw_kg_m2,
      inertiaMarkerEuler313Rad:p.inertia_marker_euler313_rad,
      provenance:p.source,
      warning:'ADAMS raw inertia products; confirm target engine offdiagonal convention; COM follows body, not fixed world'
    };
  }
  function armDynamicsSummary(){
    return {gravityMps2:model.meta.gravity_xyz_m_s2,totalArmMassKg:model.mass_audit.total_kg,
      initialAggregateCOMMm:model.mass_audit.initial_com_global_mm,bodyCount:model.mass_audit.body_count,
      excludedInterfaces:model.mass_audit.interfaces_excluded,
      warning:'Aggregate COM valid at initial CMD configuration only; there is no built-in multibody dynamics solver'};
  }
  function summary(){
    return {scope:model.meta.scope,hardpoints:Object.keys(model.hardpoints).length,
      joints:Object.keys(model.joints).length,motions:Object.keys(model.motions).length,
      internalParts:model.meta.internal_arm_body_count,knownInternalMassKg:model.meta.internal_arm_known_mass_kg,
      missingHandMass:model.meta.hand_mass_missing,
      gravityMps2:model.meta.gravity_xyz_m_s2,
      cmdOutputDriven:Object.values(model.motions).filter(v=>v.active_cmd).length,
      physicalActiveInputs:Object.values(model.motions).filter(v=>v.recommended_active_for_real_actuator_model).length,
      revisionWarnings:model.version_conflicts.length};
  }
  function validate(){
    const unpaired=Object.entries(model.joints).filter(([,j])=>j.i_j_initial_distance_mm>0.01).map(([k])=>k);
    const missingMotions=Object.entries(model.motions).filter(([,m])=>!model.joints[m.joint]).map(([k])=>k);
    const zero=wristPoseToRods(),wrist1=wristPoseToRods({rollDeg:1}),wrist2=wristPoseToRods({pitchDeg:1});
    const iw=wristRodsToPose({leftStrokeMm:wrist1.leftStrokeMm,rightStrokeMm:wrist1.rightStrokeMm});
    const elbow1=elbowPitchToRod({deltaPitchDeg:1});const ie=elbowRodToPitch({rodStrokeMm:elbow1.rodStrokeMm});
    const armParts=Object.values(model.parts).filter(p=>p.role==='arm');
    const dynOK=armParts.length===18&&armParts.every(p=>p.mass_kg>0&&p.com_body_local_initial_mm?.length===3&&
      ['ixx','iyy','izz','ixy','izx','iyz'].every(k=>Number.isFinite(p.inertia_raw_kg_m2?.[k])))&&
      Math.abs(armParts.reduce((acc,p)=>acc+p.mass_kg,0)-3.7249532)<1e-8;
    return {ok:dynOK&&unpaired.length===0&&missingMotions.length===0&&Math.abs(zero.leftStrokeMm)<1e-7&&
        Math.abs(iw.rollDeg-1)<1e-5&&Math.abs(ie.deltaPitchDeg-1)<1e-5,
      unpairedJoints:unpaired,missingMotionTargets:missingMotions,
      wrist1DegRoll:{leftMm:wrist1.leftStrokeMm,rightMm:wrist1.rightStrokeMm},
      wrist1DegPitch:{leftMm:wrist2.leftStrokeMm,rightMm:wrist2.rightStrokeMm},
      elbow1DegPitch:{strokeMm:elbow1.rodStrokeMm},
      warning:'Only ideal geometry closure tested; dynamics, friction and version conflicts not resolved'};
  }
  return Object.freeze({
    summary,validate,armDynamicsSummary,rigidBodyMassProperties,
    hardpoints:()=>model.hardpoints,
    joint:(name)=>model.joints[name]??null,
    part:(name)=>model.parts[name]??null,
    motion:(name)=>model.motions[name]??null,
    modes:()=>({cmdActive:Object.entries(model.motions).filter(([,v])=>v.active_cmd).map(([k])=>k),
      realActuatorRecommended:Object.entries(model.motions).filter(([,v])=>v.recommended_active_for_real_actuator_model).map(([k])=>k)}),
    versionConflicts:()=>model.version_conflicts,
    wristPoseToRods,wristRodsToPose,wristJacobianMmPerRad,
    elbowPitchToRod,elbowRodToPitch,elbowJacobianMmPerRad
  });
}
