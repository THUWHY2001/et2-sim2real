/** ET2 left leg full closed-chain forward + inverse actuator stroke solver.
 * No network or engine dependencies; supports browsers and Node.
 * The three linear inputs refer to PHYSICAL end-ball spans relative to the
 * user-provided nominal CMD geometry (J2=-27.5 J3=51.57 J4=-24.06 degrees).
 */
export function createCompleteLeftLeg(data) {
  const D = data;
  const P = n => D.hardpoints[`P_${n}`].xyz_mm;
  const parts = D.parts, joints = D.joints;
  const NOM = Object.freeze({J0:0,J1:0,J2:-27.5,J3:51.57,J4:-24.06,J5:0,toe:0});
  const AX = [1,0,0], AY=[0,1,0], AZ=[0,0,1];
  const v = a=>[...a], add=(a,b)=>a.map((x,i)=>x+b[i]), sub=(a,b)=>a.map((x,i)=>x-b[i]);
  const times=(a,t)=>a.map(x=>x*t),dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const norm=a=>Math.hypot(...a),unit=a=>{const n=norm(a);if(n<1e-14)throw Error('zero vector');return times(a,1/n);};
  const mm=(A,B)=>A.map((r,i)=>B[0].map((_,j)=>r.reduce((s,x,k)=>s+x*B[k][j],0)));
  const mv=(A,a)=>A.map(r=>dot(r,a));
  const mt=A=>A[0].map((_,i)=>A.map(r=>r[i]));
  const matI=()=>[[1,0,0],[0,1,0],[0,0,1]];
  const rot=(axis,deg)=>{
    const a=unit(axis),phi=deg*Math.PI/180,c=Math.cos(phi),s=Math.sin(phi),t=1-c;
    return [[c+a[0]*a[0]*t,a[0]*a[1]*t-a[2]*s,a[0]*a[2]*t+a[1]*s],
            [a[1]*a[0]*t+a[2]*s,c+a[1]*a[1]*t,a[1]*a[2]*t-a[0]*s],
            [a[2]*a[0]*t-a[1]*s,a[2]*a[1]*t+a[0]*s,c+a[2]*a[2]*t]];
  };
  const rotPt=(point,center,axis,deg)=>add(center,mv(rot(axis,deg),sub(point,center)));
  const align=(a0,b0)=>{
    const a=unit(a0),b=unit(b0),c=Math.max(-1,Math.min(1,dot(a,b))),vec=cross(a,b);
    if(c>1-1e-13)return matI();
    if(c< -1+1e-13)return rot(unit(cross(a,Math.abs(a[1])<0.9?AY:AX)),180);
    return rot(unit(vec),Math.acos(c)*180/Math.PI);
  };
  const signed2=(a,b)=>a[0]*b[1]-a[1]*b[0];
  const angleY=(a,b)=>Math.atan2(cross(a,b)[1],dot(a,b))*180/Math.PI;
  const T=(R,t)=>({R,t}), identity=()=>T(matI(),[0,0,0]);
  const at=(t,a)=>add(mv(t.R,a),t.t), ax=(t,a)=>mv(t.R,a);
  const compose=(a,b)=>T(mm(a.R,b.R),add(mv(a.R,b.t),a.t));
  const around=(p0,axis,deg)=>{let R=rot(axis,deg);return T(R,sub(p0,mv(R,p0)));};
  const fromPoints=(a0,b0,a1,b1,planar=true)=>{
    const r=planar?rot(AY,angleY(sub(b0,a0),sub(b1,a1))):align(sub(b0,a0),sub(b1,a1));
    return T(r,sub(a1,mv(r,a0)));
  };
  const rocker=(pi,pr,pt,newTip)=>{
    const a=[P(pi)[0],P(pi)[2]],b0=[P(pt)[0],P(pt)[2]],b=[newTip[0],newTip[2]];
    const r0=norm(sub([P(pr)[0],P(pr)[2]],a)),r1=norm(sub([P(pr)[0],P(pr)[2]],b0));
    const delta=sub(b,a),dist=norm(delta);
    if(dist<1e-10||dist>r0+r1+1e-7||dist<Math.abs(r0-r1)-1e-7) throw Error('four bar closure infeasible');
    const foot=(r0*r0-r1*r1+dist*dist)/(2*dist),h=Math.sqrt(Math.max(0,r0*r0-foot*foot));
    const u=times(delta,1/dist),n=[-u[1],u[0]],sols=[add(add(a,times(u,foot)),times(n,h)),sub(add(a,times(u,foot)),times(n,h))];
    const sgn0=signed2(sub(b0,a),sub([P(pr)[0],P(pr)[2]],a));
    const val=s=>signed2(sub(b,a),sub(s,a))*sgn0;
    const sol=val(sols[0])>=val(sols[1])?sols[0]:sols[1];
    return [sol[0],P(pi)[1],sol[1]];
  };
  function local(q2,q3,q4,q5){
    const hip10=rotPt(P(10),P(4),AY,q2),hip11=rotPt(P(11),P(4),AY,q2);
    const hip13=rocker(9,13,10,hip10);
    const knee18=rotPt(P(18),P(5),AY,q3),knee16=rocker(17,16,18,knee18);
    const ankle21=rotPt(rotPt(P(21),P(7),AX,q5),P(6),AY,q4);
    return {hip10,hip11,hip13,knee18,knee16,ankle21,
      lengths:{hip:norm(sub(hip13,hip11)),knee:norm(sub(knee16,P(14))),ankle:norm(sub(ankle21,P(19)))}};
  }
  const length=(k,q,roll=0)=>k==='hip'?local(q,0,0,0).lengths.hip:k==='knee'?local(0,q,0,0).lengths.knee:local(0,0,q,roll).lengths.ankle;
  const baseLen={hip:length('hip',NOM.J2),knee:length('knee',NOM.J3),ankle:length('ankle',NOM.J4)};
  function inverse(which,stroke,nominal,roll=0,seed){
    const target=baseLen[which]+Number(stroke);
    if(target<=0) throw Error(which+' rod target length nonpositive');
    let q=seed??nominal;
    if(Math.abs(stroke)<1e-12&&seed==null&&roll===0)return q;
    for(let i=0;i<60;i++){
      const error=length(which,q,roll)-target;
      if(Math.abs(error)<1e-8)return q;
      const h=1e-3,deriv=(length(which,q+h,roll)-length(which,q-h,roll))/(2*h);
      if(Math.abs(deriv)<1e-4)throw Error(which+' near singularity');
      const step=Math.max(-7,Math.min(7,-error/deriv));let good=false;
      for(let j=0;j<20;j++){
        const next=q+step*Math.pow(.5,j);
        if(Math.abs(next-nominal)>75)continue;
        try{
          if(Math.abs(length(which,next,roll)-target)<Math.abs(error)){q=next;good=true;break;}
        } catch{}
      }
      if(!good)throw Error(which+' stroke unreachable on nominal branch');
    }
    throw Error(which+' inverse did not converge');
  }
  const owners={2:'leg_l_link1',3:'leg_l_link2',4:'leg_l_link2',5:'leg_l_link3',6:'leg_l_link4',7:'leg_l_link5',8:'leg_l_link6',9:'leg_l_link2',10:'leg_l_link3',11:'leg_l_link3',13:'leg_l_link8',14:'leg_l_link3',16:'leg_l_link16',17:'leg_l_link3',18:'leg_l_link4',19:'leg_l_link4',21:'leg_l_link6'};
  const prism={12:['leg_l_link10','leg_l_link11'],15:['leg_l_link13','leg_l_link14'],20:['leg_l_link18','leg_l_link19']};
  const actuatorPairs={hip:[11,13,'leg_l_link3','leg_l_link8','J2'],knee:[14,16,'leg_l_link3','leg_l_link16','J3'],ankle:[19,21,'leg_l_link4','leg_l_link6','J4']};
  function direct(input={}){
    const q={...NOM,...input};
    const L=local(q.J2,q.J3,q.J4,q.J5);
    const t={base_link:identity()};
    const chain=[['leg_l_link1','base_link',2,'J_LEG_L_HIP_roll_RE','J0'],['leg_l_link2','leg_l_link1',3,'J_LEG_L_HIP_yaw_RE','J1'],
       ['leg_l_link3','leg_l_link2',4,'J_LEG_L_HIP_pitch_RE','J2'],['leg_l_link4','leg_l_link3',5,'J_LEG_L_KNEE_pitch_RE','J3'],
       ['leg_l_link5','leg_l_link4',6,'J_LEG_L_ANKLE_pitch_RE','J4'],['leg_l_link6','leg_l_link5',7,'J_LEG_L_ANKLE_roll_RE','J5'],
       ['leg_l_link7','leg_l_link6',8,'J_LEG_L_TOE_pitch_RE','toe']];
    for(const [body,parent,index,j,control] of chain)t[body]=compose(t[parent],around(P(index),joints[j].i_marker_axis_global,q[control]));
    t.leg_l_ball_link20=t.leg_l_link6;
    const hip8=fromPoints(P(9),P(13),P(9),L.hip13),hip9=fromPoints(P(13),P(10),L.hip13,L.hip10);
    const hip10=fromPoints(P(11),P(13),L.hip11,L.hip13),hip11=T(hip10.R,sub(L.hip13,mv(hip10.R,P(13))));
    for(const [name,val] of Object.entries({leg_l_link8:hip8,leg_l_ball_link12:hip8,leg_l_link9:hip9,leg_l_link10:hip10,leg_l_link11:hip11}))t[name]=compose(t.leg_l_link2,val);
    const knee16=fromPoints(P(17),P(16),P(17),L.knee16),knee17=fromPoints(P(16),P(18),L.knee16,L.knee18);
    const knee13=fromPoints(P(14),P(16),P(14),L.knee16),knee14=T(knee13.R,sub(L.knee16,mv(knee13.R,P(16))));
    for(const [name,val] of Object.entries({leg_l_link16:knee16,leg_l_ball_link15:knee16,leg_l_link17:knee17,leg_l_link13:knee13,leg_l_link14:knee14}))t[name]=compose(t.leg_l_link3,val);
    const a19=at(t.leg_l_link4,P(19)),a21=at(t.leg_l_link6,P(21));
    const a18=fromPoints(P(19),P(21),a19,a21,false),a19T=T(a18.R,sub(a21,mv(a18.R,P(21))));
    t.leg_l_link18=a18;t.leg_l_link19=a19T;
    for(const name of Object.keys(parts))if(!t[name])throw Error('missing transform '+name);
    let maxp=0,maxslide=0,maxax=0;
    const markers=[],checks=[];
    for(const [name,j] of Object.entries(joints)){
      const pold=j.initial_center_global_mm,pi=at(t[j.i_part],pold),pj=at(t[j.j_part],pold),gap=sub(pj,pi);
      const a=unit(ax(t[j.i_part],j.i_marker_axis_global)),b=unit(ax(t[j.j_part],j.j_marker_axis_global));
      const dist=norm(gap),lateral=norm(cross(gap,a)),axisCross=norm(cross(a,b));
      const err=j.type==='translational'?lateral:dist;
      if(j.type==='translational') maxslide=Math.max(maxslide,err);else maxp=Math.max(maxp,err);
      if(j.type==='revolute'||j.type==='translational')maxax=Math.max(maxax,axisCross);
      markers.push({joint:name,marker_side:'I',marker:j.i_marker,part:j.i_part,type:j.type,initial_xyz_mm:pold,xyz_mm:pi,axis_xyz:a,cmd_line:j.source_cmd_line});
      markers.push({joint:name,marker_side:'J',marker:j.j_marker,part:j.j_part,type:j.type,initial_xyz_mm:pold,xyz_mm:pj,axis_xyz:b,cmd_line:j.source_cmd_line});
      checks.push({joint:name,type:j.type,i_part:j.i_part,j_part:j.j_part,marker_pair_gap_mm:dist,
        lateral_gap_mm:j.type==='translational'?lateral:null,
        axial_sliding_signed_mm:j.type==='translational'?dot(gap,a):null,
        axis_cross_error:j.type==='revolute'||j.type==='translational'?axisCross:null,
        constraint_residual_mm:err,cmd_line:j.source_cmd_line});
    }
    const hps=[];
    for(let n=2;n<=21;n++){
      const initial=P(n);
      if(prism[n])for(let i=0;i<2;i++)hps.push({point:`P${n}-${i===0?'I':'J'}`,initial_xyz_mm:initial,xyz_mm:at(t[prism[n][i]],initial),body:prism[n][i],category:'prismatic_marker_side'});
      else hps.push({point:`P${n}`,initial_xyz_mm:initial,xyz_mm:at(t[owners[n]],initial),body:owners[n],category:'structural_hardpoint'});
    }
    const centers={};
    for(const [name,o] of Object.entries(D.output_joints)){
      const j=joints[o.joint_name],parent=t[j.i_part];
      centers[name]={xyz_mm:at(parent,j.initial_center_global_mm),initial_xyz_mm:j.initial_center_global_mm,
        axis_i_xyz:unit(ax(parent,j.i_marker_axis_global)),joint:o.joint_name,
        angle_cmd_initial_relative_deg:q[name==='toe_extra'?'toe':name]};
    }
    let mass=0,csum=[0,0,0];const coms={},inertias={};
    for(const [name,b] of Object.entries(parts)){
      const c=at(t[name],b.com_initial_global_mm),m=b.mass_kg;
      coms[name]=c;mass+=m;csum=add(csum,times(c,m));
      const I=b.inertia_raw_kg_m2;
      const mat=[[I.ixx,I.ixy,I.izx],[I.ixy,I.iyy,I.iyz],[I.izx,I.iyz,I.izz]];
      inertias[name]=mm(mm(t[name].R,mat),mt(t[name].R));
    }
    const measured={};
    for(const [name,[a,b,ba,bb,angle]] of Object.entries(actuatorPairs)){
      const aa=at(t[ba],P(a)),bbpos=at(t[bb],P(b)),len=norm(sub(bbpos,aa));
      measured[name]={anchor1_xyz_mm:aa,anchor2_xyz_mm:bbpos,ball_center_length_mm:len,
        delta_vs_cmd_mm:len-length(name,0),delta_vs_nominal_mm:len-baseLen[name],
        slide_joint:{hip:'J_LEG_L_HIP_pitch_T',knee:'J_LEG_L_KNEE_pitch_T',ankle:'J_LEG_L_ANKLE_pitch_T'}[name],
        passive_output:angle,original_cmd_motion_name:D.actuators[{hip:'hip_pitch',knee:'knee_pitch',ankle:'ankle_pitch'}[name]].motion};
    }
    const check={max_nonsliding_pair_gap_mm:maxp,max_prismatic_lateral_error_mm:maxslide,
      max_revolute_prismatic_axis_cross:maxax,max_allowed_prismatic_lateral_error_mm:.002,
      ok:maxp<2e-5&&maxslide<.002&&maxax<2e-6};
    return {controls:{angles_cmd_relative_deg:q},
     passive_outputs:{J2_hip_pitch_deg:q.J2,J3_knee_pitch_deg:q.J3,J4_ankle_pitch_deg:q.J4,
       delta_from_nominal_deg:{J2:q.J2-NOM.J2,J3:q.J3-NOM.J3,J4:q.J4-NOM.J4}},
     active_rod_inputs:measured,joint_centers:centers,all_hardpoints:hps,all_joint_marker_sides:markers,
     joint_constraint_checks:checks,world_transforms:Object.fromEntries(Object.entries(t).map(([k,r])=>[k,{R:r.R,t_mm:r.t}])),
     mass:{total_kg:mass,COM_world_mm:coms,aggregate_COM_world_mm:times(csum,1/mass),
       raw_inertia_kg_m2_by_body:Object.fromEntries(Object.entries(parts).map(([k,b])=>[k,b.inertia_raw_kg_m2])),
       rotated_inertia_tensor_assuming_raw_offdiagonals_kg_m2:inertias},validation:check};
  }
  function solve(input={}){
    const h=Number(input.hip_mm??0),k=Number(input.knee_mm??0),a=Number(input.ankle_mm??0);
    const j5=Number(input.J5_deg??0);
    const q={J0:Number(input.J0_deg??0),J1:Number(input.J1_deg??0),
      J2:inverse('hip',h,NOM.J2,0,input.seed_J2_deg),
      J3:inverse('knee',k,NOM.J3,0,input.seed_J3_deg),
      J4:inverse('ankle',a,NOM.J4,j5,input.seed_J4_deg),
      J5:j5,toe:Number(input.toe_deg??0)};
    const result=direct(q);
    result.controls.motor_inputs={hip_rod_mm:h,knee_rod_mm:k,ankle_rod_mm:a,
        J0_deg:q.J0,J1_deg:q.J1,J5_deg:q.J5,toe_deg:q.toe};
    result.controls.actuator_input_definition='end-to-end physical extension relative to fixed user nominal pose at J5=0; J5 coupling included; not signed CMD Motion';
    result.validation.inverse_length_error_mm={hip:result.active_rod_inputs.hip.ball_center_length_mm-baseLen.hip-h,
      knee:result.active_rod_inputs.knee.ball_center_length_mm-baseLen.knee-k,
      ankle:result.active_rod_inputs.ankle.ball_center_length_mm-baseLen.ankle-a};
    return result;
  }
  function validate(){
    const cs=[{}, {hip_mm:1,knee_mm:1,ankle_mm:1},{hip_mm:-1,knee_mm:1,ankle_mm:-1},
      {hip_mm:2,knee_mm:-2,ankle_mm:2},
      {hip_mm:.5,knee_mm:.5,ankle_mm:.5,J0_deg:5,J1_deg:-4,J5_deg:3}];
    const arr=cs.map(c=>{
      const r=solve(c);
      return {input:c,angles:r.controls.angles_cmd_relative_deg,residual:r.validation};
    });
    const ok=arr.every(x=>x.residual.ok&&Object.values(x.residual.inverse_length_error_mm).every(z=>Math.abs(z)<1e-7));
    return {ok,parts:Object.keys(parts).length,original_points:20,joints:Object.keys(joints).length,
      all_joint_marker_sides:50,motions:Object.keys(D.motions).length,nominal_deg:NOM,
      nominal_ball_lengths_mm:baseLen,cases:arr};
  }
  return {solve,directAngles:direct,validate,nominal:NOM,nominalBallLengths:baseLen,
    part:name=>parts[name],joint:name=>joints[name],motion:name=>D.motions[name],point:n=>D.hardpoints[`P_${n}`],
    lists:()=>({parts:Object.keys(parts),joints:Object.keys(joints),motions:Object.keys(D.motions)})};
}
