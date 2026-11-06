#!/usr/bin/env python3
"""ET2 left leg: three linear inputs -> passive hip/knee/ankle pitch and all rigid bodies.
All original CMD geometry is preserved in model.json. Right hand child relative parent joint angles.
Linear inputs are changes in the END-TO-END spherical anchor distance measured from the user nominal pose.
"""
import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL = json.loads((ROOT / 'model.json').read_text(encoding='utf-8'))
PT = {k: np.array(v['xyz_mm'], dtype=float) for k, v in MODEL['hardpoints'].items()}
PARTS = MODEL['parts']
JOINTS = MODEL['joints']
Y = np.array([0.0, 1.0, 0.0])
X = np.array([1.0, 0.0, 0.0])
Z = np.array([0.0, 0.0, 1.0])
NOM = {'J0': 0.0, 'J1': 0.0, 'J2': -27.5, 'J3': 51.57,
       'J4': -24.06, 'J5': 0.0, 'toe': 0.0}
LINEARS = {'hip': ('J2', 'hip_pitch', 'J_LEG_L_HIP_pitch_T', 11, 13),
           'knee': ('J3', 'knee_pitch', 'J_LEG_L_KNEE_pitch_T', 14, 16),
           'ankle': ('J4', 'ankle_pitch', 'J_LEG_L_ANKLE_pitch_T', 19, 21)}
POINT_OWNERS = {2:'leg_l_link1',3:'leg_l_link2',4:'leg_l_link2',5:'leg_l_link3',
 6:'leg_l_link4',7:'leg_l_link5',8:'leg_l_link6',9:'leg_l_link2',10:'leg_l_link3',
 11:'leg_l_link3',13:'leg_l_link8',14:'leg_l_link3',16:'leg_l_link16',
 17:'leg_l_link3',18:'leg_l_link4',19:'leg_l_link4',21:'leg_l_link6'}
PRISM = {12: ('leg_l_link10','leg_l_link11'),15: ('leg_l_link13','leg_l_link14'),
         20: ('leg_l_link18','leg_l_link19')}


def p(n):
    return PT[f'P_{n}']


def unit(v):
    v = np.array(v, dtype=float)
    t = np.linalg.norm(v)
    if t < 1e-14: raise ValueError('zero vector')
    return v / t


def rotation(axis, deg):
    axis = unit(axis)
    phi = math.radians(float(deg))
    K = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]],
                  [-axis[1], axis[0], 0.]])
    return np.eye(3) + math.sin(phi)*K + (1-math.cos(phi))*(K@K)


def rotate_about(v, axis_point, axis, deg):
    R = rotation(axis,deg)
    return np.array(axis_point) + R@(np.array(v)-np.array(axis_point))


def align3(a,b):
    a,b = unit(a),unit(b)
    v=np.cross(a,b);c=float(np.clip(a@b,-1.,1.))
    if c > 1-1e-13:return np.eye(3)
    if c < -1+1e-13:
        ortho=unit(np.cross(a, [0,1,0] if abs(a[1])<0.9 else [1,0,0]))
        return rotation(ortho,180)
    k=v/np.linalg.norm(v)
    return rotation(k,math.degrees(math.acos(c)))


def angle_y(a,b):
    a=np.asarray(a);b=np.asarray(b)
    return math.degrees(math.atan2(np.cross(a,b)[1],float(a@b)))


@dataclass
class T:
    R: np.ndarray
    t: np.ndarray
    def at(self, vec): return self.R@np.asarray(vec, dtype=float)+self.t
    def axis(self, vec): return self.R@np.asarray(vec, dtype=float)


def ident(): return T(np.eye(3),np.zeros(3))


def compose(a,b):return T(a.R@b.R,a.R@b.t+a.t)


def about(point, axis, deg):
    R=rotation(axis,deg)
    return T(R,np.asarray(point)-R@np.asarray(point))


def from_points(a0,b0,a1,b1,planar=True):
    a0,b0,a1,b1=map(np.asarray,(a0,b0,a1,b1))
    R=rotation(Y,angle_y(b0-a0,b1-a1)) if planar else align3(b0-a0,b1-a1)
    return T(R,a1-R@a0)


def sphere_circle(center, radius, moving_point, length, initial_tip):
    """Continuous branch of planar 4-bar circle intersection in source link frame."""
    a=np.asarray(center)[[0,2]];b=np.asarray(moving_point)[[0,2]]
    ref=np.asarray(initial_tip)[[0,2]]
    dd=b-a;d=float(np.linalg.norm(dd))
    if d<1e-10 or d>radius+length+1e-7 or d<abs(radius-length)-1e-7:
        raise ValueError('4-bar triangle closure infeasible: distance=%.8f' % d)
    foot=(radius*radius-length*length+d*d)/(2*d)
    hh=math.sqrt(max(0., radius*radius-foot*foot))
    v=dd/d;cross=np.array([-v[1],v[0]])
    cand=[a+foot*v+hh*cross,a+foot*v-hh*cross]
    orig_b=np.asarray(initial_tip)[[0,2]]
    # Original circle center-to-move-point is not always the same as the new
    # circle center-to-move-point; preserve triangle chirality.
    old_b=np.asarray(moving_point)[[0,2]]
    # Chirality is determined externally in rocker(): initial endpoint P10 or P18.
    closest=min(cand,key=lambda q: np.linalg.norm(q-ref))
    return closest


def rocker_point(pivot0,rocker_tip0,link_tip0,moved_link_tip):
    # Choose same assembly orientation as original geometry, not a branch
    # that happens to be nearest the old position after a large angle.
    c0=p(pivot0)[[0,2]];d0=p(link_tip0)[[0,2]]
    c1=np.asarray(moved_link_tip)[[0,2]]
    r0=float(np.linalg.norm(p(rocker_tip0)[[0,2]]-c0))
    r1=float(np.linalg.norm(p(rocker_tip0)[[0,2]]-d0))
    dv=c1-c0;dd=float(np.linalg.norm(dv))
    if dd<1e-10 or dd>r0+r1+1e-7 or dd<abs(r0-r1)-1e-7:
        raise ValueError('4-bar branch infeasible near '+str((pivot0,rocker_tip0,link_tip0)))
    aa=(r0*r0-r1*r1+dd*dd)/(2*dd)
    h=math.sqrt(max(0.,r0*r0-aa*aa))
    u=dv/dd;n=np.array([-u[1],u[0]])
    origtip=p(rocker_tip0)[[0,2]]
    original_area=float(np.linalg.det(np.stack([d0-c0,origtip-c0])))
    candidates=[c0+aa*u+h*n,c0+aa*u-h*n]
    new_sign=lambda c:float(np.linalg.det(np.stack([c1-c0,c-c0])))
    chosen=max(candidates,key=lambda c: new_sign(c)*original_area)
    return np.array([chosen[0],p(pivot0)[1],chosen[1]])


def local_mechanisms(q2,q3,q4,q5):
    """Independent geometric loop geometry in the upstream link's CMD frame."""
    hp10=rotate_about(p(10),p(4),Y,q2)
    hp11=rotate_about(p(11),p(4),Y,q2)
    hp13=rocker_point(9,13,10,hp10)
    kp18=rotate_about(p(18),p(5),Y,q3)
    kp16=rocker_point(17,16,18,kp18)
    ap21=rotate_about(rotate_about(p(21),p(7),X,q5),p(6),Y,q4)
    rods={'hip':float(np.linalg.norm(hp13-hp11)),
          'knee':float(np.linalg.norm(kp16-p(14))),
          'ankle':float(np.linalg.norm(ap21-p(19)))}
    return {'hip_p10':hp10,'hip_p11':hp11,'hip_p13':hp13,
            'knee_p18':kp18,'knee_p16':kp16,'ankle_p21':ap21,
            'lengths':rods}


def rod_len(which,q,j5=0.):
    if which=='hip':return local_mechanisms(q,0,0,0)['lengths']['hip']
    if which=='knee':return local_mechanisms(0,q,0,0)['lengths']['knee']
    if which=='ankle':return local_mechanisms(0,0,q,j5)['lengths']['ankle']
    raise KeyError(which)


def inverse_one(which,physical_delta,nominal_angle,j5_deg=0.,seed=None):
    """Solve physical ball-to-ball extension relative to nominal (mm).
    Uses local branch Newton and bounded steps; will not silently branch-switch.
    """
    initial=NOMINAL_LENGTHS[which]
    target=initial+float(physical_delta)
    if target<=0:raise ValueError('nonpositive target rod span '+which)
    x=float(nominal_angle if seed is None else seed)
    if abs(float(physical_delta))<1e-12 and seed is None and (which!='ankle' or j5_deg==0):return x
    for count in range(60):
        current=rod_len(which,x,j5_deg)
        error=current-target
        if abs(error)<1e-8:return x
        step=1e-3
        deriv=(rod_len(which,x+step,j5_deg)-rod_len(which,x-step,j5_deg))/(2*step)
        if abs(deriv)<1e-4:raise ValueError(which+' near kinematic singularity')
        raw=np.clip(-error/deriv,-7.,7.)
        accepted=False
        for j in range(20):
            cand=x+raw*(0.5**j)
            if abs(cand-nominal_angle)>75:continue
            try: e2=rod_len(which,cand,j5_deg)-target
            except ValueError:continue
            if abs(e2)<abs(error):x=float(cand);accepted=True;break
        if not accepted:raise ValueError(which+' target length unreachable on nominal assembly branch')
    raise ValueError(which+' inverse iteration did not converge')


def direct(angles, validation=True):
    # All angles relative to original CMD geometry; child-body RH about I-marker axis.
    q={**NOM, **{k:float(v) for k,v in angles.items()}}
    m=local_mechanisms(q['J2'],q['J3'],q['J4'],q['J5'])
    Tmap={'base_link':ident()}
    chain=[('leg_l_link1','base_link',2,'J_LEG_L_HIP_roll_RE','J0'),
           ('leg_l_link2','leg_l_link1',3,'J_LEG_L_HIP_yaw_RE','J1'),
           ('leg_l_link3','leg_l_link2',4,'J_LEG_L_HIP_pitch_RE','J2'),
           ('leg_l_link4','leg_l_link3',5,'J_LEG_L_KNEE_pitch_RE','J3'),
           ('leg_l_link5','leg_l_link4',6,'J_LEG_L_ANKLE_pitch_RE','J4'),
           ('leg_l_link6','leg_l_link5',7,'J_LEG_L_ANKLE_roll_RE','J5'),
           ('leg_l_link7','leg_l_link6',8,'J_LEG_L_TOE_pitch_RE','toe')]
    for child,parent,idx,joint,angle in chain:
        j=JOINTS[joint]
        Tmap[child]=compose(Tmap[parent],about(p(idx),np.asarray(j['i_marker_axis_global']),q[angle]))
    Tmap['leg_l_ball_link20']=Tmap['leg_l_link6']

    # Hip four-bar: link8 pivots on link2 at P9; link9 meets link3 at P10.
    h8_local=from_points(p(9),p(13),p(9),m['hip_p13'],planar=True)
    h9_local=from_points(p(13),p(10),m['hip_p13'],m['hip_p10'],planar=True)
    h10_local=from_points(p(11),p(13),m['hip_p11'],m['hip_p13'],planar=True)
    h11_local=T(h10_local.R.copy(),m['hip_p13']-h10_local.R@p(13))
    for part,local in [('leg_l_link8',h8_local),('leg_l_ball_link12',h8_local),
                       ('leg_l_link9',h9_local),('leg_l_link10',h10_local),('leg_l_link11',h11_local)]:
        Tmap[part]=compose(Tmap['leg_l_link2'],local)

    # Knee four-bar in link3 parent frame: P17 pivot, P18 on moving shank.
    k16_local=from_points(p(17),p(16),p(17),m['knee_p16'],planar=True)
    k17_local=from_points(p(16),p(18),m['knee_p16'],m['knee_p18'],planar=True)
    k13_local=from_points(p(14),p(16),p(14),m['knee_p16'],planar=True)
    k14_local=T(k13_local.R.copy(),m['knee_p16']-k13_local.R@p(16))
    for part,local in [('leg_l_link16',k16_local),('leg_l_ball_link15',k16_local),
                       ('leg_l_link17',k17_local),('leg_l_link13',k13_local),('leg_l_link14',k14_local)]:
        Tmap[part]=compose(Tmap['leg_l_link3'],local)

    # Ankle rod: both balls, P19 on leg4 and P21 on leg6 (after ankle-roll).
    # link18 body uses minimal 3D alignment when ankle roll creates spatial tilt.
    a19=Tmap['leg_l_link4'].at(p(19));a21=Tmap['leg_l_link6'].at(p(21))
    at18=from_points(p(19),p(21),a19,a21,planar=False)
    at19=T(at18.R.copy(),a21-at18.R@p(21))
    Tmap['leg_l_link18']=at18
    Tmap['leg_l_link19']=at19
    if set(Tmap)-{'base_link'}!=set(PARTS):
        raise RuntimeError('missing body transforms '+str(set(PARTS)-set(Tmap)))
    marker_rows=[];joint_checks=[];maxpos=0.;maxslide=0.;maxax=0.
    for jname,j in JOINTS.items():
        ti=Tmap[j['i_part']];tj=Tmap[j['j_part']]
        old=np.asarray(j['initial_center_global_mm'],dtype=float)
        pi=ti.at(old);pj=tj.at(old)
        ai=unit(ti.axis(j['i_marker_axis_global']));aj=unit(tj.axis(j['j_marker_axis_global']))
        gap=pj-pi
        non_slide=float(np.linalg.norm(gap))
        slide=float(np.linalg.norm(np.cross(gap,ai)))
        axis_cross=float(np.linalg.norm(np.cross(ai,aj)))
        err=slide if j['type']=='translational' else non_slide
        if j['type']=='translational':maxslide=max(maxslide,err)
        else:maxpos=max(maxpos,err)
        if j['type'] in ('revolute','translational'):maxax=max(maxax,axis_cross)
        marker_rows.extend([{'joint':jname,'marker_side':'I','marker':j['i_marker'],'part':j['i_part'],
                           'type':j['type'],'initial_xyz_mm':old.tolist(),'xyz_mm':pi.tolist(),'axis_xyz':ai.tolist(),
                           'cmd_line':j['source_cmd_line']},
                            {'joint':jname,'marker_side':'J','marker':j['j_marker'],'part':j['j_part'],
                           'type':j['type'],'initial_xyz_mm':old.tolist(),'xyz_mm':pj.tolist(),'axis_xyz':aj.tolist(),
                           'cmd_line':j['source_cmd_line']}])
        joint_checks.append({'joint':jname,'type':j['type'],'i_part':j['i_part'],
                 'j_part':j['j_part'],'marker_pair_gap_mm':non_slide,
                 'lateral_gap_mm':slide if j['type']=='translational' else None,
                 'axial_sliding_signed_mm':float(gap@ai) if j['type']=='translational' else None,
                 'axis_cross_error':axis_cross if j['type'] in ('revolute','translational') else None,
                 'constraint_residual_mm':err,'cmd_line':j['source_cmd_line']})

    hps=[]
    for idx in range(2,22):
        initial=p(idx)
        if idx in PRISM:
            for side,body in zip(('I','J'),PRISM[idx]):
                hps.append({'point':f'P{idx}-{side}','initial_xyz_mm':initial.tolist(),
                   'xyz_mm':Tmap[body].at(initial).tolist(),'body':body,'category':'prismatic_marker_side'})
        else:
            body=POINT_OWNERS[idx]
            hps.append({'point':f'P{idx}','initial_xyz_mm':initial.tolist(),
                   'xyz_mm':Tmap[body].at(initial).tolist(),'body':body,'category':'structural_hardpoint'})
    # 20 original points, with the three prismatic points represented twice -> 23.
    assert len(hps)==23
    centers={}
    for label,out in MODEL['output_joints'].items():
        j=JOINTS[out['joint_name']];Tpar=Tmap[j['i_part']]
        centers[label]={'xyz_mm':Tpar.at(j['initial_center_global_mm']).tolist(),
          'initial_xyz_mm':j['initial_center_global_mm'],
          'axis_i_xyz':unit(Tpar.axis(j['i_marker_axis_global'])).tolist(),
          'joint':out['joint_name'],'angle_cmd_initial_relative_deg':q['toe' if label=='toe_extra' else label]}
    coms={};inertias={};mass=0.;com_sum=np.zeros(3)
    for body,b in PARTS.items():
        c=Tmap[body].at(b['com_initial_global_mm']);coms[body]=c.tolist()
        mm=float(b['mass_kg']);mass+=mm;com_sum+=mm*c
        # All inertia markers in the checked 2026-07 CMD have zero Euler 313 angles.
        inert=b['inertia_raw_kg_m2'];I=np.array([[inert['ixx'],inert['ixy'],inert['izx']],
                        [inert['ixy'],inert['iyy'],inert['iyz']],
                        [inert['izx'],inert['iyz'],inert['izz']]])
        inertias[body]=(Tmap[body].R@I@Tmap[body].R.T).tolist()
    measured={}
    for key,(angle_name,act,joint,ia,ib) in LINEARS.items():
        # measured physical length from rigid-attachment world positions
        if key=='hip':ba,bb='leg_l_link3','leg_l_link8'
        elif key=='knee':ba,bb='leg_l_link3','leg_l_link16'
        else:ba,bb='leg_l_link4','leg_l_link6'
        A=Tmap[ba].at(p(ia));B=Tmap[bb].at(p(ib))
        measured[key]={'anchor1_xyz_mm':A.tolist(),'anchor2_xyz_mm':B.tolist(),
                        'ball_center_length_mm':float(np.linalg.norm(B-A)),
                        'delta_vs_cmd_mm':float(np.linalg.norm(B-A)-rod_len(key,0,0)),
                        'delta_vs_nominal_mm':float(np.linalg.norm(B-A)-NOMINAL_LENGTHS[key]),
                        'slide_joint':joint,'passive_output':angle_name,
                        'original_cmd_motion_name':MODEL['actuators'][act]['motion']}
    summary={'max_nonsliding_pair_gap_mm':maxpos,
             'max_prismatic_lateral_error_mm':maxslide,
             'max_revolute_prismatic_axis_cross':maxax,
             'max_allowed_prismatic_lateral_error_mm':0.002,
             'ok':maxpos<2e-5 and maxslide<0.002 and maxax<2e-6}
    return {'controls':{'angles_cmd_relative_deg':q},
      'passive_outputs':{'J2_hip_pitch_deg':q['J2'],'J3_knee_pitch_deg':q['J3'],
           'J4_ankle_pitch_deg':q['J4'],
           'delta_from_nominal_deg':{j:q[j]-NOM[j] for j in ('J2','J3','J4')}},
      'active_rod_inputs':measured,'joint_centers':centers,
      'all_hardpoints':hps,'all_joint_marker_sides':marker_rows,
      'joint_constraint_checks':joint_checks,
      'world_transforms':{k:{'R':v.R.tolist(),'t_mm':v.t.tolist()} for k,v in Tmap.items()},
      'mass':{'total_kg':mass,'COM_world_mm':coms,'aggregate_COM_world_mm':(com_sum/mass).tolist(),
              'raw_inertia_kg_m2_by_body':{k:v['inertia_raw_kg_m2'] for k,v in PARTS.items()},
              'rotated_inertia_tensor_assuming_raw_offdiagonals_kg_m2':inertias},
      'validation':summary}


def strokes_to_pose(hip_mm=0.,knee_mm=0.,ankle_mm=0.,J0=0.,J1=0.,J5=0.,toe=0.,seeds=None):
    # J0,J1,J5,toe are user right-hand input angle increments relative original CMD.
    # J5 is a separate rotary motor and affects ankle rod geometry (must be used in inverse).
    seeds=seeds or {}
    q2=inverse_one('hip',hip_mm,NOM['J2'],0,seeds.get('J2'))
    q3=inverse_one('knee',knee_mm,NOM['J3'],0,seeds.get('J3'))
    # All physical strokes refer to the SAME fixed user nominal pose (J5=0).
    # Roll motor changes ankle geometry and may require passive ankle pitch response.
    q4=inverse_one('ankle',ankle_mm,NOM['J4'],J5,seeds.get('J4'))
    angles={'J0':J0,'J1':J1,'J2':q2,'J3':q3,'J4':q4,'J5':J5,'toe':toe}
    result=direct(angles)
    result['controls']['motor_inputs']={'hip_rod_mm':hip_mm,'knee_rod_mm':knee_mm,
        'ankle_rod_mm':ankle_mm,'J0_deg':J0,'J1_deg':J1,'J5_deg':J5,'toe_deg':toe}
    result['controls']['actuator_input_definition']='end-to-end physical extension relative to fixed user nominal pose at J5=0; J5 coupling included; not signed CMD Motion'
    result['validation']['inverse_length_error_mm']={
         'hip':result['active_rod_inputs']['hip']['ball_center_length_mm']-NOMINAL_LENGTHS['hip']-hip_mm,
         'knee':result['active_rod_inputs']['knee']['ball_center_length_mm']-NOMINAL_LENGTHS['knee']-knee_mm,
         'ankle':result['active_rod_inputs']['ankle']['ball_center_length_mm']-
                  NOMINAL_LENGTHS['ankle']-ankle_mm}
    return result

# Nominal pose is held by actuator lengths derived from CMD geometry; no encoder zero inferred.
NOMINAL_LENGTHS={k:rod_len(k,NOM[a]) for k,(a,_,_,_,_) in LINEARS.items()}


def run_validation():
    part_count=len(PARTS);joint_count=len(JOINTS)
    cases=[('CMD zero',direct({j:0. for j in NOM})),
          ('User nominal',strokes_to_pose()),
          ('Stroke +1 +1 +1',strokes_to_pose(1,1,1)),
          ('Stroke -1 +1 -1',strokes_to_pose(-1,1,-1)),
          ('Stroke +2 -2 +2',strokes_to_pose(2,-2,2)),
          ('Hip roll yaw + ankle roll',strokes_to_pose(0.5,0.5,0.5,J0=5,J1=-4,J5=3))]
    mass0=sum(x['mass_kg'] for x in PARTS.values())
    rows=[]
    for name,r in cases:
        q=r['controls']['angles_cmd_relative_deg'];e=r['validation'];
        fixed_ref=all(abs(r['active_rod_inputs'][key]['ball_center_length_mm']-rod_len(key,q[{'hip':'J2','knee':'J3','ankle':'J4'}[key]],q['J5']))<1e-6 for key in LINEARS)
        rows.append({'case':name,'angles_deg':{k:round(v,8) for k,v in q.items()},
            'max_joint_point_error_mm':e['max_nonsliding_pair_gap_mm'],
            'max_prismatic_lateral_error_mm':e['max_prismatic_lateral_error_mm'],
            'max_revolute_prismatic_axis_cross':e['max_revolute_prismatic_axis_cross'],
            'rigid_geometry_ok':e['ok'] and fixed_ref,
            'mass_kg':r['mass']['total_kg']})
    mass_eig_ok=True
    for pdef in PARTS.values():
        inertia=pdef['inertia_raw_kg_m2'];tensor=np.array([[inertia['ixx'],inertia['ixy'],inertia['izx']],
                 [inertia['ixy'],inertia['iyy'],inertia['iyz']],
                 [inertia['izx'],inertia['iyz'],inertia['izz']]])
        if min(np.linalg.eigvalsh(tensor))<=0:mass_eig_ok=False
    success=all(x['rigid_geometry_ok'] for x in rows) and part_count==20 and joint_count==25 and len(MODEL['motions'])==10 and abs(mass0-13.02480932)<1e-7 and mass_eig_ok
    return {'ok':success,'source_cmd':MODEL['meta']['source_cmd'],
      'nominal_angles_cmd_relative_deg':NOM,
      'nominal_rod_lengths_mm':NOMINAL_LENGTHS,
      'parts':part_count,'original_points':len(PT),'joints':joint_count,
      'joint_marker_sides':2*joint_count,'motions':len(MODEL['motions']),
      'mass_kg':mass0,'all_raw_inertia_positive_definite':mass_eig_ok,'cases':rows}


def output_csv(result,outdir):
    outdir.mkdir(parents=True,exist_ok=True)
    def write(path,rows):
        keys=list(rows[0]);
        with (outdir/path).open('w',encoding='utf-8-sig',newline='') as f:
            w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
    def fmt(v): return json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v
    hp=[]
    for a in result['all_hardpoints']:
        hp.append({'point':a['point'],'part':a['body'],'category':a['category'],
         'original_x_mm':a['initial_xyz_mm'][0],'original_y_mm':a['initial_xyz_mm'][1],
         'original_z_mm':a['initial_xyz_mm'][2],
         'target_x_mm':a['xyz_mm'][0],'target_y_mm':a['xyz_mm'][1],'target_z_mm':a['xyz_mm'][2]})
    write('23_hardpoint_positions.csv',hp)
    jr=[]
    for a in result['all_joint_marker_sides']:
        jr.append({**{k:fmt(v) for k,v in a.items() if k not in ('initial_xyz_mm','xyz_mm','axis_xyz')},
         'original_x_mm':a['initial_xyz_mm'][0],'original_y_mm':a['initial_xyz_mm'][1],
         'original_z_mm':a['initial_xyz_mm'][2],
         'x_mm':a['xyz_mm'][0],'y_mm':a['xyz_mm'][1],'z_mm':a['xyz_mm'][2],
         'axis_x':a['axis_xyz'][0],'axis_y':a['axis_xyz'][1],'axis_z':a['axis_xyz'][2]})
    write('50_joint_marker_sides.csv',jr)
    jc=[{k:fmt(v) for k,v in r.items()} for r in result['joint_constraint_checks']]
    write('25_joint_constraint_checks.csv',jc)
    jc=[]
    for j,v in result['joint_centers'].items():
        jc.append({'joint':j,'cmd_name':v['joint'],'initial_x_mm':v['initial_xyz_mm'][0],
        'initial_y_mm':v['initial_xyz_mm'][1],'initial_z_mm':v['initial_xyz_mm'][2],
        'target_x_mm':v['xyz_mm'][0],'target_y_mm':v['xyz_mm'][1],
        'target_z_mm':v['xyz_mm'][2], 'axis_x':v['axis_i_xyz'][0],
        'axis_y':v['axis_i_xyz'][1],'axis_z':v['axis_i_xyz'][2],
        'angle_cmd_relative_deg':v['angle_cmd_initial_relative_deg']})
    write('7_output_joint_centers.csv',jc)
    masses=[]
    for b,pdef in PARTS.items():
        v=pdef['inertia_raw_kg_m2'];c=result['mass']['COM_world_mm'][b]
        masses.append({'body':b,'mass_kg':pdef['mass_kg'],
          'cmd_com_x_mm':pdef['com_initial_global_mm'][0],
          'cmd_com_y_mm':pdef['com_initial_global_mm'][1],
          'cmd_com_z_mm':pdef['com_initial_global_mm'][2],
          'target_com_x_mm':c[0],'target_com_y_mm':c[1],'target_com_z_mm':c[2],
          **v,'mass_source_line':pdef['source_cmd_lines']['mass_inertia'],
          'com_marker_source_line':pdef['source_cmd_lines']['cm_marker']})
    write('20_mass_COM_inertia.csv',masses)
    rods=[]
    for name,info in result['active_rod_inputs'].items():
        rods.append({'actuator':name,**{k:fmt(v) for k,v in info.items()}})
    write('3_rod_endpoints.csv',rods)
    motion=[{'name':k,**{k1:fmt(v1) for k1,v1 in v.items()}} for k,v in MODEL['motions'].items()]
    write('10_motions.csv',motion)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['validate','solve','direct','joint','point','part','motion'])
    parser.add_argument('--hip-mm',type=float,default=0.)
    parser.add_argument('--knee-mm',type=float,default=0.)
    parser.add_argument('--ankle-mm',type=float,default=0.)
    parser.add_argument('--J0',type=float,default=0.)
    parser.add_argument('--J1',type=float,default=0.)
    parser.add_argument('--J2',type=float,default=NOM['J2'])
    parser.add_argument('--J3',type=float,default=NOM['J3'])
    parser.add_argument('--J4',type=float,default=NOM['J4'])
    parser.add_argument('--J5',type=float,default=0.)
    parser.add_argument('--toe',type=float,default=0.)
    parser.add_argument('--name',type=str,default='')
    parser.add_argument('--out',type=str,default='')
    parser.add_argument('--csv-dir',type=str,default='')
    a=parser.parse_args()
    if a.command=='validate':obj=run_validation()
    elif a.command=='solve':obj=strokes_to_pose(a.hip_mm,a.knee_mm,a.ankle_mm,a.J0,a.J1,a.J5,a.toe)
    elif a.command=='direct':obj=direct({'J0':a.J0,'J1':a.J1,'J2':a.J2,'J3':a.J3,'J4':a.J4,'J5':a.J5,'toe':a.toe})
    elif a.command=='point':obj=MODEL['hardpoints'].get(a.name)
    elif a.command=='joint':obj=MODEL['joints'].get(a.name)
    elif a.command=='part':obj=MODEL['parts'].get(a.name)
    elif a.command=='motion':obj=MODEL['motions'].get(a.name)
    else:raise ValueError(a.command)
    if a.out:Path(a.out).write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf-8')
    if a.csv_dir and a.command in ('solve','direct'):output_csv(obj,Path(a.csv_dir))
    if a.command in ('solve','direct'):
        short={'angles_deg':obj['controls']['angles_cmd_relative_deg'],
              'rod_lengths':{k:round(v['ball_center_length_mm'],6) for k,v in obj['active_rod_inputs'].items()},
              'COM':obj['mass']['aggregate_COM_world_mm'],
              'validation':obj['validation'], 'output':a.out or None}
        print(json.dumps(short,ensure_ascii=False,indent=2))
    else:print(json.dumps(obj,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
