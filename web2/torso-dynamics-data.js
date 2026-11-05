(()=>{
  'use strict';
  const body=(partName,massKg,comM,inertia,status='已知质量属性')=>({partName,massKg,comM,inertia,status});
  const explicitBodies=[
    body('waist_link1',.6323082,[-.0336342,.0000485,.0057914],{ixx:.0004819,iyy:.0007647,izz:.0007087,ixy:.0000005,izx:-.0001696,iyz:-.0000003}),
    body('waist_link2',.1287166,[-.0292621,.0000054,.0293479],{ixx:.000026,iyy:.0000151,izz:.0000347,ixy:0,izx:.0000002,iyz:0}),
    body('waist_link3',10.2,[-.0185177,-.0004286,.2350651],{ixx:.0929389,iyy:.0741128,izz:.0675952,ixy:.0002723,izx:.0050274,iyz:-.000371},'临时简化，待核实'),
    body('waist_link4',.0983414,[-.0392485,.0220821,.1006897],{ixx:.0000328,iyy:.0000862,izz:.000057,ixy:.0000017,izx:-.0000076,iyz:-.0000005}),
    body('waist_ball_link5',.0164401,[-.0684219,.01335,.1090301],{ixx:.0000006,iyy:.0000007,izz:.0000006,ixy:0,izx:0,iyz:0}),
    body('waist_link6',.0983414,[-.0392485,-.0220821,.1006897],{ixx:.0000328,iyy:.0000862,izz:.000057,ixy:-.0000017,izx:-.0000076,iyz:.0000005}),
    body('waist_ball_link7',.0164402,[-.0684219,-.01335,.1090301],{ixx:.0000006,iyy:.0000007,izz:.0000006,ixy:0,izx:0,iyz:0}),
    body('waist_link8',.060036,[-.073858,.0194839,.0750041],{ixx:.000047,iyy:.000048,izz:.0000051,ixy:-.0000006,izx:.0000028,iyz:-.0000085}),
    body('waist_link9',.0600356,[-.0738584,-.019484,.0750038],{ixx:.000047,iyy:.000048,izz:.0000051,ixy:.0000006,izx:.0000028,iyz:.0000085}),
  ];
  const mergedAttachments=[
    ['waist_link3_1_L_link',2700],['waist_link3_1_R_link',2700],['waist_link3_2',1810],
    ['waist_link3_3_shoulder_L',4183],['waist_link3_3_shoulder_R',4183],
    ['waist_link3_4_waist_L',6220],['waist_link3_4_waist_link',2700],['waist_link3_4_waist_R',6220],
  ].map(([partName,densityKgM3])=>({partName,densityKgM3,massKg:null,attachedTo:'waist_link3',status:'附属几何，不单独增加质量'}));
  const partMapping={
    waist_link1:'yaw',waist_link2:'roll',waist_link3:'platform',
    waist_link4:'crankL',waist_ball_link5:'crankL',
    waist_link6:'crankR',waist_ball_link7:'crankR',
    waist_link8:'rodL',waist_link9:'rodR',
  };
  window.TorsoDynamicsData={
    source:'ET2_base_2026_7_28_camp.cmd',
    gravity:[0,0,-9.80665],
    knownMassKg:11.3106595,
    explicitBodies,
    mergedAttachments,
    partMapping,
    assumptions:{defaultLoadFrictionCoefficientNmPerN:.001,defaultPreloadN:0,defaultLeftShare:.5},
  };
})();
