export const LEG_JOINT_KEYS=[
  "J_LEG_L_HIP_roll_RE","J_LEG_L_HIP_yaw_RE","J_LEG_L_HIP_pitch_RE","J_LEG_L_KNEE_pitch_RE","J_LEG_L_ANKLE_pitch_RE",
  "J_LEG_L_TOE_pitch_RE","J_LEG_L_P009_RE_L02_L08","J_LEG_L_P010_S_L09_L03","J_LEG_L_P011_S_L10_L03","J_LEG_L_HIP_pitch_T",
  "J_LEG_L_P013_F_L8_L12","J_LEG_L_P013_S_L12_L11","J_LEG_L_P013_RE_L08_L09","J_LEG_L_P014_S_L03_L13","J_LEG_L_KNEE_pitch_T",
  "J_LEG_L_P016_F_L16_B15","J_LEG_L_P016_S_B15_L14","J_LEG_L_P016_RE_L16_L17","J_LEG_L_P017_RE_L03_L16","J_LEG_L_P018_S_L17_L04",
  "J_LEG_L_P019_S_L18_L04","J_LEG_L_ANKLE_pitch_T","J_LEG_L_P021_F_L06_B20","J_LEG_L_P021_S_B20_L19","J_LEG_L_ANKLE_roll_RE"
];

const NAMES={
  J_LEG_L_HIP_roll_RE:"J0 髋 Roll 旋转副",J_LEG_L_HIP_yaw_RE:"J1 髋 Yaw 旋转副",J_LEG_L_HIP_pitch_RE:"J2 髋 Pitch 输出副",
  J_LEG_L_KNEE_pitch_RE:"J3 膝 Pitch 输出副",J_LEG_L_ANKLE_pitch_RE:"J4 踝 Pitch 输出副",J_LEG_L_TOE_pitch_RE:"趾 Pitch 旋转副",
  J_LEG_L_P009_RE_L02_L08:"髋摇臂上转动副",J_LEG_L_P010_S_L09_L03:"髋传动上球副",J_LEG_L_P011_S_L10_L03:"髋推杆上球副",
  J_LEG_L_HIP_pitch_T:"髋推杆移动副",J_LEG_L_P013_F_L8_L12:"髋球头固定副",J_LEG_L_P013_S_L12_L11:"髋推杆下球副",
  J_LEG_L_P013_RE_L08_L09:"髋摇臂下转动副",J_LEG_L_P014_S_L03_L13:"膝推杆上球副",J_LEG_L_KNEE_pitch_T:"膝推杆移动副",
  J_LEG_L_P016_F_L16_B15:"膝球头固定副",J_LEG_L_P016_S_B15_L14:"膝推杆下球副",J_LEG_L_P016_RE_L16_L17:"膝摇臂转动副",
  J_LEG_L_P017_RE_L03_L16:"膝上连杆转动副",J_LEG_L_P018_S_L17_L04:"膝下连杆球副",J_LEG_L_P019_S_L18_L04:"踝推杆上球副",
  J_LEG_L_ANKLE_pitch_T:"踝推杆移动副",J_LEG_L_P021_F_L06_B20:"踝球头固定副",J_LEG_L_P021_S_B20_L19:"踝推杆下球副",
  J_LEG_L_ANKLE_roll_RE:"J5 踝 Roll 旋转副"
};

const TYPE_LABELS={revolute:"旋转副",spherical:"球副",translational:"移动副",fixed:"固定副"};

export function createLegJointCatalog(model){
  const missing=LEG_JOINT_KEYS.filter(key=>!model.joints[key]);
  if(missing.length)throw new Error(`Missing leg joints: ${missing.join(", ")}`);
  const entries=LEG_JOINT_KEYS.map((key,index)=>{
    const joint=model.joints[key],code=`L${String(index+1).padStart(2,"0")}`,name=NAMES[key]||key;
    return{key,code,name,display:`${code} ${name}`,type:joint.type,typeLabel:TYPE_LABELS[joint.type]||joint.type,connection:`${joint.i_part} ↔ ${joint.j_part}`,iPart:joint.i_part,jPart:joint.j_part};
  });
  const byKey=Object.fromEntries(entries.map(item=>[item.key,item]));
  return{entries,meta(key){const item=byKey[key];if(!item)throw new Error(`Unknown leg joint: ${key}`);return item}};
}

export function layoutLegJointLabels(items,minDistance=12){
  const offsets=[[7,-7]];
  for(let radius=14;radius<=70;radius+=14)for(let index=0;index<8;index++){const angle=index*Math.PI/4;offsets.push([7+Math.cos(angle)*radius,-7+Math.sin(angle)*radius])}
  const placed=[];
  for(const item of items){
    const offset=offsets.find(([dx,dy])=>placed.every(other=>Math.hypot(item.x+dx-other.labelX,item.y+dy-other.labelY)>=minDistance))||offsets[offsets.length-1];
    placed.push({...item,labelX:item.x+offset[0],labelY:item.y+offset[1]});
  }
  return placed;
}

export function buildLegBodySegments(markers){
  const groups={};
  for(const marker of markers){
    const key=marker.xyz_mm.map(value=>Number(value).toFixed(6)).join(",");
    const group=groups[marker.part]??=(new Map());
    if(!group.has(key))group.set(key,marker.xyz_mm.map(Number));
  }
  const segments=[];
  for(const [part,pointMap] of Object.entries(groups)){
    const points=[...pointMap.values()];
    if(points.length<2)continue;
    const connected=new Set([0]),remaining=new Set(points.slice(1).map((_,index)=>index+1));
    while(remaining.size){
      let best=null;
      for(const from of connected)for(const to of remaining){
        const distance=Math.hypot(...points[from].map((value,axis)=>value-points[to][axis]));
        if(!best||distance<best.distance)best={from,to,distance};
      }
      segments.push({part,a:points[best.from],b:points[best.to]});
      connected.add(best.to);remaining.delete(best.to);
    }
  }
  return segments;
}
