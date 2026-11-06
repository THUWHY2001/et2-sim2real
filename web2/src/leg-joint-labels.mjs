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
