import fs from "node:fs";
import {createLegPinRuntime} from "../src/leg-pin-runtime.mjs";
import {PIN_JOINT_MAPPINGS} from "../src/leg-pin-model.mjs";

const packageUrl=new URL("../data/leg-full-package/",import.meta.url);
const legacy=JSON.parse(fs.readFileSync(new URL("model.json",packageUrl),"utf8"));
const runtime=createLegPinRuntime(legacy),model=runtime.model;
const tolerances={pin_position_mm:2e-5,pin_axis_line_angle_deg:2e-6,translational_lateral_mm:1e-3,translational_axis_cross_error:1e-8};
const translationalIds=Object.entries(model.joints).filter(([,item])=>item.type==="translational").map(([id])=>id);

function sampleSummary(solution,deltaDeg){
  const pins=solution.frame.pin_constraint_checks.map(item=>({joint:item.joint,hardpoint:model.joints[item.joint].hardpoint,position_residual_mm:item.position_residual_mm,axis_line_angle_deg:item.axis_line_angle_deg,compatible:item.compatible}));
  const translations=translationalIds.map(joint=>{const item=solution.frame.joint_constraint_checks.find(check=>check.joint===joint);return{joint,lateral_gap_mm:item.lateral_gap_mm,axis_cross_error:item.axis_cross_error,compatible:item.lateral_gap_mm<=tolerances.translational_lateral_mm&&item.axis_cross_error<=tolerances.translational_axis_cross_error}});
  return{delta_deg:deltaDeg,compatible:pins.every(item=>item.compatible)&&translations.every(item=>item.compatible),pin_checks:pins,translational_checks:translations};
}

const nominalSolution=runtime.solveOutput("J2",0),nominal=sampleSummary(nominalSolution,0),scans={};
for(const joint of ["J2","J3","J4"]){
  const samples=[];
  for(let index=0;index<=120;index++){const delta=-10+20*index/120;samples.push(sampleSummary(runtime.solveOutput(joint,delta),delta))}
  scans[joint]={samples,compatible:samples.every(item=>item.compatible),max_pin_position_residual_mm:Math.max(...samples.flatMap(item=>item.pin_checks.map(check=>check.position_residual_mm))),max_pin_axis_angle_deg:Math.max(...samples.flatMap(item=>item.pin_checks.map(check=>check.axis_line_angle_deg))),max_translational_lateral_mm:Math.max(...samples.flatMap(item=>item.translational_checks.map(check=>check.lateral_gap_mm)))};
}

function j5Conflict(delta){
  const solution=runtime.solveRods({J5_deg:delta}),byPoint=Object.fromEntries(solution.frame.pin_constraint_checks.map(item=>[model.joints[item.joint].hardpoint,item]));
  return{J5_deg:delta,compatible:solution.frame.pin_constraint_checks.every(item=>item.compatible),P19_axis_angle_deg:byPoint.P19.axis_line_angle_deg,P21_axis_angle_deg:byPoint.P21.axis_line_angle_deg,P19_position_residual_mm:byPoint.P19.position_residual_mm,P21_position_residual_mm:byPoint.P21.position_residual_mm,pin_checks:solution.frame.pin_constraint_checks};
}

const j5ConflictCases={minus_5_deg:j5Conflict(-5),plus_5_deg:j5Conflict(5)},j5ZeroPassed=nominal.compatible&&Object.values(scans).every(item=>item.compatible),fullLegValid=j5ZeroPassed&&Object.values(j5ConflictCases).every(item=>item.compatible);
const mapping=PIN_JOINT_MAPPINGS.map(item=>{const joint=model.joints[item.pin_joint_id];return{...item,i_part:joint.i_part,j_part:joint.j_part,i_axis_local:joint.i_marker_axis_local,j_axis_local:joint.j_marker_axis_local,nominal_axis_world:joint.assumed_axis_world_nominal,axis_source:joint.axis_source}});
const report={generated_at:new Date().toISOString(),model_id:model.meta.model_id,legacy_model_id:model.meta.parent_model_id,legacy_model_unchanged:true,axis_assumption:{world_axis:"+Y",status:"基于现有几何的轴向假设，待最新 CAD 销轴轴线复核",sign_note:"+Y/-Y 仅影响有符号角度，不改变物理轴线",local_axis_method:"axis_local = R_body_nominal^T * [0,1,0]，I/J 两侧独立计算与验证"},tolerances,mapping,nominal,j5_zero_scans:scans,j5_conflict:j5ConflictCases,stage_gate:{j5_zero_passed:j5ZeroPassed,proceed_to_statics:j5ZeroPassed,reason:j5ZeroPassed?"J5=0 基准及 J2/J3/J4 各 ±10° 全部通过":"J5=0 几何兼容性失败，禁止进入静力计算"},full_leg_motion_valid:fullLegValid,valid_scope:j5ZeroPassed?{J5_deg:0,J2_delta_deg:[-10,10],J3_delta_deg:[-10,10],J4_delta_deg:[-10,10]}:null,conclusion:fullLegValid?"完整测试范围内兼容":j5ZeroPassed?"仅 J5=0、J2/J3/J4 各 ±10° 范围通过；J5 非零与单自由度销轴闭链冲突":"J5=0 范围不兼容"};

fs.writeFileSync(new URL("pin_revolute_v1_kinematics_report.json",packageUrl),JSON.stringify(report,null,2)+"\n");
const rows=[["case","joint","delta_deg","compatible","max_pin_position_mm","max_pin_axis_angle_deg","max_translational_lateral_mm","P19_axis_angle_deg","P21_axis_angle_deg"]];
for(const [joint,scan] of Object.entries(scans))for(const sample of scan.samples)rows.push(["J5=0 sweep",joint,sample.delta_deg,sample.compatible,Math.max(...sample.pin_checks.map(item=>item.position_residual_mm)),Math.max(...sample.pin_checks.map(item=>item.axis_line_angle_deg)),Math.max(...sample.translational_checks.map(item=>item.lateral_gap_mm)),"",""]);
for(const item of Object.values(j5ConflictCases))rows.push(["J5 conflict","J5",item.J5_deg,item.compatible,Math.max(item.P19_position_residual_mm,item.P21_position_residual_mm),Math.max(item.P19_axis_angle_deg,item.P21_axis_angle_deg),"",item.P19_axis_angle_deg,item.P21_axis_angle_deg]);
fs.writeFileSync(new URL("pin_revolute_v1_kinematics_report.csv",packageUrl),rows.map(row=>row.join(",")).join("\n")+"\n");
console.log(JSON.stringify({model_id:report.model_id,stage_gate:report.stage_gate,full_leg_motion_valid:report.full_leg_motion_valid,j5_conflict:{minus_5_deg:{P19:itemValue("minus_5_deg","P19_axis_angle_deg"),P21:itemValue("minus_5_deg","P21_axis_angle_deg")},plus_5_deg:{P19:itemValue("plus_5_deg","P19_axis_angle_deg"),P21:itemValue("plus_5_deg","P21_axis_angle_deg")}}},null,2));
function itemValue(caseName,key){return report.j5_conflict[caseName][key]}
