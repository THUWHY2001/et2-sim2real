import {createCompleteLeftLeg} from "../data/leg-full-package/leg_full.mjs";

const OUTPUT_KEYS={J2:"J2_hip_pitch_deg",J3:"J3_knee_pitch_deg",J4:"J4_ankle_pitch_deg"};
const ROD_KEYS={J2:"hip",J3:"knee",J4:"ankle"};

export function createLegFullRuntime(model){
  const blackbox=createCompleteLeftLeg(model);
  const nominal={...blackbox.nominal};

  function normalize(frame){
    const outputs={
      J0:frame.controls.angles_cmd_relative_deg.J0,
      J1:frame.controls.angles_cmd_relative_deg.J1,
      J2:frame.passive_outputs.J2_hip_pitch_deg,
      J3:frame.passive_outputs.J3_knee_pitch_deg,
      J4:frame.passive_outputs.J4_ankle_pitch_deg,
      J5:frame.controls.angles_cmd_relative_deg.J5,
    };
    const motors=frame.controls.motor_inputs??{};
    return{
      complete:frame.validation.ok,
      frame,
      outputs,
      outputDelta:{J2:outputs.J2-nominal.J2,J3:outputs.J3-nominal.J3,J4:outputs.J4-nominal.J4},
      rodStrokes:{hip_mm:Number(motors.hip_rod_mm??frame.active_rod_inputs.hip.delta_vs_nominal_mm),knee_mm:Number(motors.knee_rod_mm??frame.active_rod_inputs.knee.delta_vs_nominal_mm),ankle_mm:Number(motors.ankle_rod_mm??frame.active_rod_inputs.ankle.delta_vs_nominal_mm)},
      residualMm:Math.max(...frame.joint_constraint_checks.map(item=>Number(item.constraint_residual_mm)||0)),
    };
  }

  function solveRods(inputs={}){return normalize(blackbox.solve(inputs))}

  function solveOutput(joint,deltaDegrees){
    if(!OUTPUT_KEYS[joint])throw new Error(`腿足完整闭链仅支持 J2/J3/J4 输出反解，收到 ${joint}`);
    const absolute=nominal[joint]+Number(deltaDegrees),angles={...nominal,[joint]:absolute};
    const direct=blackbox.directAngles(angles),rod=ROD_KEYS[joint],stroke=direct.active_rod_inputs[rod].delta_vs_nominal_mm;
    return solveRods({hip_mm:joint==="J2"?stroke:0,knee_mm:joint==="J3"?stroke:0,ankle_mm:joint==="J4"?stroke:0});
  }

  return Object.freeze({
    nominal:{...nominal},
    validate:()=>blackbox.validate(),
    solveRods,
    solveOutput,
    kinematics:solution=>solution?.frame??solution,
    blackbox,
  });
}
