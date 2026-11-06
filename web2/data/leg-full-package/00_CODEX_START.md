# ET2 Left Leg - FULL Closed Chain / Motion / Mass / Inertia

Read this file first, not the entire `model.json` or 70k-line solved JSON. This package is **only the left leg**, from `ET2_base_2026_7_28_camp.cmd` (2026-07-28). It includes the 20 original structural Points P2-P21, 25 Joints with 50 I/J Marker sides, 10 Motion definitions, 20 body masses/COM/inertias, three true rod-driven closed loops, and the validated user nominal pose.

## Important: The three linear rods MUST be the active inputs

- Active rotary motors: `MOTION_HIP_L_roll` (J0), `MOTION_HIP_L_yaw` (J1), and `MOTION_ANKLE_L_roll` (J5).
- Active linear actuators: `MOTION_HIP_L_pitch_T` (hip rod), `MOTION_KNEE_L_pitch_T` (knee rod), `MOTION_ANKLE_L_pitch_T` (ankle rod).
- Passive rotational output constraints (retain the REVOLUTE JOINTS but turn OFF their direct MOTION drive): `J_LEG_L_HIP_pitch_RE` -> J2, `J_LEG_L_KNEE_pitch_RE` -> J3, `J_LEG_L_ANKLE_pitch_RE` -> J4.
- Source CMD originally has all three rod Motions **OFF**, the hip and knee output Motions ON, and ankle pitch output Motion with no explicit active field. Do not simply reuse that simulated controller configuration to represent a hardware rod drive.
- A separate Toe pitch joint and CMD Motion exist (ON), outside J0-J5. Whether real hardware has an independently active toe motor is **unverified**. Benchmark uses Toe=0. Retain its constraint and define explicit hold or control strategy before dynamics.

## User-specified nominal pose (treated as a FIXED reference)

Source CMD initial geometry is the zero of this package's right-hand **child-body relative** angles. With `base_link` fixed, the user nominal pose is:

```
J0 = 0, J1 = 0, J2 = -27.50 deg, J3 = +51.57 deg,
J4 = -24.06 deg, J5 = 0, Toe = 0
```

Do not assume this right-hand sign convention equals ADAMS I/J signed Motion, or a physical joint encoder sign/offset. **Do not change mass or original hardpoints when setting the nominal pose.** Transforms live in the pose output.

Actuator lengths in original CMD q=0: HIP 321.134228912 mm, KNEE 332.536178905 mm, ANKLE 262.976680268 mm.

Actuator lengths at the user nominal pose: HIP **305.840058743 mm**, KNEE **304.121089855 mm**, ANKLE **281.003620419 mm**.

`hip_mm`, `knee_mm`, `ankle_mm` denote **PHYSICAL EXTENSION relative to these three nominal end-to-end ball-center spans**, with + sign = longer. This is not the ADAMS prismatic I/J Marker signed displacement. Distinguish these in your app and log.

## Exact source-connected chains and point ownership

1. **HIP pitch**: leg_l_link2--P9 revolute--leg_l_link8--P13 revolute--leg_l_link9--P10 spherical--leg_l_link3; its rod: leg_l_link3--P11 spherical--leg_l_link10 (housing)--P12 translational--leg_l_link11 (shaft)--P13 spherical via leg_l_ball_link12 fixed to rocker. J2 output revolves about P4.
2. **KNEE pitch**: leg_l_link3--P17 revolute--leg_l_link16 rocker--P16 revolute--leg_l_link17--P18 spherical--leg_l_link4; its rod: leg_l_link3--P14 spherical--leg_l_link13 (housing)--P15 translational--leg_l_link14 (shaft)--P16 spherical via leg_l_ball_link15 fixed to rocker. J3 output revolves about P5.
3. **ANKLE pitch**: leg_l_link4--P19 spherical--leg_l_link18 (housing)--P20 translational--leg_l_link19 (shaft)--P21 spherical via leg_l_ball_link20, fixed to leg_l_link6. The output J4 revolves about P6, and leg_l_link6 is **after the J5 ankle roll joint about P7**. Thus ankle roll can change the ankle rod span and drive a passive J4 compensation even for unchanged ankle rod length.

**Every material body moves**: link8/9/10/11, link13/14, link16/17, link18/19, all ball_link12/15/20, the main links1-7. Use `world_transforms` in the result to update every rigid part. Do NOT merely rotate equivalent J2/J3/J4 revolute markers while the rods remain static.

## First run these two independent validators

```sh
python leg_full.py validate
node --input-type=module -e "import fs from 'node:fs';import {createCompleteLeftLeg} from './leg_full.mjs';const d=JSON.parse(fs.readFileSync('model.json'));console.log(createCompleteLeftLeg(d).validate())"
```

### Python: drive actual rod lengths; do not directly drive J2/J3/J4

```sh
python leg_full.py solve --hip-mm 1 --knee-mm 1 --ankle-mm 1 --out solved.json --csv-dir result_csv
python leg_full.py solve --J5 5 --out ankle_roll_coupling.json
```

For debugging ONLY, `python leg_full.py direct --J2 -27.5 --J3 51.57 --J4 -24.06` computes rod lengths from target output angles. Production animation should call `solve` from real actuator inputs.

### Browser / Node

```javascript
import {createCompleteLeftLeg} from './leg_full.mjs';
const leg = createCompleteLeftLeg(modelJson);
const frame = leg.solve({hip_mm: 1, knee_mm: 1, ankle_mm: 1});
console.log(frame.passive_outputs, frame.validation);
```

`leg.solve` returns `world_transforms` (21 entries incl. fixed base_link), `all_hardpoints` (23 records; P12/P15/P20 both I/J), `all_joint_marker_sides` (50 records), `joint_centers` (J0-J5 and Toe), `joint_constraint_checks` (25 entries), `active_rod_inputs` (three physical spans and anchor positions), and `mass` (20 COM world positions, original raw inertia and per-body rotated inertia matrices).

## Validation reference cases

- All rod strokes 0 -> J2=-27.500000 deg, J3=+51.570000 deg, J4=-24.060000 deg.
- HIP/KNEE/ANKLE strokes (+1,+1,+1) mm -> J2=-25.932618659 deg, J3=+50.052188561 deg, J4=-25.530447752 deg.
- At rod strokes all 0 but J5 motor +5 deg, J4 changes to approximately -23.842537 deg to preserve the ankle rod span (cross-coupling, not independent motor command).
- All 22 non-prismatic joint point pairs are geometric-zero within ~1e-12 mm; prismatic I/J markers separate AXIALLY by the commanded slide, as required.
- Maximum prismatic TRANSVERSE residual in nominal: **0.000616802 mm**, because CMD Point/Marker data have rounded coordinates. We accept <0.002 mm, never call this literally zero. Axis alignment residual <2e-6.
- Python and JS have been compared across multiple cases including hip roll, yaw, ankle roll and toe; max discrepancy across numeric output fields <6e-12 (in matching units).
- The mass of 20 leg_l_* bodies is exactly **13.02480932 kg**; all have nonzero mass and 6 raw inertia entries; source inertia tensors tested positive definite. Aggregate initial CMD COM (-6.551619,93.208510,-428.402901) mm; user nominal aggregate COM (78.512458,93.208510,-399.063421) mm. `base_link` is only a kinematic interface; its mass is excluded. Gravity world (0,0,-9.80665) m/s^2 is configured ONCE.

## Offline visual acceptance

Open `ET2_left_leg_actuated_closedchain_viewer.html` directly in Edge / Chrome. No web requests or server required. Adjust the three physical rod sliders and inspect the full 20 structural Points (23 records after split) and 50 Marker positions. All bodies / joints remain constrained. Also inspect the two PNG diagrams.

## Limitations and source cautions

- **Not** a full ADAMS solve or motor force prediction. No real actuator stroke limits, hard stops, friction, CAD collision meshes, and dynamic/contact reaction forces are included.
- All inertia markers in the 7/28 source have zero local Euler angles; raw inertia off-diagonals are preserved as named in the CMD. Confirm product-of-inertia SIGN convention in your target physics engine before importing raw tensor entries.
- July CMD retains ONE ankle roll revolute; a later September drawing revises front/rear ankle support. Do not silently merge those versions.
- `model.json` preserves original Part/Joint/Motion source CMD line numbers. It contains the 20 defined connection Points, not all CAD surface vertices.
