# ET2 左臂黑盒模型集成 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用 `arm_model.zip` 覆盖现有手臂简化模型，发布统一的 J0–J6 左臂闭链运动学与初步动力学分析页面。

**Architecture:** 原样机械复制 `model.json`、`pose_reference.json` 与黑盒模块，新增 `arm-runtime.mjs` 作为唯一页面和动力学接口。运行时复用黑盒肘/腕正逆解，按 CMD 主链刚体变换绑定18个 COM/惯量，以数值虚功计算重力矩和有效惯量。

**Tech Stack:** 原生 ES modules、SVG、Node.js 回归测试、GitHub Pages。

**Spec:** `web2/docs/superpowers/specs/2026-11-05-left-arm-blackbox-integration-design.md`

## Global Constraints

- 首先调用黑盒 `validate()`，不得全文读取或重写 `model.json`。
- 只使用18个左臂刚体，总质量 `3.7249532 kg`；手部及胸腰接口质量排除。
- 仅使用2026-07-28 CMD版本；9月 PDF差异不得混入。
- J3/J5/J6输出轴为被动输出，唯一主动输入分别为肘单推杆和腕部双推杆。
- 重力 `[0,0,-9.80665] m/s²` 只施加一次。

## Review Focus

- J3/J5/J6在±10°边界必须正逆解回代收敛，失败时页面不能沿用上一帧结果。
- J5和J6分别激励时，双推杆行程组合必须不同且回代得到目标输出。
- 18个COM必须随所属刚体变化，手部和胸腰接口不能进入质量合计。
- J1=+20°、J3=-20°参考姿态的J0–J6中心必须通过固定数值回归。
- 换向点速度为零时阻力方向、惯性项和贡献度不得出现 NaN。

---

### Task 1: 黑盒资产与统一运行时

**Files:**
- Create: `web2/data/arm/model.json`
- Create: `web2/data/arm/pose_reference.json`
- Create: `web2/src/left-arm-blackbox.mjs`
- Create: `web2/src/arm-runtime.mjs`
- Create: `web2/tests/arm-blackbox-runtime.test.mjs`
- Modify: `web2/scripts/build-pages.mjs`

**Interfaces:**
- Produces: `createArmRuntime(model, poseReference)`，包含 `validate()`、`solveOutput(joint,degrees)`、`solveActive(inputs)`、`summary()`。

- [ ] **Step 1: Write the failing runtime test** for validation, 18 bodies, mass `3.7249532`, J3/J5/J6 ±10° roundtrip residual `<1e-6 mm`, active-mode mapping and excluded interface masses.
- [ ] **Step 2: Run `node web2/tests/arm-blackbox-runtime.test.mjs`; expect RED because runtime/assets are absent.**
- [ ] **Step 3: Mechanically copy package assets and implement the adapter around `createLeftArmBlackbox(model)` without duplicating its solvers.**
- [ ] **Step 4: Build and run the new test plus existing arm tests; expect GREEN.**
- [ ] **Step 5: Commit `Add validated left-arm blackbox runtime`.**

### Task 2: 刚体姿态、质量与广义动力学

**Files:**
- Modify: `web2/src/left-arm-blackbox.mjs`
- Modify: `web2/src/arm-runtime.mjs`
- Create: `web2/tests/arm-dynamics-runtime.test.mjs`

**Interfaces:**
- Consumes: Task 1 runtime.
- Produces: `kinematics()`、`bodyPoses()`、`rates()`、`equivalentResistance()`、`gravityHoldingTorque()`、`effectiveInertia()`、`trajectoryDynamics()`、`cycle()`。

- [ ] **Step 1: Write failing tests** for 18 moving COMs, exact total mass, finite gravity/effective inertia, 100% contribution sums, cycle closure, zero-point finiteness and J1+20/J3−20 reference joint centers.
- [ ] **Step 2: Run the test; expect RED for missing dynamics APIs.**
- [ ] **Step 3: Expose read-only blackbox geometry snapshots, implement main-chain and closed-chain rigid-body poses, numerical rates, gravity and inertia.**
- [ ] **Step 4: Run new and all arm runtime/model tests; expect GREEN.**
- [ ] **Step 5: Commit `Calculate left-arm mass and torque dynamics`.**

### Task 3: 统一覆盖 J0–J6 页面

**Files:**
- Create: `web2/src/arm-joint.fragment.html`
- Modify: `web2/scripts/build-pages.mjs`
- Modify: `web2/site.css`
- Create: `web2/tests/arm-unified-pages.test.mjs`
- Replace generated: `web2/arm-j0.html` through `web2/arm-j6.html`

**Interfaces:**
- Consumes: Task 2 runtime.
- Produces: seven uniform pages and J0–J6 navigation.

- [ ] **Step 1: Write failing page tests** for all seven pages, active/passive labels, rod readouts, residual, friction inputs, live/cycle contributions, total torque, 18-body mass table and version warning.
- [ ] **Step 2: Build and run the page test; expect RED because J0/J1/J2/J4 and unified structure are absent.**
- [ ] **Step 3: Implement the shared page template, update navigation/home, and remove old page-model dependencies from generated J3/J5/J6.**
- [ ] **Step 4: Build and run page/runtime tests; open J0–J6 locally and verify animation and controls.**
- [ ] **Step 5: Commit `Replace arm pages with unified blackbox model`.**

### Task 4: 全量验证与发布

- [ ] **Step 1: Run every root and `web2/tests/*.test.mjs`; require zero failures.**
- [ ] **Step 2: Verify root and web2 arm navigation enter J0 and retain J0–J6 tabs.**
- [ ] **Step 3: Verify files outside the arm/web2 build scope are unchanged except generated navigation.**
- [ ] **Step 4: Merge, push `main`, and verify GitHub Pages serves J0–J6.**

