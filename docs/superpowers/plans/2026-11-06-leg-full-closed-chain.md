# 腿足完整推杆闭链模型 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用新建模包的三推杆主动、J2/J3/J4被动完整闭链模型替换腿足三页旧简化模型，并补齐加载贡献度和闭链周期曲线。

**Architecture:** 保留建模包 `leg_full.mjs` 作为几何求解真值源，在项目运行时适配层中统一输出姿态、20刚体质心、连接副传动率、重力/惯性及加载修正。J2/J3/J4共用一个页面片段，输出端±10°通过反解对应推杆物理行程，再由推杆正解回代动画与残差。

**Tech Stack:** 原生 JavaScript ES modules、HTML/SVG、Node.js 回归测试、GitHub Pages。

**Spec:** `web2/data/leg-full-package/00_CODEX_START.md`

## Global Constraints

- J2/J3/J4不得直接作为动画驱动；生产路径必须由三根物理推杆行程正解。
- 基准姿态固定为 J2=-27.5°、J3=51.57°、J4=-24.06°，三推杆行程增量为0。
- 每帧使用包返回的20运动刚体、23硬点记录、50 Marker及20刚体质量属性。
- 保留J5对J4/踝推杆的闭链耦合，不添加无源数据支持的行程限位或约束。
- 仅使用左腿20刚体质量，总质量13.02480932 kg，重力只施加一次。
- 加载系数默认0；基础贡献度、加载后贡献度和总力矩分开显示。

## Review Focus

- 三推杆0行程必须精确回到用户基准姿态。
- 三推杆各+1 mm必须复现包内验收角度与小于0.002 mm横向残差。
- J5变化且踝推杆不变时J4必须自动补偿，不得锁死。
- 页面±10°输出反解后必须再次走推杆正解，残差失败时不显示排名。
- 质量、质心和惯量必须随各自刚体变换，不能集中到等效轴。

---

### Task 1: 接入并验证完整闭链黑盒

**Files:**
- Create: `web2/src/leg-full-runtime.mjs`
- Copy/consume: `web2/data/leg-full-package/leg_full.mjs`, `model.json`
- Test: `web2/tests/leg-full-runtime.test.mjs`

**Interfaces:**
- Produces: `createLegFullRuntime(model)` with `validate()`, `solveRods(inputs)`, `solveOutput(joint, deltaDeg)`, `kinematics(frame)`, `cycle(joint, ...)`.

- [ ] Write failing tests for nominal, +1/+1/+1 mm, J5 coupling, 20 bodies, 23 hardpoints, 50 marker sides and residual thresholds.
- [ ] Run the test and confirm missing runtime failure.
- [ ] Implement the adapter without duplicating the package solver.
- [ ] Run the test and expect PASS.

### Task 2: 动力学、连接副阻力与加载修正

**Files:**
- Modify: `web2/src/leg-full-runtime.mjs`
- Test: `web2/tests/leg-full-dynamics.test.mjs`

**Interfaces:**
- Produces: `rates`, `equivalentResistance`, `nominalLoads`, `loadedResistance`, `gravityHoldingTorque`, `effectiveInertia`, `trajectoryDynamics`, `loadedCycle`.

- [ ] Write failing tests for finite rates, 13.02480932 kg scope, zero load coefficient identity, single-connection perturbation isolation and finite gravity/inertia.
- [ ] Implement numerical derivatives from the pushrod-driven closed-chain solution and body COM/inertia states.
- [ ] Run tests and expect PASS.

### Task 3: 共享J2/J3/J4页面与全机构动画

**Files:**
- Create: `web2/src/leg-full-joint.fragment.html`
- Modify: `web2/site.css`
- Test: `web2/tests/leg-full-pages.test.mjs`

**Interfaces:**
- Consumes: Tasks 1–2 runtime.
- Produces: 3D full mechanism view, physical rod input readout, residual readout, base/live/cycle loaded contribution sections and total torque.

- [ ] Write failing structural tests for full-model labels, 20-body/23-point/50-marker status, and loaded modules.
- [ ] Implement one shared page fragment with output-to-rod inverse command followed by rod-driven forward solve every frame.
- [ ] Render all hardpoints, rods, rockers, ball ends and COM positions from frame data.
- [ ] Run page tests and expect PASS.

### Task 4: 闭链周期曲线、生成和发布

**Files:**
- Modify: `web2/src/leg-full-joint.fragment.html`
- Modify: `web2/scripts/build-pages.mjs`
- Regenerate: `web2/j2.html`, `web2/j3.html`, `web2/j4.html`
- Test: all `web2/tests/*.test.mjs`

**Interfaces:**
- Produces: output angle, three physical rod strokes and J5-coupled J4 series on the 1 Hz chart.

- [ ] Add failing tests for the four curve series and generated pages.
- [ ] Implement SVG cycle chart from solved frame samples.
- [ ] Replace the three old fragments in the generator, copy runtime/model assets and bump the leg cache version.
- [ ] Run package Python and Node validators, all repository tests, and browser-check J2/J3/J4.
- [ ] Commit, push and verify GitHub Pages.
