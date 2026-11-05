# ET2 头颈黑盒模型集成 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用头颈黑盒包替换现有简化头颈模型，建立推杆主动、Roll/Pitch 被动输出、Yaw 主动的三自由度闭链页面，并加入与躯干一致的阻力、负载和总力矩分析。

**Architecture:** 原样保留 `model.json` 与 `neck_blackbox.mjs`，新增 `neck-runtime.mjs` 作为动力学和页面统一接口。J0/J1 输出激励先由 `poseToRods()` 得到真实推杆行程，再用 `rodsToPose()`回代校核；组合输入直接调用 `rodsToPose()`。页面只消费运行时接口，不复制闭链公式。

**Tech Stack:** 原生 ES modules、SVG、Node.js 回归测试、GitHub Pages。

**Spec:** `web2/docs/superpowers/specs/2026-11-05-neck-blackbox-integration-design.md`

## Global Constraints

- 首先调用 `validate()`；不得全文读取或重写黑盒求解器。
- 仅使用头颈 13 刚体，总质量 `2.8109963 kg`；不计 `waist_link3` / `waist_link3_2`。
- 全局重力 `[0,0,-9.80665] m/s²` 只施加一次。
- J0/J1 不得直接作为独立驱动；主动输入仅为左右推杆行程与 Yaw。
- 保留现有硬点公差页面，覆盖原 J0/J1 主页面并新增 J2 与组合页。

## Review Focus

- 输出角接近 `±10°` 时反解行程仍须回代收敛并保持同一运动学分支。
- 推杆行程超出求解域时页面应明确报错，不得沿用上一帧几何。
- Yaw 改变时两推杆行程及相对 Roll/Pitch 必须保持不变。
- 传感器 4 刚体必须随头部刚体运动，不能停留在初始质心。
- 运行阻力换向点速度为零时不得产生 NaN 或错误方向突变。

---

### Task 1: 黑盒资产与验收接口

**Files:**
- Create: `web2/data/neck/model.json`
- Create: `web2/src/neck-blackbox.mjs`
- Create: `web2/src/neck-runtime.mjs`
- Create: `web2/tests/neck-blackbox-runtime.test.mjs`
- Modify: `web2/scripts/build-pages.mjs`

**Interfaces:**
- Produces: `createNeckRuntime(data)`；返回 `validate()`、`solveInputs({leftMm,rightMm,yawDeg})`、`solveOutput(joint,degrees)`、`summary()`。

- [ ] **Step 1: Write the failing runtime acceptance test** asserting mass `2.8109963`, `+1/+1 → Pitch 2.569408°`, `+1/-1 → Roll 1.849923° / Pitch 0.009610°`, Yaw invariance and residual `<1e-9 mm`.
- [ ] **Step 2: Run `node web2/tests/neck-blackbox-runtime.test.mjs` and confirm RED because `neck-runtime.mjs` is absent.**
- [ ] **Step 3: Copy the two black-box assets mechanically and implement `createNeckRuntime(data)` only as an adapter around `makeNeck(data)`.**
- [ ] **Step 4: Run the test and confirm GREEN; run existing neck tests.**
- [ ] **Step 5: Commit `Add validated neck blackbox runtime`.**

### Task 2: 刚体姿态、重力和阻力折算

**Files:**
- Modify: `web2/src/neck-runtime.mjs`
- Create: `web2/tests/neck-dynamics-runtime.test.mjs`

**Interfaces:**
- Consumes: Task 1 runtime.
- Produces: `bodyPoses(solution)`、`rates(joint,degrees)`、`equivalentResistance(...)`、`loadedResistance(...)`、`trajectoryDynamics(...)`、`cycle(...)`。

- [ ] **Step 1: Write failing tests** for 13 moving COMs, summed mass, finite positive effective inertia, finite gravity torque, contribution sums of 100%, load-before/load-after comparison and 1 Hz cycle closure.
- [ ] **Step 2: Run the new test and confirm RED for missing dynamics methods.**
- [ ] **Step 3: Build rigid-body poses from queried PART/joint attachments; bind each initial world COM and raw COM inertia to its PART. Use numerical virtual work for rates, gravity torque and effective inertia.**
- [ ] **Step 4: Run new and existing neck tests; confirm GREEN.**
- [ ] **Step 5: Commit `Calculate neck load and torque contributions`.**

### Task 3: 覆盖头颈页面并新增 J2/组合页

**Files:**
- Replace: `web2/src/neck-joint.fragment.html`
- Create: `web2/src/neck-combined.fragment.html`
- Modify: `web2/src/neck-3d-view.js`
- Modify: `web2/site.css`
- Modify: `web2/scripts/build-pages.mjs`
- Modify: `web2/tests/neck-pages.test.mjs`

**Interfaces:**
- Consumes: Task 2 runtime.
- Produces: `neck-j0.html`、`neck-j1.html`、`neck-j2.html`、`neck-combined.html`。

- [ ] **Step 1: Write failing page tests** for three active inputs, passive-output labels, residual/readout fields, original and loaded contribution modules, total torque cards, mass table, J2 and combined navigation.
- [ ] **Step 2: Run `node web2/tests/neck-pages.test.mjs` and confirm RED for missing new structure.**
- [ ] **Step 3: Replace J0/J1 main pages with the torso layout, add J2 and combined pages, and render all hardpoints/rods/axes from runtime solutions.**
- [ ] **Step 4: Build pages and run page/model tests; confirm GREEN.**
- [ ] **Step 5: Commit `Replace neck pages with blackbox closed-chain model`.**

### Task 4: 全量验证与发布

**Files:**
- Modify only if verification exposes a tested defect.

**Interfaces:**
- Consumes: Tasks 1–3 complete site.
- Produces: published `web2` head-neck module.

- [ ] **Step 1: Run every `web2/tests/*.test.mjs`; require zero failures.**
- [ ] **Step 2: Build and open J0/J1/J2/组合页; verify initialization, 3D motion, input edits, cycle calculation, mobile and desktop layout.**
- [ ] **Step 3: Verify files outside `web2` public site are unchanged and root webpage1 is untouched.**
- [ ] **Step 4: Commit any tested cache-version fix, push `main`, and verify GitHub Pages serves the new version.**
