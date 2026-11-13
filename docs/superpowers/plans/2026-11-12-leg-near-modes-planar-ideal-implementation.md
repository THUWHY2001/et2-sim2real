# ET2 Leg Near Modes and Planar Ideal Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 定位 `pin-revolute-v1` 三个近零模态，并交付独立 `pin-planar-ideal-v1` 的准静态理论载荷与虚功验收。

**Architecture:** 三维诊断仅消费现有运行时和静力矩阵，不改变模型。平面模型使用独立几何、运动学、平面静力文件和独立报告，最后在基准验收通过后生成单关节全时序结果。

**Tech Stack:** Node.js ESM、现有 rank-revealing SVD、JSON/CSV 报告、Node test。

**Spec:** `docs/superpowers/specs/2026-11-12-leg-near-modes-planar-ideal-design.md`

## Global Constraints

- 不修改原始 CMD、`legacy-ball`、`pin-revolute-v1` 及其既有报告。
- 不以放宽 SVD 阈值作为三维物理验收。
- 平面模型仅适用于 J5=0 和 X–Z 理想平面。
- 非唯一载荷输出 `null`；不开发摩擦圆。

## Review Focus

- 近零模态的符号/排序可能导致 J2/J3/J4 错配：用输出速度矩阵的最优一一映射测试。
- 刚体速度向量的参考点不一致会污染约束残差：统一使用各刚体质心速度和角速度。
- 移动副轴向理想化可能隐式修改杆长：报告基准长度差必须为零或明确列出。
- 平面静力可能超静定：逐分量零空间投影决定唯一性，不使用最小范数冒充。
- 全时序接近奇异姿态：逐样本记录长度雅可比行列式、条件数与失败点。

---

### Task 1: 三维近零模态与约束残差定位

**Files:**
- Create: `web2/src/leg-pin-modal-diagnostics.mjs`
- Create: `web2/tests/leg-pin-modal-diagnostics.test.mjs`
- Create: `web2/scripts/report-leg-pin-modal-diagnostics.mjs`
- Create: `web2/data/leg-full-package/pin_revolute_v1_modal_diagnostics_report.json`
- Create: `web2/data/leg-full-package/pin_revolute_v1_modal_diagnostics_report.csv`

**Interfaces:**
- Produces: `analyzePinNearModes(model,runtime,statics,boundary)`，含三个近零模态、有限差分速度、逐运动副残差和移动副几何审计。

- [ ] 先写失败测试，断言三个近零奇异值、谱间隙、三个模态与 J2/J3/J4 一一对应，并输出每个运动副残差排名。
- [ ] 运行测试确认因模块缺失而失败。
- [ ] 实现尺度一致的速度向量、被动约束 SVD 模态提取、有限差分和 `A_passive^T q` 分解。
- [ ] 生成 JSON/CSV，验证不改动三维模型与旧报告。
- [ ] 运行测试并提交。

### Task 2: 独立理想平面几何与运动学

**Files:**
- Create: `web2/src/leg-pin-planar-model.mjs`
- Create: `web2/src/leg-pin-planar-kinematics.mjs`
- Create: `web2/tests/leg-pin-planar-kinematics.test.mjs`

**Interfaces:**
- Produces: `createLegPinPlanarModel(legacy)`、`createLegPinPlanarKinematics(model)`；输出基准姿态、平面约束雅可比、三个自由度、推杆长度雅可比和理想化差异。

- [ ] 先写失败测试，断言模型隔离、Y 投影差异清单、严格三自由度和 3×3 长度雅可比满秩。
- [ ] 运行测试确认 RED。
- [ ] 实现二维刚体坐标、原硬点 X–Z 投影、销轴/移动副理想约束及姿态求解。
- [ ] 验证基准和 J2/J3/J4 各 ±10° 的闭链残差与雅可比条件数。
- [ ] 运行测试并提交。

### Task 3: 平面重力、推杆保持力和铰点反力

**Files:**
- Create: `web2/src/leg-pin-planar-statics.mjs`
- Create: `web2/tests/leg-pin-planar-statics.test.mjs`

**Interfaces:**
- Produces: `createLegPinPlanarStatics(model,kinematics)`；含势能梯度、推杆保持力、平面多刚体反力、唯一性和虚功结果。

- [ ] 先写失败测试，断言势能微分、长度雅可比满秩、推杆力唯一、虚功通过、平面平衡残差分单位合格。
- [ ] 运行测试确认 RED。
- [ ] 实现重力势能、`J_length^T F + tau_g=0`、平面刚体平衡矩阵和反力唯一性。
- [ ] 交叉验证输出力矩、推杆力和平面静力结果；非唯一反力保持 `null`。
- [ ] 运行测试并提交。

### Task 4: 基准报告与 1 Hz 全时序

**Files:**
- Create: `web2/scripts/report-leg-pin-planar-statics.mjs`
- Create: `web2/tests/leg-pin-planar-report.test.mjs`
- Create: `web2/data/leg-full-package/pin_planar_ideal_v1_report.json`
- Create: `web2/data/leg-full-package/pin_planar_ideal_v1_report.csv`

**Interfaces:**
- Consumes: Tasks 2–3 基准验收结果。
- Produces: 基准自由度、推杆力、铰点反力、虚功，以及 J2/J3/J4 单关节 1 Hz、±10° 时序。

- [ ] 先写失败测试，断言报告边界、排除项、基准验收门和每轴完整时序。
- [ ] 运行测试确认 RED。
- [ ] 仅在基准门通过后生成每轴至少 121 个采样点，记录姿态、重力矩、推杆力、雅可比条件数与反力状态。
- [ ] 运行报告测试、全套回归、旧文件隔离检查并提交。
