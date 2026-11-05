# ET2 躯干负载耦合阻力贡献度实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在网页2中加入 ET2 躯干质量属性、重力、惯性和加载前后连接副阻力贡献度比较。

**Architecture:** 保留 `torso-model.js` 的闭链解算接口，在其上增加 PART 刚体姿态、重力势能、等效惯量和连接副名义承载计算。原阻力贡献度作为基准，加载后贡献度使用同一姿态雅可比和可编辑载荷—摩擦参数，页面同时显示变化量和排名稳定性。

**Tech Stack:** 静态 HTML/CSS/JavaScript、Node.js ESM 测试、GitHub Pages。

**Spec:** `web2/docs/superpowers/specs/2026-11-05-torso-load-coupled-dynamics-design.md`

## Global Constraints

- 只修改 `web2/`；网页1不得变化。
- 保持现有闭链硬点、约束和三个真实主动轴不变。
- 只导入 `waist_*` 质量；`base_link` 和其他模块全部排除。
- 8 个密度零件不增加质量，固定并入 `waist_link3`，标记“临时简化，待核实”。
- 默认载荷—摩擦系数 `0.001 N·m/N` 仅用于比较计算。
- 页面使用直接工程表述，不使用宣传性或拟人化文字。

## Review Focus

- PART 名称或单位错配不得静默进入计算。
- J0/J1/J2 原闭链解算数值必须保持不变。
- 直接重力矩不得分摊成连接副摩擦贡献。
- 只有密度的 8 个零件不得重复增加质量。
- 参数变化导致排名交叉时必须显示，不得只显示名义排名。

---

### Task 1: 质量属性数据与 PART 映射

**Files:**
- Create: `web2/data/ET2_waist_dynamics_addon.json`
- Create: `web2/data/ET2_waist_dynamics_body_table.csv`
- Create: `web2/src/torso-dynamics-data.js`
- Modify: `web2/scripts/build-pages.mjs`
- Create: `web2/tests/torso-dynamics-data.test.mjs`

**Interfaces:**
- Consumes: 补充包 JSON/CSV 中的 `part_name`、质量、质心、惯量和重力。
- Produces: `window.TorsoDynamicsData`，包含 `gravity`, `explicitBodies`, `mergedAttachments`, `partMapping`, `knownMassKg`。

- [ ] **Step 1: 写失败测试**

断言 9 个显式质量 PART、8 个附属几何、已知质量 `11.3106595 kg`、重力 `[0,0,-9.80665]`、`base_link` 排除、`waist_link3` 简化状态和全部 PART 映射。

- [ ] **Step 2: 运行测试并确认缺少数据模块而失败**

Run: `node web2/tests/torso-dynamics-data.test.mjs`

- [ ] **Step 3: 导入原始数据并实现数据模块**

数据模块保留 ADAMS 原始惯量分量及符号，不对 8 个附属几何补虚拟质量。

- [ ] **Step 4: 更新构建脚本并运行测试**

Run: `node web2/scripts/build-pages.mjs && node web2/tests/torso-dynamics-data.test.mjs`

- [ ] **Step 5: 提交**

提交数据、数据模块、构建和测试。

### Task 2: 刚体重力与等效惯量

**Files:**
- Modify: `web2/src/torso-model.js`
- Modify: `web2/tests/torso-model.test.mjs`

**Interfaces:**
- Consumes: Task 1 的 `TorsoDynamicsData` 和现有 `solveJoint`/`worldGeometry`。
- Produces: `bodyPoses(solution)`, `gravityHoldingTorque(joint, outputDegrees)`, `effectiveInertia(joint, outputDegrees)`, `trajectoryDynamics(joint, outputDegrees)`。

- [ ] **Step 1: 写失败测试**

断言 9 个刚体姿态、零位质心、J0 重力保持力矩接近零、J1/J2结果有限、等效惯量为正、1 Hz惯性力矩有限，并重跑原闭链数值测试。

- [ ] **Step 2: 运行测试并确认接口不存在而失败**

- [ ] **Step 3: 实现刚体姿态**

Yaw、Roll、Pitch、左右曲柄按现有刚体变换；左右连杆采用最小旋转和零附加轴向自旋。

- [ ] **Step 4: 实现重力与惯量计算**

重力保持力矩由势能对目标输出角的中心差分得到；等效惯量由各刚体质心速度和角速度的动能系数求和；1 Hz惯性力矩包含 `J(q)qddot + 0.5 J'(q)qdot²`。

- [ ] **Step 5: 运行测试并提交**

### Task 3: 加载后贡献度与敏感度

**Files:**
- Modify: `web2/src/torso-model.js`
- Modify: `web2/tests/torso-model.test.mjs`

**Interfaces:**
- Consumes: Task 2 刚体动力学和现有 `rates`/`equivalentResistance`。
- Produces: `connectionLoads(solution, assumptions)`, `loadedResistance(joint, solution, loads, direction, mode, assumptions)`, `loadedCycle(joint, loads, assumptions, options)`, `rankStability(...)`。

- [ ] **Step 1: 写失败测试**

断言零载荷系数时加载后结果等于基准；正系数时总阻力增加；贡献度合计 100%；每项含基准值、加载值、百分点变化、N·m增量和负载敏感度；J1/J2支路分载变化会改变对应左右贡献；稳定性输出名次范围和前三比例。

- [ ] **Step 2: 运行测试并确认接口不存在而失败**

- [ ] **Step 3: 实现名义承载与加载后阻力**

左右连杆按两力杆和可编辑分载比例计算；输出轴、曲柄轴和球铰承载由已知自重、支路载荷及预紧组成。重力直接力矩保持独立。

- [ ] **Step 4: 实现周期积分和参数扫描**

周期仍使用 121 点；稳定性扫描采用固定输入集合，改变左右分载、系数倍率和预紧，输出可复现结果。

- [ ] **Step 5: 运行测试并提交**

### Task 4: 网页呈现与发布验证

**Files:**
- Modify: `web2/src/torso-joint.fragment.html`
- Modify: `web2/site.css`
- Modify: `web2/tests/torso-pages.test.mjs`
- Generate: `web2/torso-j0.html`, `web2/torso-j1.html`, `web2/torso-j2.html`

**Interfaces:**
- Consumes: Tasks 1–3 的动力学和加载后贡献度接口。
- Produces: PART表、假设输入、加载前后排序、变化量、负载敏感度、稳定性和总力矩组成。

- [ ] **Step 1: 写失败页面测试**

断言页面有质量属性、`waist_link3`简化提示、系数/预紧/分载输入、加载前后百分比、百分点、N·m增量、敏感度、名次范围、前三比例、重力保持力矩、等效惯量、惯性力矩和使用限制。

- [ ] **Step 2: 运行测试并确认缺少页面模块而失败**

- [ ] **Step 3: 实现页面和样式**

沿用现有横向贡献度柱条；加载后贡献度为主值，基准值、变化量和敏感度放在同一行。直接重力和惯性仅进入总力矩组成。

- [ ] **Step 4: 构建并运行网页2全部测试**

Run: `node web2/scripts/build-pages.mjs`，随后运行 `web2/tests/*.test.mjs`。

- [ ] **Step 5: 浏览器检查**

检查 J0/J1/J2 当前姿态和完整周期、参数修改、排名变化、文字遮挡及网页2内部导航。

- [ ] **Step 6: 提交并发布网页2**

网页1公开文件差异必须为零；合并后再次运行网页2全部测试，再推送 GitHub Pages。

## Self-review

- 数据导入、刚体动力学、加载后贡献度和页面呈现均有独立测试任务。
- 原闭链回归测试贯穿 Task 2–4，防止动力学补充改变运动学。
- 直接重力矩与连接副摩擦贡献分开，避免错误分摊。
- 输入不充分的部分均通过可编辑参数和排名范围表达。
