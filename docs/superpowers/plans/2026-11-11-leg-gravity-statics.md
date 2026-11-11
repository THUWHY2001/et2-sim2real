# ET2 左腿重力准静态约束反力求解器 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有左腿完整闭链运动学上完成基准姿态的六维重力静力平衡、SVD 唯一性分析、推杆保持力和重力虚功校验，并逐项报告 25 个运动副哪些反力可唯一确定。

**Architecture:** 新增一个无 UI 的通用尺度化 SVD 数值模块和一个腿足专用静力模块。静力模块从现有闭链解读取实时 Marker、刚体变换和质心，在统一世界参考点装配约束扳手矩阵；所有公开结果均经过物理坐标零空间唯一性检查。独立报告脚本只消费求解器结果，不参与求解。

**Tech Stack:** 原生 JavaScript ES modules、Node.js `assert` 回归测试、现有 `leg_full.mjs` 与 `leg-full-runtime.mjs`，不新增运行时依赖。

**Spec:** `docs/superpowers/specs/2026-11-11-leg-gravity-statics-design.md`

## Global Constraints

- 不修改硬点、现有完整闭链运动学和 J2/J3/J4 基准角度。
- 世界边界仅使用现有 `base_link ↔ leg_l_link1` 接口；足端无接触。
- J0/J1/J5 使用当前工况角度保持；脚趾使用当前角度理想保持并标记未核实。
- 第一验收门只计算 J2=-27.5°、J3=51.57°、J4=-24.06° 的重力准静态姿态。
- 不包含惯性、装配预紧、摩擦反馈、接触、结构柔度、摩擦圆或页面美化。
- `nominalLoads()` 不得作为铰点约束反力输入。
- 非唯一载荷返回 `null` 与 `indeterminate`，不得展示数值特解。
- 约束合反力不得直接命名为摩擦接触法向载荷。

## Review Focus

- 近秩亏矩阵经尺度化后出现虚假唯一性：Task 1 用跨度 1e9 的已知秩矩阵验证秩、零空间与条件数。
- I/J Marker 不重合导致内部力矩不成对：Task 2 用人为偏置 Marker 验证统一参考点扳手平移与整体零合扳手。
- 位置保持轴错误地当作自由旋转副：Task 2 分别验证 J0/J1/J5/脚趾六维扳手和 J2/J3/J4 五维扳手。
- 非唯一分量泄漏数值特解：Task 3 验证所有有零空间投影的分量值为 `null`。
- 推杆符号与重力势能导数相反：Task 4 对三个输出轴分别验证有符号虚功残差并输出符号诊断。

---

### Task 1: 尺度归一化 SVD 与唯一线性量判定

**Files:**
- Create: `web2/src/rank-revealing-svd.mjs`
- Create: `web2/tests/rank-revealing-svd.test.mjs`

**Interfaces:**
- Consumes: 普通二维数值数组 `A` 和向量 `b`。
- Produces: `solveRankRevealing(A,b,options)`，返回缩放信息、奇异值、秩、增广秩、条件数、物理坐标特解、物理坐标零空间、残差；`evaluateLinearQuantity(solution, coefficients)` 返回 `{status,value,nullProjection}`。

- [ ] **Step 1: 写失败测试**

覆盖满秩唯一解、秩亏一致系统、秩亏不一致系统、跨度 1e9 的尺度矩阵，以及非唯一线性量必须返回 `null`。

- [ ] **Step 2: 运行测试确认 RED**

Run: `node web2/tests/rank-revealing-svd.test.mjs`
Expected: FAIL，因为 `rank-revealing-svd.mjs` 不存在。

- [ ] **Step 3: 实现 `solveRankRevealing` 与 `evaluateLinearQuantity`**

使用行列二范数尺度归一化和单边 Jacobi SVD；将零空间映射回物理未知量坐标。秩阈值同时受相对容差和机器精度控制，条件数只使用非零奇异值。

- [ ] **Step 4: 运行测试确认 GREEN**

Run: `node web2/tests/rank-revealing-svd.test.mjs`
Expected: PASS。

- [ ] **Step 5: 提交**

```bash
git add web2/src/rank-revealing-svd.mjs web2/tests/rank-revealing-svd.test.mjs
git commit -m "Add scaled rank-revealing SVD solver"
```

### Task 2: 运动副空间扳手基与静力矩阵装配

**Files:**
- Create: `web2/src/leg-gravity-statics.mjs`
- Create: `web2/tests/leg-gravity-statics-assembly.test.mjs`

**Interfaces:**
- Consumes: `model`、`createLegFullRuntime(model)` 返回的闭链 `solution`、显式 `boundary`。
- Produces: `createLegGravityStatics(model,runtime)`；实例方法 `assemble(solution,boundary)` 返回 `{A,b,unknowns,equations,joints,boundary}`。

- [ ] **Step 1: 写失败测试**

断言边界字段缺失时拒绝装配；20 个刚体各有 6 条方程；25 个运动副全部进入元数据；三根推杆各有独立轴向未知量；J0/J1/J5/脚趾为位置保持旋转副；J2/J3/J4 为自由旋转副；固定副保留六维扳手但标记不可摩擦。

- [ ] **Step 2: 运行测试确认 RED**

Run: `node web2/tests/leg-gravity-statics-assembly.test.mjs`
Expected: FAIL，因为静力模块不存在。

- [ ] **Step 3: 实现运动副扳手基和 `assemble`**

球副 3 维、自由旋转副 5 维、位置保持旋转副 6 维、移动副 5 维被动扳手加 1 个轴向执行器力、固定副 6 维。统一参考点采用 I/J Marker 中点，并为两侧 Marker 输出平移后的局部力矩表达。

- [ ] **Step 4: 增加偏置 Marker 作用反作用测试并确认 RED**

人为平移某移动副 J Marker，断言旧式局部力矩直接取反会失败，统一参考点换算后的世界合扳手为零。

- [ ] **Step 5: 完成扳手平移实现并确认 GREEN**

Run: `node web2/tests/leg-gravity-statics-assembly.test.mjs`
Expected: PASS。

- [ ] **Step 6: 提交**

```bash
git add web2/src/leg-gravity-statics.mjs web2/tests/leg-gravity-statics-assembly.test.mjs
git commit -m "Assemble leg gravity statics wrench system"
```

### Task 3: 基准姿态静力求解与 25 运动副唯一性报告

**Files:**
- Modify: `web2/src/leg-gravity-statics.mjs`
- Create: `web2/tests/leg-gravity-statics-nominal.test.mjs`

**Interfaces:**
- Consumes: Task 1 的 `solveRankRevealing`、Task 2 的装配结果。
- Produces: `solve(solution,boundary)` 和 `solveNominal(boundary)`；结果含 `matrix`、`residuals`、`jointReactions`、`actuatorForces`、`holdingTorques`、`globalBalance`、`loadInterpretation`。

- [ ] **Step 1: 写失败测试**

断言基准姿态角度不变、系统一致、报告秩/零空间/条件数、20 个刚体六维残差、25 个运动副反力记录、三根推杆保持力、整体重力平衡及内部扳手成对平衡。

- [ ] **Step 2: 运行测试确认 RED**

Run: `node web2/tests/leg-gravity-statics-nominal.test.mjs`
Expected: FAIL，因为 `solveNominal` 尚未实现。

- [ ] **Step 3: 实现 `solve`、唯一性投影和残差报告**

每个 Fx/Fy/Fz、轴向线性分量、两个径向线性分量、约束力矩分量和推杆力分别调用 `evaluateLinearQuantity`；只有所有组成分量唯一时才计算合反力或径向合力，否则其值为 `null`。

- [ ] **Step 4: 增加非唯一结果屏蔽测试并确认 GREEN**

断言所有 `status: "indeterminate"` 项均为 `value: null`，报告不包含用于残差的数值特解；约束反力统一标记为“重力准静态理论约束反力”，接触法向载荷资格单独为 `not_mapped` 或 `candidate_only`。

Run: `node web2/tests/leg-gravity-statics-nominal.test.mjs`
Expected: PASS。

- [ ] **Step 5: 提交**

```bash
git add web2/src/leg-gravity-statics.mjs web2/tests/leg-gravity-statics-nominal.test.mjs
git commit -m "Solve nominal leg gravity statics"
```

### Task 4: 推杆重力虚功交叉校验

**Files:**
- Modify: `web2/src/leg-gravity-statics.mjs`
- Modify: `web2/src/leg-full-runtime.mjs`
- Create: `web2/tests/leg-gravity-virtual-work.test.mjs`

**Interfaces:**
- Consumes: 三根唯一或非唯一推杆轴向力、`runtime.rates(joint,0)` 的有符号移动副传动率、`runtime.gravityHoldingTorque(joint,0)`。
- Produces: `verifyGravityVirtualWork(staticResult,outputJoint)`，返回可唯一确定的单根推杆广义力、总折算力矩、势能微分力矩、绝对/相对残差和 `passed`；运行时新增 `signedRates(joint,deltaDegrees)`，不改变现有 `rates()` 行为。

- [ ] **Step 1: 写失败测试**

对 J2/J3/J4 基准姿态分别断言三根推杆全部出现在虚功明细中、保持角的虚位移为零。即使单根推杆力非唯一，也必须直接对 `sum(F_i*dL_i/dq)` 这个组合线性量做零空间唯一性判定；只有组合量也非唯一时，虚功状态才为 `indeterminate`。

- [ ] **Step 2: 运行测试确认 RED**

Run: `node web2/tests/leg-gravity-virtual-work.test.mjs`
Expected: FAIL，因为 `signedRates` 和虚功接口不存在。

- [ ] **Step 3: 实现有符号速度雅可比和虚功校验**

移动副使用现有轴向位移的中心差分；旋转副使用相对旋转向量投影到当前关节轴，避免 `rotationDistance` 丢失符号。虚功符号按执行器对 J 刚体正轴做功定义统一。总推杆广义力作为未知量向量的一个线性泛函交给 Task 1 的唯一性判定，不能因某一单根推杆力非唯一就提前放弃可能唯一的总虚功。

- [ ] **Step 4: 运行测试确认 GREEN**

Run: `node web2/tests/leg-gravity-virtual-work.test.mjs`
Expected: PASS，或在数学超静定导致推杆力非唯一时按设计稳定返回 `indeterminate`，测试验证该诚实结论。

- [ ] **Step 5: 提交**

```bash
git add web2/src/leg-full-runtime.mjs web2/src/leg-gravity-statics.mjs web2/tests/leg-gravity-virtual-work.test.mjs
git commit -m "Verify leg gravity statics by virtual work"
```

### Task 5: 工程报告与单姿态验收门

**Files:**
- Create: `web2/scripts/report-leg-gravity-statics.mjs`
- Create: `web2/tests/leg-gravity-statics-report.test.mjs`
- Create: `web2/data/leg-full-package/nominal_gravity_statics_report.json`
- Create: `web2/data/leg-full-package/nominal_gravity_statics_report.csv`

**Interfaces:**
- Consumes: Task 3/4 的基准姿态求解和虚功结果。
- Produces: 可审计 JSON、逐运动副 CSV，以及控制台摘要；不生成或修改网页。

- [ ] **Step 1: 写失败测试**

断言报告含理论边界、适用限制、矩阵秩/零空间/条件数、残差、三根推杆、虚功校验、25 个运动副及每个分量唯一性；CSV 单位列明确且没有把约束合反力写成接触法向载荷。

- [ ] **Step 2: 运行测试确认 RED**

Run: `node web2/tests/leg-gravity-statics-report.test.mjs`
Expected: FAIL，因为报告脚本和产物不存在。

- [ ] **Step 3: 实现报告脚本并生成基准产物**

Run: `node web2/scripts/report-leg-gravity-statics.mjs`
Expected: 写入 JSON/CSV，并在控制台明确列出可唯一确定与不能唯一确定的运动副分量。

- [ ] **Step 4: 运行报告测试和完整回归**

Run: `node web2/tests/leg-gravity-statics-report.test.mjs`
Expected: PASS。

Run: `node --test web2/tests/*.test.mjs`
Expected: 全部 PASS；若 Windows 通配符不可用，按文件批次执行全部测试。

- [ ] **Step 5: 检查验收门**

只有当运动学未变化、线性系统一致、刚体/整体残差合格、内部扳手成对平衡且虚功状态为通过或诚实的 `indeterminate` 时，才把单姿态阶段标记完成；不实现 `solveCycle()`。

- [ ] **Step 6: 提交**

```bash
git add web2/scripts/report-leg-gravity-statics.mjs web2/tests/leg-gravity-statics-report.test.mjs web2/data/leg-full-package/nominal_gravity_statics_report.json web2/data/leg-full-package/nominal_gravity_statics_report.csv
git commit -m "Report nominal leg gravity statics"
```
