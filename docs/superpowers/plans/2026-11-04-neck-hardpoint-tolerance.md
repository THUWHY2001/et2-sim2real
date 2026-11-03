# 头颈硬点公差试点 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在头颈 J0/J1 上试点八个硬点的 1 mm 球形公差分析，输出静态启动力矩与动态运行阻力的单点及两点组合贡献度排名。

**Architecture:** 保留 `NeckModel` 作为标准态计算基础，新建纯计算模块 `NeckToleranceModel`，以标准轴线、偏移后的零件局部硬点和虚功映射计算静态/动态阻力变化。J0/J1 共用一个公差分析页面片段；新旧贡献度差异仅在本地核验并通过当前对话反馈，不进入网页或公开文件。

**Tech Stack:** Vanilla JavaScript、SVG、HTML/CSS、Node `assert` 回归测试、现有静态页面生成脚本

**Spec:** `docs/superpowers/specs/2026-11-04-neck-hardpoint-tolerance-design.md`

## Global Constraints

- 只试点头颈 J0/J1，不修改腿足、躯干和手臂模型。
- 八个硬点各自在固定半径 1 mm 球内独立移动；质心保持标准值。
- J0/J1 始终绕改动前的标准轴线运动，公差点不得重建运动轴。
- 每次先标 J0、再标 J1；当前标零销仅保留接口，不预测销孔预载或拔销释放量。
- 单轴采用 1 Hz、±10° 激励，另一关节伺服保持零位。
- 静摩擦启动力矩和动态运行阻力独立计算、独立排序。
- 假设参数结果、几何结果和实测参数结果必须清楚区分，不能把理论假设标为实测值。
- 内部新旧版本核验必须使用同一组摩擦、刚度、间隙、方向和激励输入，结果不写入网页或仓库文件。

## Review Focus

- 偏移模长恰为 1 mm 时允许计算，超过 1 mm 时拒绝且不能污染上一有效状态。
- 标准态偏移全部为零时，新模型的旧版兼容结果必须与 `NeckModel` 数值一致。
- R1/R2 或 P1/P2 偏移后，运动轴仍等于标准轴，侧载残差随偏移变化。
- 零刚度、零摩擦、正反不对称参数和全零贡献时不得出现 `NaN` 或错误归一化。
- 内部核验不能把数值舍入误差报告成显著变化。

---

### Task 1: 公差几何与参数校验内核

**Files:**
- Create: `src/neck-tolerance-model.js`
- Create: `tests/neck-tolerance-model.test.mjs`

**Interfaces:**
- Consumes: `window.NeckModel.points`, `keys`, `solve`, `equivalent`, `cycle`
- Produces: `window.NeckToleranceModel.validateOffsets(offsets)`, `projectOffset(offset)`, `createState(joint,degrees,offsets,parameters)`, `evaluate(joint,offsets,parameters,loads)`

- [ ] **Step 1: Write the failing geometry tests**

  Assert zero offsets are accepted; `(1,0,0)` is accepted; `(1,1,0)` is rejected; projection of `(1,1,0)` has norm 1 mm; a rejected update leaves the caller's previous offsets unchanged.

- [ ] **Step 2: Run test to verify it fails**

  Run: `node tests/neck-tolerance-model.test.mjs`
  Expected: FAIL because `src/neck-tolerance-model.js` does not exist.

- [ ] **Step 3: Implement immutable point offsets and fixed nominal axes**

  `createState` must rotate offset moving points with their owning rigid body, retain the nominal `NeckModel` axis, and return rod lengths, rod Jacobians, radial axis residuals and closure residuals without modifying `NeckModel.points`.

- [ ] **Step 4: Add fixed-axis and reset assertions**

  Assert R1/R2/P1/P2 offsets do not change `state.axis`; all-zero offsets reproduce nominal geometry; resetting an offset reproduces the original standard state.

- [ ] **Step 5: Run test to verify it passes**

  Run: `node tests/neck-tolerance-model.test.mjs`
  Expected: PASS.

- [ ] **Step 6: Commit**

  `git commit -m "Add neck hardpoint tolerance geometry model"`

### Task 2: 侧载、阻力与静动态贡献度

**Files:**
- Modify: `src/neck-tolerance-model.js`
- Modify: `tests/neck-tolerance-model.test.mjs`

**Interfaces:**
- Consumes: Task 1 `createState`
- Produces: `jointLoads(state,parameters,direction,mode)`, `equivalentResistance(...)`, `cycle(...)`, `rankContributions(...)`, `rankPairContributions(...)`

- [ ] **Step 1: Write failing resistance tests**

  Assert clearance produces zero side load below threshold; side load is `k_r(|e|-c)` above threshold; positive/negative clearances select by motion direction; static and dynamic friction use their own base and load coefficients.

- [ ] **Step 2: Run focused test and verify RED**

  Run: `node tests/neck-tolerance-model.test.mjs`
  Expected: FAIL on missing load functions.

- [ ] **Step 3: Implement load and virtual-work mapping**

  Use radial residual against the nominal axis, axial rod deformation and pose-dependent numerical Jacobians. Return per-connection equivalent torque, total friction torque, gravity hold torque and drive torque; keep gravity outside friction contribution percentages.

- [ ] **Step 4: Implement one-at-a-time worst-direction contribution search**

  `rankContributions` scans a deterministic unit-sphere direction set plus local refinement for each hardpoint while all other points remain standard. Return independent static peak and dynamic peak/RMS/mean-absolute/energy rankings.

- [ ] **Step 5: Implement two-point combination search**

  `rankPairContributions` evaluates all 28 hardpoint pairs with deterministic joint direction refinement, keeps each offset inside its own 1 mm sphere and returns the most sensitive pairs with both directions, torque increment and normalized pair contribution.

- [ ] **Step 6: Add normalization, pair-boundary and finite-result tests**

  Assert single-point and pair contributions total 100% when their respective totals are nonzero and all return 0% when zero; all 28 unique pairs are evaluated; neither point exceeds 1 mm; asymmetric and zero parameter cases remain finite.

- [ ] **Step 7: Run focused test and verify GREEN**

  Run: `node tests/neck-tolerance-model.test.mjs`
  Expected: PASS.

- [ ] **Step 8: Commit**

  `git commit -m "Calculate neck tolerance resistance contributions"`

### Task 3: J0/J1 公差分析页面

**Files:**
- Create: `src/neck-tolerance.fragment.html`
- Modify: `scripts/build-pages.mjs`
- Modify: `site.css`
- Create: `tests/neck-tolerance-pages.test.mjs`

**Interfaces:**
- Consumes: `window.NeckModel`, `window.NeckToleranceModel`, `window.Neck3DView`
- Produces: `neck-j0-tolerance.html`, `neck-j1-tolerance.html`, existing J0/J1 entry links and built `neck-tolerance-model.js`

- [ ] **Step 1: Write failing page-generation tests**

  Assert two pages and assets exist; J0/J1 pages link to them; each page includes eight point rows, fixed `≤1 mm` validation、单点/全部复位、参数状态以及静态/动态排名，不包含新旧版本对比文案或表格。

- [ ] **Step 2: Build and run page test to verify RED**

  Run: `node scripts/build-pages.mjs; node tests/neck-tolerance-pages.test.mjs`
  Expected: FAIL because tolerance pages are absent.

- [ ] **Step 3: Add shared point and parameter controls**

  Provide per-point `ΔX/ΔY/ΔZ`, norm, enable, projection, worst-direction and reset controls. Provide nominal/lower/upper fields for stiffness, clearance and friction coefficients; zero-pin fields are present but disabled and labelled reserved.

- [ ] **Step 4: Add rotatable 3D tolerance visualization**

  Draw nominal points, 1 mm spheres, actual points, offset vectors, nominal axes and current mechanism; use the existing camera behavior and preserve the current reset view.

- [ ] **Step 5: Add result presentation**

  Display static/dynamic single-point rankings plus the most sensitive point-pair ranking, torque impact, percentage, worst direction/angle and contribution interval placeholders. All-zero reset must show the current standard model.

- [ ] **Step 6: Run page and focused model tests**

  Run: `node scripts/build-pages.mjs; node tests/neck-tolerance-pages.test.mjs; node tests/neck-tolerance-model.test.mjs`
  Expected: all PASS.

- [ ] **Step 7: Commit**

  `git commit -m "Add neck hardpoint tolerance analysis pages"`

### Task 4: 参数区间鲁棒排名

**Files:**
- Modify: `src/neck-tolerance-model.js`
- Modify: `src/neck-tolerance.fragment.html`
- Modify: `tests/neck-tolerance-model.test.mjs`
- Modify: `tests/neck-tolerance-pages.test.mjs`

**Interfaces:**
- Consumes: Task 2 rankings and parameter lower/nominal/upper values
- Produces: `robustRanking(joint,loads,parameterRanges,options)` with 5–95% interval, first-place probability, top-three probability and rank stability

- [ ] **Step 1: Write failing deterministic sampling tests**

  Assert the same seed produces identical intervals/probabilities; probabilities stay within `[0,1]`; a collapsed range reproduces the nominal result.

- [ ] **Step 2: Run focused test and verify RED**

  Run: `node tests/neck-tolerance-model.test.mjs`
  Expected: FAIL on missing `robustRanking`.

- [ ] **Step 3: Implement bounded seeded parameter sampling**

  Default range is `0.1×–10×` nominal. Return interval statistics and ranking stability without changing the nominal computation.

- [ ] **Step 4: Add page outputs**

  Show geometry-only risk, assumed-parameter result and robust interval separately; label inputs as assumed until the user marks them measured.

- [ ] **Step 5: Run focused tests and commit**

  Run: `node scripts/build-pages.mjs; node tests/neck-tolerance-model.test.mjs; node tests/neck-tolerance-pages.test.mjs`
  Expected: PASS.

  `git commit -m "Add robust neck tolerance ranking"`

### Task 5: 全量回归、浏览器核验与发布准备

**Files:**
- Modify only files required by verified corrections

**Interfaces:**
- Consumes: all preceding tasks
- Produces: verified public root files and `dist/` artifacts

- [ ] **Step 1: Run the complete Node regression suite**

  Run: `Get-ChildItem tests/*.test.mjs | ForEach-Object { node $_.FullName }`
  Expected: every test exits 0.

- [ ] **Step 2: Verify both pages in a desktop browser**

  Check J0 and J1 at zero offset, one 1 mm boundary offset, one rejected oversize offset, ±10° motion, reset and worst-direction search without console errors.

- [ ] **Step 3: Run a private one-off comparison**

  在本地临时命令中使用完全相同的输入计算旧版和新版 J0/J1 贡献度、最大百分点差和排名变化；不保存脚本、不写入网页或仓库文件，只在当前对话中向用户反馈结论，并说明实测参数可能改变结论。

- [ ] **Step 4: Commit verified corrections**

  `git commit -m "Verify neck tolerance analysis pilot"`
