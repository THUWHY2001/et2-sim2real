# 头颈自定义硬点移动贡献度 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在头颈 J0/J1 公差页面中，根据用户当前自定义硬点偏移计算移动过程曲线、最终静动态阻力增量及多点 Shapley 贡献度。

**Architecture:** 在现有 `NeckToleranceModel` 中增加纯计算的路径采样和精确 Shapley 接口，复用 `responseMetrics`、参数和基础阻力输入。页面模块直接读取现有硬点偏移，不复制输入状态；单点显示最坏工况利用率，多点显示有符号 Shapley 值、幅值贡献度和交互量。

**Tech Stack:** Vanilla JavaScript、SVG、HTML/CSS、Node `assert` 回归测试、现有静态页面生成脚本

**Spec:** `docs/superpowers/specs/2026-11-04-neck-custom-offset-contribution-design.md`

## Global Constraints

- 每个硬点偏移独立满足 `||ΔP|| ≤ 1 mm`，激活阈值为 `1e-6 mm`。
- 质心、质量、标准轴线和推杆标零后无轴向预载基准保持不变。
- 路径固定为 `ΔP(s)=sΔP`，默认21个采样点。
- 静摩擦峰值、动态峰值、动态 RMS 和耗散增量均复用现有1 Hz、±10°计算。
- 最多八个激活点，使用最多256个子集的精确 Shapley，不使用随机近似。
- 风险贡献度按 `|φ_i|/Σ|φ|` 归一化，同时保留 `φ_i` 正负号。
- 不改变现有自动最坏点、点对及鲁棒排名结果。

## Review Focus

- 无激活点时不能出现除零、空表或误报100%贡献。
- 单点移动时贡献度为100%，但必须同时输出非平凡的实际增量和最坏工况利用率。
- 两个点存在抵消时，Shapley符号和幅值贡献度必须分别表达，不能把负值显示成“降低风险百分比”。
- 八点状态必须严格只计算256个唯一子集并保持确定性。
- 自定义计算不得修改页面当前偏移、参数、自动排名缓存或标准硬点。

---

### Task 1: 自定义路径和 Shapley 计算内核

**Files:**
- Modify: `src/neck-tolerance-model.js`
- Modify: `tests/neck-tolerance-model.test.mjs`

**Interfaces:**
- Consumes: `responseMetrics(joint,offsets,parameters,loads,steps)` and existing offset validation
- Produces: `customPath(joint,targetOffsets,parameters,loads,options)`, `customContribution(joint,targetOffsets,parameters,loads,options)`

- [ ] **Step 1: Write failing custom-path tests**

  Assert 21 samples include progress 0 and 1; zero-progress metrics are zero; final metrics equal direct `responseMetrics`; an offset over 1 mm throws; input offsets remain unchanged.

- [ ] **Step 2: Run test and verify RED**

  Run: `node tests/neck-tolerance-model.test.mjs`
  Expected: FAIL because `customPath` is missing.

- [ ] **Step 3: Implement `customPath`**

  Scale every target offset by the same progress value and return static peak, dynamic peak, RMS and energy for each sample.

- [ ] **Step 4: Write failing Shapley tests**

  Assert no-point result is empty and finite; one-point Shapley equals total impact with 100% magnitude contribution; two-point Shapley values sum to the total metric; negative marginal values retain their sign; eight points report 256 evaluated subsets and deterministic output.

- [ ] **Step 5: Run test and verify RED**

  Run: `node tests/neck-tolerance-model.test.mjs`
  Expected: FAIL because `customContribution` is missing.

- [ ] **Step 6: Implement exact subset caching and Shapley allocation**

  Return separate static/dynamic results with `totalImpact`, `interaction`, `evaluatedSubsets`, and per-point `signedImpact`, `contribution`, `effect` (`aggravate`, `mitigate`, `neutral`). For one active point also compute `worstCaseUtilization` from `rankContributions`.

- [ ] **Step 7: Run focused regression and commit**

  Run: `node tests/neck-tolerance-model.test.mjs`
  Expected: PASS.

  `git commit -m "Add neck custom offset contribution model"`

### Task 2: J0/J1 自定义状态页面模块

**Files:**
- Modify: `src/neck-tolerance.fragment.html`
- Modify: `tests/neck-tolerance-pages.test.mjs`

**Interfaces:**
- Consumes: Task 1 `customPath` and `customContribution`, plus existing `offsets`, `parameters`, `loads`
- Produces: “自定义公差状态分析” section in both generated pages

- [ ] **Step 1: Write failing page tests**

  Assert the page contains a custom-analysis button, active-point summary, path chart, static/dynamic Shapley tables, worst-case utilization, interaction output and no-point guidance.

- [ ] **Step 2: Build and verify RED**

  Run: `node scripts/build-pages.mjs; node tests/neck-tolerance-pages.test.mjs`
  Expected: FAIL because the module is absent.

- [ ] **Step 3: Add the module markup and rendering**

  Add “计算当前自定义状态” below the point inputs. Render a 0%–100% SVG curve and result tables without adding another offset form.

- [ ] **Step 4: Wire computation and states**

  Read current offsets on button click; disable the button while calculating; render empty guidance for zero active points; show single-point utilization or multi-point interaction; label positive effects “加剧” and negative effects “抵消”.

- [ ] **Step 5: Verify page syntax and focused tests**

  Run: `node scripts/build-pages.mjs; node tests/neck-tolerance-pages.test.mjs; node tests/neck-tolerance-model.test.mjs; node tests/neck-pages.test.mjs`
  Expected: all PASS.

- [ ] **Step 6: Commit**

  `git commit -m "Add neck custom offset contribution module"`

### Task 3: 浏览器核验、全量回归和发布

**Files:**
- Modify only files required by verified corrections

**Interfaces:**
- Consumes: Tasks 1–2
- Produces: verified J0/J1 public pages

- [ ] **Step 1: Verify browser behavior**

  Check zero-point guidance; one custom point at 0.5 mm; two custom points including an interaction result; projection/reset; curve rendering; no console errors.

- [ ] **Step 2: Run complete regression suite**

  Run all `tests/*.test.mjs`.
  Expected: every test exits 0.

- [ ] **Step 3: Commit verified corrections and push**

  Commit only if browser verification required fixes, then push `main` and verify both GitHub Pages URLs contain the new custom-analysis marker.
