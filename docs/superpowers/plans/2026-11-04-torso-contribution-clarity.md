# 躯干动态贡献度直观展示 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 J0/J1/J2 页面中明确每项阻力的实际来源、目标输出轴，并用随姿态实时变化的排序条形图和机构高亮展示贡献度。

**Architecture:** 保留 `TorsoModel` 的闭链和虚功计算，只扩展页面语义与可视化。当前姿态条形图直接消费 `equivalentResistance()` 的 `parts` 和 `contribution`；周期条形图消费 `cycle()` 的积分排名；SVG 元素通过连接副 key 与排名条目关联。

**Tech Stack:** 原生 HTML/CSS/JavaScript、Node.js 回归测试、现有静态页面生成器。

**Spec:** `docs/superpowers/specs/2026-11-04-torso-contribution-clarity-design.md`

## Global Constraints

- 不改变硬点、闭链解算、1 Hz ±10° 工况和虚功折算公式。
- J0/J1/J2 的贡献度目标分别固定为 Yaw/Roll/Pitch 输出轴等效阻力矩。
- 曲柄项必须命名为电机曲柄轴旋转副，并明确不是刚性曲柄杆本体摩擦。
- 所有连接副默认静摩擦值相同、默认动摩擦值相同、正反向初值相同，同时保留逐项调节；默认值必须标明仅为演示值。
- 页面继续由 `scripts/build-pages.mjs` 生成并发布到现有 GitHub Pages。

## Review Focus

- J0/J1/J2 页面必须显示各自正确的目标输出轴名称，避免模板替换错误；由页面回归测试覆盖。
- 动画播放时实时贡献度条形图必须随 `update()` 每帧刷新；由页面源码行为测试覆盖。
- 排名必须按贡献度降序，不得沿用固定连接副顺序；由纯函数测试覆盖。
- 点击排名项必须高亮正确的轴或铰点；由 data-key 映射测试覆盖。
- 当总阻力为零时，条形图显示 0%，不得出现 NaN；由纯函数测试覆盖。

---

### Task 1: 明确贡献度语义与阻力来源

**Files:**
- Modify: `src/torso-joint.fragment.html`
- Modify: `tests/torso-pages.test.mjs`

**Interfaces:**
- Consumes: `__JOINT__`、`__MOTION__` 模板变量和现有 `rateKeys`。
- Produces: `names`、`sources`、目标轴说明和演示值提示，供实时及周期图复用。

- [ ] **Step 1: Write the failing test** — 断言页面含“左电机曲柄轴旋转副”“不是刚性曲柄杆本体摩擦”“目标输出轴等效阻力矩”“演示默认值”，并验证所有连接副的默认静摩擦相同、默认动摩擦相同且正反向初值相同。
- [ ] **Step 2: Run test to verify it fails** — `node tests/torso-pages.test.mjs`，预期因新文案缺失失败。
- [ ] **Step 3: Implement the semantic copy and source mapping** — 名称、来源说明和 J0/J1/J2 目标轴动态文案全部由同一映射生成。
- [ ] **Step 4: Run test to verify it passes** — `node scripts/build-pages.mjs; node tests/torso-pages.test.mjs`。
- [ ] **Step 5: Commit** — `git commit -m "Clarify torso resistance contribution semantics"`。

### Task 2: 实时动态贡献度排序条形图与机构高亮

**Files:**
- Modify: `src/torso-joint.fragment.html`
- Modify: `site.css`
- Modify: `tests/torso-pages.test.mjs`

**Interfaces:**
- Consumes: `M.equivalentResistance(joint, solution, loads, direction, mode)` 返回的 `parts` 和 `contribution`。
- Produces: `renderContributionBars(result, mode)`、`data-torso-live-bars`、`data-torso-key` 高亮映射。

- [ ] **Step 1: Write the failing test** — 断言实时条形图容器、降序排序、等效 N·m/百分比、启动/运行切换、SVG key 和点击高亮代码存在。
- [ ] **Step 2: Run test to verify it fails** — `node tests/torso-pages.test.mjs`。
- [ ] **Step 3: Implement the live ranking** — 每次 `update()`/动画帧重新排序并渲染；点击条目高亮对应输出轴、主动轴旋转副或球铰；零总阻力显示 0%。
- [ ] **Step 4: Run test to verify it passes** — `node scripts/build-pages.mjs; node tests/torso-pages.test.mjs`。
- [ ] **Step 5: Commit** — `git commit -m "Add live torso contribution ranking"`。

### Task 3: 周期贡献度条形图与运动曲线分区

**Files:**
- Modify: `src/torso-joint.fragment.html`
- Modify: `site.css`
- Modify: `tests/torso-pages.test.mjs`

**Interfaces:**
- Consumes: `M.cycle()` 的 `rankings.static`、`rankings.dynamic` 和 `samples`。
- Produces: 周期积分量/占比排序条形图；独立“闭链运动关系”曲线区。

- [ ] **Step 1: Write the failing test** — 断言“完整周期贡献度”和“闭链运动关系”是两个独立区域，周期条形图显示积分量和占比。
- [ ] **Step 2: Run test to verify it fails** — `node tests/torso-pages.test.mjs`。
- [ ] **Step 3: Implement the separated views** — 排名按贡献度降序，静/动切换，角度曲线不再置于贡献度区标题下。
- [ ] **Step 4: Run task and full regression tests** — `node scripts/build-pages.mjs` 后运行全部 `tests/*.test.mjs`，预期全部通过。
- [ ] **Step 5: Commit** — `git commit -m "Separate torso cycle ranking from motion chart"`。
