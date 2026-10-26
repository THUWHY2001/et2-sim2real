# 全关节连杆长度公差分析实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为腿足 J2/J3/J4 和手臂 J3 提供统一、可核验的连杆长度公差—峰值等效启动力矩曲线。

**Architecture:** 新建共享的公差扫描、二次拟合和 SVG 卡片渲染模块；各机构模型只负责接受长度覆盖值并准确重建自身闭环。页面向共享模块提供模型、可调杆名称、静摩擦载荷和启动方向。

**Tech Stack:** 原生 JavaScript、SVG、静态 HTML/CSS、Node.js 回归测试。

**Spec:** `docs/superpowers/specs/2026-10-26-all-joint-link-tolerance-design.md`

## Global Constraints

- 推杆不作为普通连杆参与长度公差扫描。
- 每个采样点重新求解闭环和雅可比，并取 ±10°行程峰值启动力矩。
- 拟合方程使用普通小数，显示 dτ/dL、最大变化率和 R²。
- 不增加第三方运行时依赖。

## Review Focus

- 公差带边界 199.999/200/300/400 mm 必须按既定区间处理。
- 无解公差点不得进入拟合或伪造数值。
- 各模型长度变化后实际点距必须等于请求长度。
- 标准态 ΔL=0 的峰值必须与页面原有峰值一致。
- 动画帧不得重复执行整套公差扫描，只在静摩擦参数或方向改变时更新。

---

### Task 1: 共享公差计算与图表

**Files:**
- Create: `src/link-tolerance.js`
- Modify: `scripts/build-pages.mjs`
- Test: `tests/link-tolerance.test.mjs`

**Interfaces:**
- Produces: `window.LinkTolerance.toleranceForLength(length)`、`quadraticFit(points)`、`analyze(options)`、`render(options)`。

- [ ] 先写边界、公差扫描、普通小数、灵敏度和无解点测试并确认失败。
- [ ] 实现共享模块并复制到发布目录。
- [ ] 运行共享模块测试并确认通过。

### Task 2: J2 迁移到统一结果格式

**Files:**
- Modify: `src/j2.fragment.html`
- Test: `tests/j2-tolerance-analysis.test.mjs`

**Interfaces:**
- Consumes: Task 1 的 `LinkTolerance`。

- [ ] 先更新测试，要求普通小数、标准态灵敏度和最大变化率，并确认失败。
- [ ] 用共享模块替换 J2 内联拟合与渲染逻辑，保持现有五根杆和数值结果。
- [ ] 运行 J2 测试并确认通过。

### Task 3: 腿足 J3 长度覆盖与公差页面

**Files:**
- Modify: `src/j3.fragment.html`
- Test: `tests/j3-tolerance-analysis.test.mjs`

**Interfaces:**
- Produces: `J3Model.solve(degrees, overrides)`，支持 JP、PA、AQ、JQ。

- [ ] 先写四段杆长约束、闭环、峰值基线与页面卡片测试并确认失败。
- [ ] 实现长度覆盖、闭环校验和四张公差图。
- [ ] 运行 J3 测试并确认通过。

### Task 4: 腿足 J4 长度覆盖与公差页面

**Files:**
- Modify: `src/j4.fragment.html`
- Test: `tests/j4-tolerance-analysis.test.mjs`

**Interfaces:**
- Produces: `J4Model.solve(degrees, overrides)`，仅将 JB 暴露为非推杆可调杆。

- [ ] 先写 JB 点距、峰值基线、排除推杆和页面卡片测试并确认失败。
- [ ] 实现 JB 长度覆盖和一张公差图。
- [ ] 运行 J4 测试并确认通过。

### Task 5: 手臂 J3 长度覆盖与公差页面

**Files:**
- Modify: `src/arm-j3-model.js`
- Modify: `src/arm-j3.fragment.html`
- Test: `tests/arm-j3-tolerance-analysis.test.mjs`

**Interfaces:**
- Produces: `ArmJ3Model.solve(degrees, overrides)`，支持 L、Z、K，保持三维轴系和固定 R/B 点。

- [ ] 先写三段三维杆长、闭环、峰值基线与页面卡片测试并确认失败。
- [ ] 实现三维长度覆盖、闭环校验和三张公差图。
- [ ] 运行手臂 J3 测试并确认通过。

### Task 6: 构建、浏览器核验与发布

**Files:**
- Modify generated: `j2.html`, `j3.html`, `j4.html`, `arm-j3.html`, `dist/*`

- [ ] 构建所有页面并运行完整测试集。
- [ ] 在浏览器逐页检查卡片数量、坐标轴、普通小数、缩放和控制台错误。
- [ ] 提交并推送 GitHub Pages，缓存刷新后在线复核四个页面。
