# 全关节连杆长度调节模块 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为腿足 J3、J4 和手臂 J3 增加与 J2 一致的实时连杆长度调节模块。

**Architecture:** 页面层维护标准长度与 ΔL 输入，生成 overrides 并传入现有模型的 `solve(angle, overrides)`。所有当前态与周期计算共享同一 overrides；公差模块继续使用标准态模型。

**Tech Stack:** 静态 HTML/CSS/JavaScript、Node.js 回归测试、GitHub Pages

**Spec:** `docs/superpowers/specs/2026-10-27-joint-link-length-controls-design.md`

## Global Constraints

- 不把推杆作为长度调节项。
- 调节后闭环、雅可比、静态和动态阻力全部重算。
- 闭环无解必须可见且可通过恢复标准态解除。

## Review Focus

- ΔL 是否真正进入模型，而非只改变文字。
- 周期峰值与贡献度是否使用当前尺寸。
- 无解尺寸是否停止动画并显示错误。
- 恢复按钮是否把全部 ΔL 归零。
- 公差图是否仍以标准态为基准。

### Task 1: 页面模块与实时计算接线

**Files:**
- Modify: `site.css`
- Modify: `src/j3.fragment.html`
- Modify: `src/j4.fragment.html`
- Modify: `src/arm-j3.fragment.html`
- Test: `tests/joint-link-length-controls.test.mjs`

**Interfaces:**
- Consumes: `J3Model.solve(angle, overrides)`, `J4Model.solve(angle, overrides)`, `ArmJ3Model.solve(angle, overrides)`
- Produces: 每页 `currentLengths()`、状态行、恢复按钮以及使用当前尺寸的全计算链

- [ ] 写页面行为回归测试并确认因模块缺失而失败。
- [ ] 添加统一样式和三个页面的 ΔL 输入模块。
- [ ] 将机构绘制、静态峰值、贡献度和运行周期计算接入当前尺寸。
- [ ] 添加无解状态处理与恢复标准态行为。
- [ ] 构建页面并运行完整测试套件。
- [ ] 浏览器验证三个页面输入、恢复和错误状态。
