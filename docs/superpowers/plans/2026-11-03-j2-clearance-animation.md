# J2 Clearance Animation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 统一腿足滑动轴承术语，并让J2局部间隙动画清晰连续。

**Architecture:** 保留现有J2计算模型和主机构绘图，只重写局部放大SVG渲染。用1 Hz相位生成0→1→0的可视化接触进度，驱动单一轴心在衬套内移动。

**Tech Stack:** HTML、内联JavaScript、SVG、Node.js回归测试

**Spec:** `docs/superpowers/specs/2026-11-03-j2-clearance-animation-design.md`

## Global Constraints

- 术语统一为“滑动轴承”。
- 计算结果与现有键名不变。
- 页面不增加原理长文。

## Review Focus

- 动画必须存在单一放大窗口和连续轴心位置。
- 正、负侧不对称间隙仍能分别显示。
- J2/J3/J4不得残留“滑套轴承”。
- 主机构不得重新引入放大后的杆件几何。
- 全部既有回归测试必须通过。

### Task 1: 回归测试

**Files:**
- Modify: `tests/j2-clearance-page.test.mjs`
- Modify: `tests/leg-sleeve-bearing.test.mjs`

- [ ] 写入术语与单窗口动画断言。
- [ ] 运行测试并确认因功能尚未实现而失败。

### Task 2: 页面实现

**Files:**
- Modify: `src/j2-clearance.fragment.html`
- Modify: `src/j2.fragment.html`
- Modify: `src/j3.fragment.html`
- Modify: `src/j4.fragment.html`

- [ ] 全局替换腿足显示术语。
- [ ] 将局部三联图改为单一连续移动放大图。
- [ ] 构建页面并运行全部测试。
- [ ] 提交并发布公开页面。

