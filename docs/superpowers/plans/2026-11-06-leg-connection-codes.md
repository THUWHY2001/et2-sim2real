# 腿足连接副统一编号 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为腿足25个运动副建立稳定的 `L01`～`L25` 编号，并在机构图、对照表及全部贡献度展示中统一使用。

**Architecture:** 从 `model.joints` 的稳定模型顺序生成唯一编号映射；页面所有名称均通过同一个 `jointMeta()` 接口读取。机构图按运动副两侧 Marker 中心绘制编号，对照表保留原始名称和连接零件。

**Tech Stack:** 静态 HTML、原生 JavaScript、SVG、Node.js 回归测试

**Spec:** `docs/superpowers/specs/2026-11-06-leg-connection-codes-design.md`

## Global Constraints

- 不修改闭链运动学、阻力折算、质量属性和负载计算。
- `L01`～`L25` 在 J2/J3/J4 三个页面必须完全一致。
- 25个运动副无重复、无遗漏，原始模型名可追溯。

## Review Focus

- 模型对象顺序变化：编号依据显式稳定键序列，不得依赖贡献度排序。
- 一个运动副两侧 Marker：图上只绘制一个中心编号。
- 当前关节不相关的运动副：仍出现在统一对照表，但不进入该页贡献度排名。
- 原始名称较长：对照表允许换行，图中仅显示短编号。
- 根目录与 `/web2` 页面：构建后两套入口内容必须一致。

---

### Task 1: 编号映射和回归测试

**Files:**
- Modify: `web2/tests/leg-full-pages.test.mjs`
- Modify: `web2/src/leg-full-joint.fragment.html`

**Interfaces:**
- Produces: `jointCodes: Record<string,string>`、`jointMeta(key)`，返回编号、中文名、类型和连接零件。

- [ ] **Step 1: 写失败测试**

断言页面包含 `L01`、`L25`、连接副对照表，并且阻力输入及贡献度通过统一格式化函数显示编号。

- [ ] **Step 2: 运行测试确认失败**

Run: `node web2/tests/leg-full-pages.test.mjs`

- [ ] **Step 3: 实现稳定编号映射和统一名称接口**

在共享腿足页面脚本中根据完整25个运动副键序列建立 `L01`～`L25`，所有输入表和贡献度行统一调用该接口。

- [ ] **Step 4: 运行测试确认通过**

Run: `node web2/tests/leg-full-pages.test.mjs`

### Task 2: 机构图标注和对照表

**Files:**
- Modify: `web2/src/leg-full-joint.fragment.html`
- Modify: `web2/site.css`
- Modify: `web2/scripts/build-pages.mjs`
- Generated: `j2.html`, `j3.html`, `j4.html`, `web2/j2.html`, `web2/j3.html`, `web2/j4.html`

**Interfaces:**
- Consumes: Task 1 的 `jointMeta(key)`。
- Produces: 图中 `Lxx` 标签和 `[data-leg-full-joint-key]` 对照表。

- [ ] **Step 1: 扩展失败测试**

断言25个编号均进入对照表，SVG绘图使用相同编号，根目录页面同步生成。

- [ ] **Step 2: 实现运动副中心标签与对照表**

用每个运动副两侧 Marker 坐标的中点绘制 `Lxx`；表格显示编号、中文名、类型、连接零件、原始模型名。

- [ ] **Step 3: 构建并运行全部测试**

Run: `node web2/scripts/build-pages.mjs`，随后运行 `web2/tests/*.test.mjs`。

- [ ] **Step 4: 浏览器检查并提交**

核对 J2/J3/J4 图中编号、贡献度名称和对照表一致，再提交并发布到 `main`。
