# 网页1冻结与网页2副本实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将当前线上版本冻结为网页1，并建立可独立构建、独立修改和独立访问的网页2副本。

**Architecture:** `webpage-1` Git 标签固定网页1的公开版本；仓库根目录继续承载网页1。`web2/` 完整复制源文件、构建脚本、测试所需模型和生成页面，所有浏览器资源与导航均使用副本内部相对路径。

**Tech Stack:** Git、Node.js ESM、静态 HTML/CSS/JavaScript、GitHub Pages。

**Spec:** `docs/superpowers/specs/2026-11-05-webpage-version-freeze-design.md`

## Global Constraints

- 网页1公开页面和资源内容不得改变。
- 网页2地址固定为 `/et2-sim2real/web2/`。
- 网页2不得以 `../` 或站点根绝对路径引用网页1资源。
- 本次不改变计算模型、参数和交互功能。
- 后续默认仅修改 `web2/`。

## Review Focus

- 网页2首页和各模块链接必须保持在 `web2/` 内。
- 每个网页2页面引用的 CSS/JS 文件必须存在于 `web2/`。
- 副本生成不得递归复制已有 `web2/`。
- 网页1主要公开文件在复制过程前后内容哈希不变。
- 副本必须记录来源提交，便于核验数据版本。

---

### Task 1: 自包含副本生成器

**Files:**
- Create: `scripts/create-web-copy.mjs`
- Create: `tests/web-copy.test.mjs`
- Create: `web2/`（由生成器创建并纳入版本控制）

**Interfaces:**
- Consumes: 仓库根目录公开 HTML/CSS/JS、`src/`、`scripts/`、`tests/`。
- Produces: `createWebCopy({root, destination, sourceCommit})`，生成自包含的 `web2/`。

- [ ] **Step 1: 写失败测试**

测试在临时仓库夹具中调用 `createWebCopy`，断言：复制公开文件和开发源文件、跳过目标目录自身、写入来源版本、HTML 中不存在跨出副本的资源引用。

- [ ] **Step 2: 运行测试并确认因生成器不存在而失败**

Run: `node tests/web-copy.test.mjs`
Expected: FAIL，模块或导出不存在。

- [ ] **Step 3: 实现最小生成器并生成网页2**

实现 `createWebCopy({root: string, destination: string, sourceCommit: string}): void`；复制当前站点文件、`src/`、`scripts/`、`tests/`，排除 `.git`、`.worktrees`、`.superpowers`、`dist` 和目标目录自身；写入 `WEBPAGE_VERSION.md`。

- [ ] **Step 4: 运行测试并确认通过**

Run: `node tests/web-copy.test.mjs`
Expected: PASS。

- [ ] **Step 5: 提交**

提交生成器、测试和 `web2/`。

### Task 2: 冻结标识、回归与发布

**Files:**
- Modify: `web2/WEBPAGE_VERSION.md`
- Verify: 根目录公开文件、`web2/*.html`、全量测试。

**Interfaces:**
- Consumes: Task 1 的自包含网页2。
- Produces: Git 标签 `webpage-1` 指向 `d816a70`，以及可公开访问的网页2。

- [ ] **Step 1: 建立网页1恢复标签并验证目标提交**

Run: `git tag webpage-1 d816a70`，随后核验 `git rev-list -n 1 webpage-1`。

- [ ] **Step 2: 验证网页1未被改写且网页2可独立构建**

运行网页1全部测试；在 `web2/` 运行构建和全部测试；检查 `git diff d816a70 --` 中网页1公开文件无变化。

- [ ] **Step 3: 浏览器检查**

本地打开 `web2/index.html`，进入腿足、头颈、躯干和手臂页面，确认 URL 与资源均位于 `web2/`。

- [ ] **Step 4: 发布并核验线上网址**

推送主分支和 `webpage-1` 标签，核验网页1原网址与网页2新网址均可访问。

## Self-review

- 设计中的冻结、独立副本、恢复标签和不改变模型均有对应任务。
- 生成器与发布验证分开，后续可重复生成但默认不覆盖网页1。
- 测试直接运行复制行为并检查输出，不仅检查源码文本。
