# Neck Hardpoint Control Priority Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the default dynamic-analysis view with a decision-oriented hardpoint ranking while retaining detailed curves in a collapsed section.

**Architecture:** Add cycle-integrated absolute Shapley aggregation to the tolerance model. Render a shared J0/J1 ranking table with static/dynamic shares, peak impact and control recommendation; move existing charts into a closed `<details>` block.

**Tech Stack:** Browser JavaScript, SVG, Node regression tests, static GitHub Pages.

**Spec:** `docs/superpowers/specs/2026-11-04-neck-control-priority-design.md`

## Global Constraints

- Static and dynamic shares each sum to 100% when impact is nonzero.
- Priority uses the larger static/dynamic share; ≥30% high, 10%–30% medium, <10% low unless peak is abnormal.
- Existing dynamic calculation and J0/J1 inputs remain unchanged.

## Review Focus

- Zero and single active-point states.
- Trapezoidal integration over the inclusive 0–1 s samples.
- Signed cancellation must not erase accumulated influence.
- Stable sorting for ties.
- Curves remain available but collapsed by default.

---

### Task 1: Cycle priority aggregation

**Files:** `src/neck-tolerance-model.js`, `tests/neck-tolerance-model.test.mjs`, generated `neck-tolerance-model.js`

- [ ] Add failing tests for zero/single/multi-point shares, 100% conservation and peak tracking.
- [ ] Implement `cyclePriority(dynamicResult)` using trapezoidal integration of `abs(signedImpact)`.
- [ ] Run focused tests and commit.

### Task 2: Decision-first page

**Files:** `src/neck-tolerance.fragment.html`, `tests/neck-tolerance-pages.test.mjs`, generated J0/J1 pages

- [ ] Add failing page assertions for priority table, recommendation labels and closed details section.
- [ ] Render ranking immediately after calculation and move charts/tables into “详细计算”.
- [ ] Run focused tests and commit.

### Task 3: Verify and publish

- [ ] Run all 27 tests.
- [ ] Browser-check zero, single and multiple points on J0 plus J1 loading.
- [ ] Push main and verify both public URLs.
