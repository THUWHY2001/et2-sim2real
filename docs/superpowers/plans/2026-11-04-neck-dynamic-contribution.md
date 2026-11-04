# Head/Neck Dynamic Contribution Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add full-cycle J0/J1 resistance curves and exact time-varying hardpoint contribution rankings for the selected tolerance state.

**Architecture:** Extend the existing tolerance model with a per-time-sample evaluator and exact subset/Shapley allocator. Reuse the shared J0/J1 page fragment to render total curves, per-point curves, and the peak-time ranking without changing existing tolerance-path analysis.

**Tech Stack:** Browser JavaScript, SVG, Node regression tests, static GitHub Pages.

**Spec:** `docs/superpowers/specs/2026-11-04-neck-dynamic-contribution-design.md`

## Global Constraints

- Motion is `q(t)=10°·sin(2πt)` over 0–1 s.
- Default 49 samples; at most 8 active points and 256 subsets.
- COM, nominal axes, zeroing logic, and existing parameter interfaces remain unchanged.
- Static and dynamic friction results remain separate.

## Review Focus

- Zero active points returns a valid zero series rather than throwing.
- One active point has 100% contribution at nonzero samples.
- Multiple-point signed allocations sum to total resistance increment at every sample.
- The final time sample closes the cycle with the initial sample.
- Motion reversal uses the correct positive/negative friction inputs.

---

### Task 1: Dynamic model API

**Files:**
- Modify: `src/neck-tolerance-model.js`
- Test: `tests/neck-tolerance-model.test.mjs`

**Interfaces:**
- Produces: `dynamicContribution(joint, offsets, parameters, loads, options)` returning time, angle, direction, baseline/current/delta resistance and per-point signed Shapley values for static and dynamic modes.

- [ ] Add failing tests for cycle closure, direction reversal, zero/single/multiple active points, Shapley conservation, and input immutability.
- [ ] Run `node tests/neck-tolerance-model.test.mjs` and confirm failure.
- [ ] Implement the minimal shared subset evaluation and time-sample allocation.
- [ ] Re-run the focused test and confirm pass.
- [ ] Build generated root model and commit.

### Task 2: Dynamic page module

**Files:**
- Modify: `src/neck-tolerance.fragment.html`
- Modify: `tests/neck-tolerance-pages.test.mjs`
- Generate: `neck-j0-tolerance.html`, `neck-j1-tolerance.html`, `neck-tolerance-model.js`

**Interfaces:**
- Consumes: `dynamicContribution(...)` from Task 1.
- Produces: 0–1 s static/dynamic charts, per-point signed contribution curves, peak-time cards and ranking tables.

- [ ] Add failing page assertions for dynamic controls, SVG charts, peak state and contribution tables.
- [ ] Run the page test and confirm failure.
- [ ] Add the responsive shared J0/J1 markup, renderer, legend and calculation action.
- [ ] Run page/model/neck regressions and confirm pass.
- [ ] Commit.

### Task 3: Verification and publishing

**Files:**
- Test: all `tests/*.test.mjs`

- [ ] Run all 27 regression tests.
- [ ] Browser-check zero, single-point and two-point cases on J0 and page load on J1.
- [ ] Verify peak-time ranking and Shapley conservation in displayed results.
- [ ] Push `main`, wait for GitHub Pages success, and verify both public URLs.

