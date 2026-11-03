# J2 Clearance Leadership View Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add static one-factor clearance sweeps, sensitivity and contribution rankings, and a management summary to the J2 clearance page.

**Architecture:** Add a pure `screening(input)` API to the existing model that isolates each source, sweeps it from 0–100%, and searches the ±10° pose range for peak output dead-zone. Render the returned data as static SVG curves, ranking bars, summary cards, and a table; collapse the existing animation.

**Tech Stack:** Buildless HTML/CSS/JavaScript, SVG, Node.js tests.

**Spec:** `docs/superpowers/specs/2026-11-03-j2-clearance-leadership-view-design.md`

## Global Constraints

- Each factor is evaluated alone with all other gaps zero.
- Sweep both sides proportionally from zero to the entered values.
- Keep angular and millimetre sensitivity units explicit.
- Use peak dead-zone over −10° to +10° as the cross-factor impact value.
- Keep the animated diagram as collapsed secondary content.

## Review Focus

- Screening is independent of unrelated input gaps.
- A zero input produces a zero curve, zero impact, and zero contribution.
- J2 direct angular sensitivity remains exactly 1 °/°.
- Contribution shares sum to 100% when any input is nonzero.
- Curve endpoints equal the reported 100% impact.

---

### Task 1: Single-factor screening model

**Files:**
- Modify: `src/j2-clearance-model.js`
- Test: `tests/j2-clearance-model.test.mjs`

**Interfaces:**
- Produces: `J2ClearanceModel.screening(input)` with `items`, each containing `curve`, `totalGap`, `unit`, `sensitivity`, `impact`, `peakAngle`, and `contribution`.

- [ ] Write failing tests for J2 unit sensitivity, zero factor, independence, curve endpoint, and contribution sum.
- [ ] Run the model test and confirm failure because `screening` is absent.
- [ ] Implement isolated factor scaling and pose search.
- [ ] Run the model test and confirm pass.

### Task 2: Static decision view

**Files:**
- Modify: `src/j2-clearance.fragment.html`
- Test: `tests/j2-clearance-page.test.mjs`

**Interfaces:**
- Consumes: `J2ClearanceModel.screening(input)`.
- Produces: management summary, sensitivity ranking, contribution ranking, per-factor curves, and summary table.

- [ ] Write failing page assertions for the static decision components and collapsed animation.
- [ ] Rebuild and confirm expected failure.
- [ ] Implement static rendering and recomputation on input changes.
- [ ] Run model/page tests, full suite, browser QA, and deploy.
