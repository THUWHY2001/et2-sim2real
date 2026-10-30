# Arm J5 Oblique View Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a reference-like 3D oblique mechanism view to Arm J5 while retaining the X–Z engineering side view.

**Architecture:** Keep the dynamics model unchanged. Add a view selector and route all SVG point placement through a view-dependent projection function.

**Tech Stack:** Buildless HTML, SVG, JavaScript, Node regression tests.

**Spec:** `docs/superpowers/specs/2026-10-30-arm-j5-oblique-view-design.md`

## Global Constraints

- Projection affects display only.
- Default view is 3D oblique; X–Z remains selectable.
- Existing animation and link-length controls continue to redraw the diagram.

## Review Focus

- Selector changes every displayed point and line without changing computed rod lengths.
- Default view uses the Y coordinate rather than collapsing to X–Z.
- Labels remain readable in both views.

### Task 1: View selector and projection

**Files:**
- Modify: `src/arm-j5.fragment.html`
- Modify: `tests/arm-j5-page.test.mjs`

**Interfaces:**
- Consumes: `ArmJ5Model.solve(angle, lengths).points`
- Produces: `#arm-j5-view` with values `oblique` and `xz`

- [ ] Add a failing page regression test for the selector and Y-aware projection.
- [ ] Run the focused test and confirm failure.
- [ ] Add the selector, oblique projection, X–Z fallback, and redraw listener.
- [ ] Rebuild and run the focused test and full suite.
- [ ] Browser-check both views and publish.

