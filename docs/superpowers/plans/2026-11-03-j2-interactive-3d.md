# J2 Interactive 3D Mechanism Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the J2 resistance page's 2D mechanism drawing with a rotatable, zoomable 3D hard-point visualization synchronized to the existing motion controls.

**Architecture:** Add a focused dependency-free Canvas renderer that consumes the existing J2 solved geometry plus full Y-axis hard-point data. The J2 page owns view controls and passes every solved pose to the renderer; all resistance and sensitivity calculations remain untouched.

**Tech Stack:** Vanilla JavaScript, Canvas 2D, HTML/CSS, Node test runner

**Spec:** `docs/superpowers/specs/2026-11-03-j2-interactive-3d-design.md`

## Global Constraints

- Only the J2 resistance page changes; the clearance page remains unchanged.
- No third-party runtime dependency.
- Existing resistance, contribution and tolerance calculations must remain numerically unchanged.
- Support pointer drag, touch drag, wheel zoom, X–Z side view and reset view.

## Review Focus

- High-DPI and resized canvases remain sharp and correctly scaled.
- Pointer cancellation cannot leave the view stuck in drag mode.
- Wheel zoom is clamped and does not scroll the page while over the canvas.
- Labels remain readable at ±10° and at supported zoom limits.
- Existing play/pause and length-variation controls continue to refresh the diagram.

---

### Task 1: Canvas 3D projection and interaction module

**Files:**
- Create: `src/j2-3d-view.js`
- Test: `tests/j2-3d-view.test.mjs`

**Interfaces:**
- Consumes: J2 solved point object `{j2, axis1, axis2, axis3, point2Rod, strokeEnd, lower}` in millimetres.
- Produces: `window.J2ThreeView.create(canvas, options)` returning `{render(points), sideView(), resetView(), destroy()}`.

- [ ] Write failing tests for module export, projection, zoom clamping and required view methods.
- [ ] Run `node --test tests/j2-3d-view.test.mjs` and verify failure.
- [ ] Implement the dependency-free 3D renderer and pointer/wheel interaction.
- [ ] Run the focused test and verify pass.

### Task 2: Integrate the 3D view into J2

**Files:**
- Modify: `src/j2.fragment.html`
- Modify: `site.css`
- Modify: `scripts/build-pages.mjs`
- Test: `tests/j2-direct-page.test.mjs`

**Interfaces:**
- Consumes: `window.J2ThreeView.create(...)` from Task 1 and existing `J2Model.solve(...)` output.
- Produces: a labeled Canvas panel synchronized with the J2 slider, playback loop and length controls.

- [ ] Add failing page assertions for Canvas, view buttons, script inclusion and absence of the old main SVG.
- [ ] Run `node --test tests/j2-direct-page.test.mjs` and verify failure.
- [ ] Replace the main SVG with Canvas markup and view controls while retaining the coordinate key.
- [ ] Connect the existing render cycle to `threeView.render(points)` and add X–Z/reset actions.
- [ ] Add responsive Canvas/control styling and copy the new module during builds.
- [ ] Run `node scripts/build-pages.mjs` and the focused page test.

### Task 3: Regression and browser verification

**Files:**
- Generated: `j2.html`, `j2-3d-view.js`, `dist/j2.html`, `dist/j2-3d-view.js`

**Interfaces:**
- Consumes: completed Tasks 1–2.
- Produces: deployable static J2 page.

- [ ] Run all Node tests and confirm all pass.
- [ ] Serve the site locally and verify rotation, zoom, side view, reset, playback and responsive layout.
- [ ] Confirm resistance totals and contribution charts match the pre-change implementation at representative angles.
- [ ] Commit the verified implementation.

