# ET2 Torso Closed-Chain Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add verified J0 Yaw, J1 Roll, J2 Pitch and combined-input ET2 torso pages using the real dual-crank spherical-link closed chain, with 1 Hz ±10° output tests and quasi-static resistance contribution analysis.

**Architecture:** A pure browser JavaScript model owns source hardpoints, rigid transforms, damped-Newton closure solving, numerical pose Jacobians and virtual-work resistance mapping. Shared HTML fragments render the three output-joint pages and a separate true-actuator combination page; a lightweight SVG camera renders all points from the solved rigid-body state. Generated root and `dist/` pages remain products of `scripts/build-pages.mjs`.

**Tech Stack:** Buildless HTML/CSS/JavaScript, Node.js regression tests with `assert` and `vm`, SVG 3D projection, GitHub Pages.

**Spec:** `docs/superpowers/specs/2026-11-04-torso-closed-chain-design.md`

## Global Constraints

- Use the ZIP hardpoints and rigid-body ownership; do not invent constraints or geometry.
- Define both crank angles by the initial parent-frame `+Y` right-hand rule, regardless of the right ADAMS marker's `-Y` axis.
- J0/J1/J2 tests target output amplitude `±10°` at `1 Hz`; solve crank motion from the closed chain.
- J1 uses opposite crank angles and solves companion Pitch; J2 uses equal crank angles and solves companion Roll.
- Combined mode accepts Yaw, left crank and right crank as the only active inputs and solves Roll/Pitch.
- Do not use the initial Jacobian as a finite-motion ratio.
- Do not add gravity, inertia, contact, flexibility, clearance or tolerance analysis in this phase.
- A failed closure solve must be visible and must not produce a fabricated animation frame.
- Static and dynamic friction remain separate and support positive/negative output directions.

## Review Focus

- Assembly branch continuity: a full ±10° cycle must return to the original branch without angle jumps.
- Near-singular Newton steps: the solver must damp or fail explicitly rather than returning large finite-looking angles.
- Unified right-crank sign: zero-pose numerical slopes must match the normalized source Jacobian.
- Spherical-joint resistance: calculate only observable swing articulation, never an unobservable rod-spin term.
- Generated-page drift: source fragment, root pages and `dist/` pages must contain the same model version and controls.

---

### Task 1: Source geometry and rigid transforms

**Files:**
- Create: `src/torso-model.js`
- Create: `tests/torso-model.test.mjs`

**Interfaces:**
- Produces: `window.TorsoModel.points`, `window.TorsoModel.lengths`, `window.TorsoModel.zeroState()`, `window.TorsoModel.worldGeometry(state)`.
- `state` fields use degrees at the public boundary: `{yaw, roll, pitch, crankL, crankR}`.

- [ ] **Step 1: Write the failing geometry test**

Assert the seven core point coordinates, both `69.9970813773 mm` rod lengths, both `41.7710715402 mm` crank lengths, zero-state closure, and fixed rigid distances from each crank pivot to its upper ball.

- [ ] **Step 2: Run the test and verify RED**

Run: `node tests/torso-model.test.mjs`

Expected: FAIL because `src/torso-model.js` does not exist.

- [ ] **Step 3: Implement source constants and transforms**

Add vector/matrix primitives, `Rx/Ry/Rz`, the serial upper-platform orientation `Rx(roll)Ry(pitch)`, unified `+Y` crank rotation, Yaw world transform about `P_waist_link1`, and per-body point generation. Return left/right rod length errors and squared closure residuals with every geometry result.

- [ ] **Step 4: Run the test and verify GREEN**

Run: `node tests/torso-model.test.mjs`

Expected: PASS with zero-pose residuals below `1e-9 mm` equivalent.

- [ ] **Step 5: Commit**

```bash
git add src/torso-model.js tests/torso-model.test.mjs
git commit -m "Add torso closed-chain source geometry"
```

### Task 2: Nonlinear inverse and forward closure solvers

**Files:**
- Modify: `src/torso-model.js`
- Modify: `tests/torso-model.test.mjs`

**Interfaces:**
- Produces: `solveJoint(joint, outputDegrees, previousState?)` for `J0|J1|J2`.
- Produces: `solveActive({yaw, crankL, crankR}, previousState?)`.
- Both return `{converged, iterations, residual, state, geometry}`; non-converged results have no accepted geometry.

- [ ] **Step 1: Write failing tests for the three joint modes**

Test J0 at ±10° with zero internal relative motion. Test J1 at `-10, 0, +10°` with `crankR=-crankL` and solved companion Pitch. Test J2 at the same outputs with `crankR=crankL` and solved companion Roll. Assert both rod errors remain below `1e-6 mm`.

- [ ] **Step 2: Run and verify RED**

Run: `node tests/torso-model.test.mjs`

Expected: FAIL because the solver APIs are missing.

- [ ] **Step 3: Implement damped Newton with continuation**

Use centered numerical derivatives, a bounded line search, finite-value guards, iteration/residual limits, normalized angle deltas and the previous accepted state as the preferred initial guess. Implement the J1 unknown pair `(commonCrank, companionPitch)`, J2 pair `(commonCrank, companionRoll)`, and active-input unknown pair `(roll,pitch)`.

- [ ] **Step 4: Add failing branch and singularity tests**

Exercise a monotonic `0→+10→0→-10→0` trajectory and assert no crank/output jump exceeds a conservative adjacent-sample bound. Supply deliberately unreachable crank inputs and assert `converged=false` rather than a fabricated pose.

- [ ] **Step 5: Verify normalized initial Jacobian**

At a small output perturbation, assert J1 slopes approach `crankL/roll=-0.6878331741`, `crankR/roll=+0.6878331741`; assert J2 slopes approach `crankL/pitch=crankR/pitch=-1.1122594418` within the declared numerical tolerance.

- [ ] **Step 6: Run and verify GREEN**

Run: `node tests/torso-model.test.mjs`

Expected: PASS for all closure, branch, failure and Jacobian tests.

- [ ] **Step 7: Commit**

```bash
git add src/torso-model.js tests/torso-model.test.mjs
git commit -m "Solve torso dual-crank closed chain"
```

### Task 3: Pose rates, resistance mapping and cycle contribution

**Files:**
- Modify: `src/torso-model.js`
- Modify: `tests/torso-model.test.mjs`

**Interfaces:**
- Produces: `rates(joint, outputDegrees, previousState?)` with rates for `yaw`, `roll`, `pitch`, `crankL`, `crankR`, `ballLUpper`, `ballLLower`, `ballRUpper`, `ballRLower`.
- Produces: `equivalentResistance(joint, solution, loads, direction, mode)`.
- Produces: `cycle(joint, loads, {samples})` with samples, peak totals, integrated per-item torques and normalized static/dynamic contribution rankings.

- [ ] **Step 1: Write failing rate tests**

Assert J0 has unit Yaw rate and zero internal rates. Assert J1/J2 crank rates agree with the initial Jacobian and all four ball-swing rates are finite and nonnegative. Assert returned geometry still satisfies closure after differencing.

- [ ] **Step 2: Run and verify RED**

Run: `node tests/torso-model.test.mjs`

Expected: FAIL because rate APIs are missing.

- [ ] **Step 3: Implement centered pose derivatives**

Differentiate accepted solutions at `q±h`. For upper spherical joints, compare rod direction in the corresponding crank frame; for lower spherical joints, compare rod direction in the `waist_link1` frame. Use the norm of the direction derivative as swing angular rate and omit axial spin.

- [ ] **Step 4: Write failing resistance tests**

Use unequal positive/negative static/dynamic inputs. Assert J0 contribution is 100% Yaw; mode and direction change totals as entered; J1/J2 totals equal the sum of their nine nonnegative virtual-work parts; nonzero parts normalize to 100%.

- [ ] **Step 5: Implement virtual-work resistance**

Map each measured N·m input with `Ti*|dφi/dq|`. Keep static and dynamic calculations separate. Use the sign of the main output velocity only to select positive/negative measured data.

- [ ] **Step 6: Write failing 1 Hz cycle tests**

Assert 1 Hz samples follow `10 sin(2πt)`, reach both ±10° outputs, preserve closure, contain no branch jumps, and return integrated absolute contributions summing to 100% for both friction modes.

- [ ] **Step 7: Implement and verify cycle analysis**

Use at least 121 samples with continuation. Integrate absolute item torque by the trapezoidal rule. If any sample fails, return the failing time/index and do not publish a complete ranking.

- [ ] **Step 8: Run full model tests and commit**

Run: `node tests/torso-model.test.mjs`

Expected: PASS.

```bash
git add src/torso-model.js tests/torso-model.test.mjs
git commit -m "Add torso resistance contribution analysis"
```

### Task 4: Interactive 3D torso pages

**Files:**
- Create: `src/torso-3d-view.js`
- Create: `src/torso-joint.fragment.html`
- Create: `src/torso-combined.fragment.html`
- Create: `tests/torso-pages.test.mjs`
- Modify: `site.css`

**Interfaces:**
- `window.Torso3DView.createCamera(options)` supplies rotate, zoom, reset and project.
- Joint fragment placeholders: `__JOINT__`, `__MOTION__`, `__DESCRIPTION__`.
- Both fragments consume `window.TorsoModel` and render only accepted solver geometry.

- [ ] **Step 1: Write failing page contract tests**

Require a rotatable SVG, X/Y/Z axes, Yaw/Roll/Pitch axes, both cranks, four ball joints, both fixed-length rods, upper-platform reference geometry, reset-view control, solve status, rod errors, residual, output/coupling/crank values, directional static/dynamic inputs, instantaneous result and cycle ranking.

- [ ] **Step 2: Run and verify RED**

Run: `node tests/torso-pages.test.mjs`

Expected: FAIL because fragments and camera do not exist.

- [ ] **Step 3: Implement the camera and solved-geometry drawing**

Render left and right loops in distinct restrained colors. Bind pointer drag, wheel zoom and reset. Keep the last valid geometry on screen when a solve fails and show the failure state beside the animation.

- [ ] **Step 4: Implement J0/J1/J2 page behavior**

Add output slider, 1 Hz play/pause, real-time curve state, calculated crank angles, companion output, closure metrics, nine connection-pair load inputs and static/dynamic result tables. J0 must visibly report zero internal motion.

- [ ] **Step 5: Implement combined true-actuator behavior**

Add Yaw, left-crank and right-crank controls only; solve and display Roll/Pitch. Include presets for Yaw only, equal cranks and opposite cranks without labeling the latter two as exact pure output modes.

- [ ] **Step 6: Run page tests and commit**

Run: `node tests/torso-pages.test.mjs`

Expected: PASS.

```bash
git add src/torso-3d-view.js src/torso-joint.fragment.html src/torso-combined.fragment.html tests/torso-pages.test.mjs site.css
git commit -m "Add interactive torso closed-chain pages"
```

### Task 5: Build integration and module navigation

**Files:**
- Modify: `scripts/build-pages.mjs`
- Modify: `tests/engineering-shell.test.mjs`
- Modify: `tests/torso-pages.test.mjs`
- Generate: `torso-j0.html`, `torso-j1.html`, `torso-j2.html`, `torso-combined.html`, `torso-model.js`, `torso-3d-view.js`
- Generate: matching `dist/` files and updated `index.html`

**Interfaces:**
- Build script registers torso joint navigation and copies source assets to root and `dist/`.
- Home page replaces the torso placeholder with J0/J1/J2/组合 entries.

- [ ] **Step 1: Extend failing build tests**

Require the four torso URLs in the home page, a torso top-level navigation link to `torso-j0.html`, torso subnavigation on all four pages, correct script assets and no iframe/srcdoc embedding.

- [ ] **Step 2: Run and verify RED**

Run separately: `node tests/torso-pages.test.mjs`, then `node tests/engineering-shell.test.mjs`.

Expected: FAIL because generated pages are absent.

- [ ] **Step 3: Update the generator and build**

Register the torso fragments, asset copies, page transformations, navigation and home cards. Increment only the relevant torso model asset version.

Run: `node scripts/build-pages.mjs`

- [ ] **Step 4: Verify generated parity and GREEN**

Run separately: `node tests/torso-pages.test.mjs`, then `node tests/engineering-shell.test.mjs`.

Expected: PASS and generated root/`dist/` content matches source contracts.

- [ ] **Step 5: Commit**

```bash
git add scripts/build-pages.mjs tests/engineering-shell.test.mjs tests/torso-pages.test.mjs site.css src/torso-* torso-* dist index.html
git commit -m "Integrate torso module into engineering site"
```

### Task 6: Full verification, browser QA and publication

**Files:**
- Modify only if a failing test or observed defect requires a TDD-backed correction.

**Interfaces:**
- Produces a verified GitHub Pages deployment with the four torso URLs.

- [ ] **Step 1: Run the complete regression suite**

Run every `tests/*.test.mjs` file and report any pre-existing or new failure by name. Expected: all tests pass without warnings.

- [ ] **Step 2: Run numerical sweep QA**

Sweep J1/J2 across the complete ±10° range and representative combined crank pairs. Record maximum rod-length error, maximum residual, maximum iterations, output extrema and any branch discontinuity. Expected: accepted states meet the design tolerances.

- [ ] **Step 3: Read the local Sites preview reference and launch preview**

Use the established local static-server workflow. Do not start browser QA before reading `sites/references/preview/local.md`.

- [ ] **Step 4: Inspect all four pages in the browser**

Verify labels are unobstructed, drag/zoom works, J1 opposite cranks and J2 equal cranks are visually correct, rods do not change length, combined presets produce the expected dominant output, charts update, and solve failures remain visible.

- [ ] **Step 5: Re-run tests after any visual correction**

Expected: full suite remains green and the worktree is clean after commit.

- [ ] **Step 6: Push and verify GitHub Pages**

Push `main`, poll all four public URLs until the torso closed-chain marker is present, and verify HTTP success plus the published asset version.

- [ ] **Step 7: Final handoff**

Return the four public links, numerical closure maxima, test count, friction-model scope and the explicit limitation that gravity/inertia are not yet included.
