# ET2 Leg Planar Friction Circle Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an isolated `pin-planar-ideal-v1` theoretical friction-circle module with traceable physical contacts, signed multi-output virtual-work conversion, reports, CSV export, and an interactive page.

**Architecture:** Keep the accepted planar kinematics/statics immutable. Add a contact catalog keyed by both `joint_id` and `physical_contact_id`, a pure friction engine consuming accepted statics, a report/export layer, then a thin browser UI. Use central finite differences as the primary relative-rate Jacobian and an output-aligned planar constraint nullspace as an independent check.

**Tech Stack:** Native JavaScript ES modules, Node test runner, static HTML/CSS/SVG, existing ET2 planar model/kinematics/statics modules.

**Spec:** `docs/superpowers/specs/2026-11-13-leg-planar-friction-circle-design.md`

## Global Constraints

- Do not modify legacy-ball, `pin-revolute-v1`, `pin-planar-ideal-v1` kinematics, or its statics solver.
- Use only `pin-planar-ideal-v1` accepted gravity-quasi-static loads; never use failed 3D reactions.
- Current load mode is `gravity_only`; `gravity_plus_inertia` is an unsupported reserved interface.
- Missing friction parameters produce no fabricated numeric contribution.
- P10/P18 remain visible and load-reportable but default disabled with `real_joint_type_unverified`.
- Prismatic contacts remain visible but excluded from the official first-version total.
- A total means enabled, fully modelled physical contacts only, not true whole-leg running resistance.

## Review Focus

- Co-located P13/P16/P21 interfaces must not merge or duplicate one physical load across unrelated contacts.
- Angle unwrap around ±π must not spike the relative-rate Jacobian.
- Multi-output motion must satisfy full power equality, not a single-axis magnitude shortcut.
- Reversal/stick candidates must not acquire a fabricated kinetic friction sign.
- Manual and split-bearing loads must not double-count the automatic joint reaction.

---

### Task 1: Physical contact catalog

**Files:**
- Create: `web2/src/leg-pin-planar-friction-catalog.mjs`
- Test: `web2/tests/leg-pin-planar-friction-catalog.test.mjs`

**Interfaces:**
- Consumes: `createLegPinPlanarModel(legacyModel)`.
- Produces: `createLegPinPlanarFrictionCatalog(model) -> {contacts, byContactId, byJointId, audit}` and `validateContactConfig(config)`.

- [ ] Write a failing catalog test asserting all 25 joint IDs are classified, 13 pin candidates have unique `physical_contact_id`, P13/P16 co-located contacts remain distinct, fixed/held/prismatic classifications are excluded, and P10/P18 are disabled/unverified.
- [ ] Run `node --test web2/tests/leg-pin-planar-friction-catalog.test.mjs` and confirm failure because the module is missing.
- [ ] Implement the immutable catalog and validation for parameter ranges, unique physical IDs, side-load fractions summing to one, and unsupported contact models.
- [ ] Run the catalog test and the full existing test suite.
- [ ] Commit the catalog and test.

### Task 2: Relative-rate Jacobian and independent cross-check

**Files:**
- Create: `web2/src/leg-pin-planar-friction-kinematics.mjs`
- Test: `web2/tests/leg-pin-planar-friction-kinematics.test.mjs`

**Interfaces:**
- Consumes: planar kinematic solutions and Task 1 contacts.
- Produces: `relativeRateJacobian(solution, contact)`, `relativeVelocity(solution, contact, outputRates)`, and `crossCheckRelativeRates(solution)`.

- [ ] Write failing tests for output joints' identity rates, fixed/held zero rates, angle unwrap stability, central-difference convergence, complete `omega = g*qdot`, and nullspace cross-check tolerance.
- [ ] Run the focused test and confirm the missing-module failure.
- [ ] Implement central-difference relative body-angle derivatives and prismatic displacement derivatives without changing the accepted kinematics module.
- [ ] Implement the independent output-aligned constraint-nullspace cross-check and return per-contact residuals rather than silently relaxing tolerance.
- [ ] Run focused and full suites, then commit.

### Task 3: Friction engine and virtual-work validation

**Files:**
- Create: `web2/src/leg-pin-planar-friction.mjs`
- Test: `web2/tests/leg-pin-planar-friction.test.mjs`

**Interfaces:**
- Consumes: catalog, accepted planar statics, Task 2 relative rates, parameter/load overrides.
- Produces: `createLegPinPlanarFriction(model, kinematics, statics, options)` with `evaluatePose`, `evaluateTrajectory`, and `serializeParameters`.

- [ ] Write failing tests for μ=0, missing parameters, automatic `planar_gravity_estimate`, P10/P18 exclusion, prismatic exclusion, double-bearing allocation, manual override precedence, reversal null kinetic torque, sign reversal, nonnegative dissipation, single-contact virtual work, and multi-output power equality.
- [ ] Run the focused test and confirm failure because the engine is absent.
- [ ] Implement load resolution, per-side allocation, `mu*N*r`, static capacity, visual-only friction-circle radius, generalized torque vector, dissipated power, and contribution shares.
- [ ] Reject `gravity_plus_inertia` with `unsupported_load_mode` while preserving the API field.
- [ ] Run focused and full suites, then commit.

### Task 4: Traceable JSON/CSV report

**Files:**
- Create: `web2/scripts/report-leg-pin-planar-friction.mjs`
- Create: `web2/data/leg-full-package/pin_planar_ideal_v1_friction_report.json`
- Create: `web2/data/leg-full-package/pin_planar_ideal_v1_friction_report.csv`
- Test: `web2/tests/leg-pin-planar-friction-report.test.mjs`

**Interfaces:**
- Consumes: Tasks 1–3.
- Produces: `buildReport(parameters)`, `serializeCsv(report)`, and generated baseline/1 Hz trajectory artifacts.

- [ ] Write a failing report test asserting model/load boundaries, all contact classifications, provenance, 121 samples per output, reversal states, power/virtual-work checks, and CSV units/source columns.
- [ ] Run the focused test and confirm failure.
- [ ] Implement report generation with a clearly marked non-factual example parameter set used only for software verification; production defaults remain null.
- [ ] Generate JSON/CSV and run focused/full tests.
- [ ] Commit reports, generator, and tests.

### Task 5: Interactive theoretical friction page

**Files:**
- Create: `web2/src/leg-pin-planar-friction.fragment.html`
- Modify: `web2/scripts/build-pages.mjs`
- Modify: `web2/site.css`
- Create: `web2/tests/leg-pin-planar-friction-page.test.mjs`
- Generated: `web2/leg-friction.html`, `web2/dist/leg-friction.html`, `sites/robot-joint-sensitivity-publish/leg-friction.html`

**Interfaces:**
- Consumes: Tasks 1–3 in the browser.
- Produces: output/pose selectors, contact map, parameter editor, current/cycle results, friction-circle SVG, and CSV download.

- [ ] Write a failing page test for build registration, leg navigation, model warnings, contact IDs, P10/P18 status, load mode, parameter controls, current/cycle charts, and CSV export.
- [ ] Run the page test and confirm failure.
- [ ] Add the fragment, page builder entry, required module copies/data copies, navigation link, and restrained engineering CSS.
- [ ] Implement live recalculation without default factual μ/r values; provide an explicitly labelled example-parameter button and reset-to-missing action.
- [ ] Build pages, run page and full suites, inspect generated HTML, then commit.

### Task 6: Whole-module acceptance

**Files:**
- Modify only if acceptance exposes a defect: files owned by Tasks 1–5.

**Interfaces:**
- Consumes: complete module.
- Produces: final verification evidence and an unchanged legacy-model audit.

- [ ] Run all `web2/tests/*.test.mjs` and capture pass/fail count.
- [ ] Run the report generator and verify μ=0, single-contact, forward/reverse, reversal, multi-output power, and physical-contact uniqueness summaries.
- [ ] Run `git diff --check` and verify legacy model/solver files are unchanged from the phase baseline.
- [ ] Review every requirement in the design against code/tests and record any ruling or deferred limitation.
- [ ] Commit any test-first acceptance fix, otherwise leave the verified head unchanged.

