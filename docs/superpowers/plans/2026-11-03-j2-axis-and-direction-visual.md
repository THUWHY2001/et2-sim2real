# J2 Axis and Direction Visual Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the real J2 equivalent rotation axis to clearance contribution calculations and replace the ambiguous two-color clearance graphic with a three-state contact explanation.

**Architecture:** Extend `J2ClearanceModel` with a direct angular-clearance source whose transmission coefficient is one. Keep passive radial and pushrod models unchanged, then update the existing SVG renderer and inputs to support J2 plus negative-transition-positive state cards.

**Tech Stack:** Buildless HTML/CSS/JavaScript, SVG, Node.js regression tests.

**Spec:** `docs/superpowers/specs/2026-11-03-j2-clearance-direction-visual-design.md`

## Global Constraints

- Use the name “J2等效旋转轴” everywhere.
- Model J2 with positive and negative angular clearance in degrees; do not infer radial-to-angular conversion without support geometry.
- Preserve the existing passive-joint and pushrod calculations.
- The three-state graphic is explanatory; numerical outputs use true entered values.

## Review Focus

- J2-only asymmetric input produces dead-zone equal to the two entered sides and the expected bias.
- Zero J2 input does not change existing combined results.
- Contribution shares still sum to 100 percent after adding J2.
- J2 rows use °/° while passive radial and pushrod rows use °/mm.
- Positive/negative animation direction highlights the matching state card.

---

### Task 1: J2 angular clearance source

**Files:**
- Modify: `src/j2-clearance-model.js`
- Test: `tests/j2-clearance-model.test.mjs`

**Interfaces:**
- Consumes: existing `analyze`, `metric`, `contactState`, and `presentation` APIs.
- Produces: `gaps.j2 = {positive, negative}`, `keys` including `j2`, and J2 presentation type `angular`.

- [ ] Add failing tests for J2-only asymmetric dead-zone/bias, contribution normalization, and angular presentation.
- [ ] Run the model test and confirm failure because J2 is absent.
- [ ] Implement direct angular offsets with unit transmission coefficient.
- [ ] Run the model test and confirm pass.

### Task 2: Inputs, ranking, and three-state visual

**Files:**
- Modify: `src/j2-clearance.fragment.html`
- Test: `tests/j2-clearance-page.test.mjs`

**Interfaces:**
- Consumes: J2 model input and presentation state from Task 1.
- Produces: J2 input row, J2 selector button, unit-aware ranking, and three labeled contact-state panels.

- [ ] Add failing page assertions for J2 inputs/selector and all three state labels.
- [ ] Rebuild and confirm the page test fails before markup exists.
- [ ] Implement the input and contribution display.
- [ ] Replace the magnifier with negative, transition, and positive state panels for radial, rod, and angular types.
- [ ] Rebuild and run model/page tests.
- [ ] Run the full test suite, desktop browser QA, and public deployment verification.
