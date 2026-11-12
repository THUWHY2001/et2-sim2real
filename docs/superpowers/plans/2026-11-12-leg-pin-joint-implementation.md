# ET2 左腿销轴运动副模型 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在保留旧球铰模型的同时新增 `pin-revolute-v1`，先完成 J5=0 与 J5=±5° 的独立闭链几何验收，再仅对通过域完成六维重力静力和虚功报告。

**Architecture:** 新模型以旧黑盒的输出角/推杆长度反解为基础，但用“推杆纵向轴 + 父刚体传递的销轴横向轴”独立重建六个推杆刚体姿态；六个端点从各自相连刚体分别生成 Marker 后验证 Revolute 约束。模型、报告和测试均使用独立文件与 `pin-revolute-v1` 标识，旧 `legacy-ball` 不变。

**Tech Stack:** 原生 JavaScript ES modules、Node.js `assert`/`node:test`、现有 `leg_full.mjs`、现有尺度化 SVD 与静力装配器。

**Spec:** `docs/superpowers/specs/2026-11-12-leg-pin-joint-design.md`

## Global Constraints

- 六处新销轴基准轴线暂设 +Y，并标记为基于现有几何、待最新 CAD 复核。
- 不覆盖或改写 `legacy-ball` 的运动学和历史报告。
- 不修改硬点，不在采样姿态重新定义销轴轴向，不投影不兼容姿态，不放宽容差使测试通过。
- J5=0 下每次只激励 J2/J3/J4 一个关节，范围为相对基准 ±10°，每轴不少于 121 个采样。
- J5=±5° 单独输出 P19/P21 销轴夹角和闭链残差；失败时限定模型有效域，不强制闭合。
- 新静力矩阵从头计算秩、左右零空间和分量唯一性；不得继承旧模型右零空间维数。
- 非唯一反力输出 `null`；运动副反力不自动解释为接触法向载荷。
- 本计划不实现摩擦圆。

## Review Focus

- 只用端点位置构造姿态导致“必然闭合”：Task 2 从 I/J 两侧刚体独立生成 Marker 并检查位置、轴向和允许相对运动。
- J5 Roll 使 P19/P21 销轴不平行：Task 3 对 ±5° 输出实际夹角且禁止自动修正。
- 世界坐标最小旋转再次引入 J1 Yaw 质心漂移：Task 2 用 +1°/-1° Yaw 验证全部随动刚体的刚体协变性与质心高度。
- 六个 Revolute 增加未知量后继续沿用旧唯一性：Task 4 断言新矩阵列数、重算秩和左右零空间，并逐分量投影。
- 双模型结果串用：Task 1/3/4 的所有产物带模型 ID，旧报告路径不被修改。

---

### Task 1: 销轴模型变体与新旧运动副映射

**Files:**
- Create: `web2/src/leg-pin-model.mjs`
- Create: `web2/tests/leg-pin-model.test.mjs`

**Interfaces:**
- Consumes: `web2/data/leg-full-package/model.json`。
- Produces: `PIN_JOINT_MAPPINGS`；`createPinJointModel(legacyModel)` 返回独立克隆，含 6 个新 Revolute、模型 ID、轴向假设与 CAD 复核状态。

- [ ] **Step 1: 写失败测试**

断言旧模型不变；六个旧 Spherical 被一一映射为新 ID 的 Revolute；新模型总运动副仍为 25；每个新副的 I/J 刚体、硬点和 +Y 基准轴正确；`axis_source="geometry_assumption_pending_cad"`；三个 Translational 未变化。

- [ ] **Step 2: 运行测试确认 RED**

Run: `node web2/tests/leg-pin-model.test.mjs`
Expected: FAIL，模块不存在。

- [ ] **Step 3: 实现模型克隆与映射**

不修改输入对象；新 Revolute Marker 名称独立，基准 I/J 轴均为 `[0,1,0]`，同时保存 `legacy_joint_id`。

- [ ] **Step 4: 运行测试确认 GREEN 并提交**

Run: `node web2/tests/leg-pin-model.test.mjs`

```bash
git add web2/src/leg-pin-model.mjs web2/tests/leg-pin-model.test.mjs
git commit -m "Add leg pin-joint model variant"
```

### Task 2: 销轴约束下的推杆姿态与独立 Marker 验证

**Files:**
- Create: `web2/src/leg-pin-runtime.mjs`
- Create: `web2/tests/leg-pin-runtime.test.mjs`

**Interfaces:**
- Consumes: `createLegFullRuntime(legacyModel)` 的角度/长度解和 Task 1 的 pin model。
- Produces: `createLegPinRuntime(legacyModel)`；`solveRods()`、`solveOutput()`、`signedRates()`、`gravityHoldingTorque()`；每帧包含 `pin_constraint_checks`。

- [ ] **Step 1: 写失败测试**

覆盖基准姿态：六个端点从父件和推杆件分别变换后的 Marker 中心重合；I/J 轴平行；三个移动副仍可轴向伸缩且无内部相对转动；模型 ID 为 `pin-revolute-v1`。

同时断言测试不是“共同端点自证”：人为扰动任一推杆侧 Marker 局部位置或轴向后，位置/轴向检查必须失败，运行时不得重新投影回兼容值。

- [ ] **Step 2: 运行测试确认 RED**

Run: `node web2/tests/leg-pin-runtime.test.mjs`
Expected: FAIL，运行时不存在。

- [ ] **Step 3: 实现协变推杆坐标系**

以当前上下端中心连线为纵向轴，以上端父刚体传递的 +Y 为横向参考构建正交刚体坐标系；外筒锚定上端、伸缩杆锚定下端，两者共享旋转矩阵。分别由 pin model 的 I/J Marker 局部数据生成世界 Marker 后检查约束，不覆盖检查结果。

- [ ] **Step 4: 增加 J1 独立运动学失败测试**

只令 J1=±1°：J2/J3/J4/J5/toe 和三根推杆长度不变；除 link1 外全部随动刚体必须等于绕世界 Z 的刚体旋转；每个质心高度变化小于 `1e-9 mm`，特别检查 `leg_l_link18`。

- [ ] **Step 5: 完成 J1 协变实现并确认 GREEN**

Run: `node web2/tests/leg-pin-runtime.test.mjs`
Expected: PASS。

- [ ] **Step 6: 提交**

```bash
git add web2/src/leg-pin-runtime.mjs web2/tests/leg-pin-runtime.test.mjs
git commit -m "Build covariant leg pin-joint kinematics"
```

### Task 3: 第一阶段全行程几何验收与冲突报告

**Files:**
- Create: `web2/scripts/report-leg-pin-kinematics.mjs`
- Create: `web2/tests/leg-pin-kinematics-report.test.mjs`
- Create: `web2/data/leg-full-package/pin_revolute_v1_kinematics_report.json`
- Create: `web2/data/leg-full-package/pin_revolute_v1_kinematics_report.csv`

**Interfaces:**
- Consumes: Task 2 的 pin runtime。
- Produces: 121 点×3 轴的 J5=0 扫描、J5=±5° 冲突工况、有效范围和逐运动副残差。

- [ ] **Step 1: 写失败测试**

断言报告含新旧映射、轴向假设、基准姿态、J2/J3/J4 各 121 点 ±10° 扫描；每个样本含 6 个 Revolute 的位置残差/无向轴角、3 个 Translational 的横向残差；不出现姿态投影或容差修复字段。

J5=±5° 必须报告 P19/P21 轴夹角和明确 pass/fail。若失败，`valid_scope` 只能包含 J5=0 下已通过的 J2/J3/J4 范围，`full_leg_motion_valid=false`。

- [ ] **Step 2: 运行测试确认 RED**

Run: `node web2/tests/leg-pin-kinematics-report.test.mjs`
Expected: FAIL，报告脚本不存在。

- [ ] **Step 3: 实现扫描和报告生成**

约束容差沿用源模型精度并在报告中显式列出；任一超限样本保留真实数值、硬点、运动副和失败原因。

- [ ] **Step 4: 生成报告并确认 GREEN**

Run: `node web2/scripts/report-leg-pin-kinematics.mjs`

Run: `node web2/tests/leg-pin-kinematics-report.test.mjs`

- [ ] **Step 5: 第一阶段门**

若 J5=0 的基准或任一 ±10° 样本失败，停止，不执行 Task 4；提交冲突报告。J5=±5° 失败不阻止限定 J5=0 的 Task 4，但必须限制声明范围。

- [ ] **Step 6: 提交**

```bash
git add web2/scripts/report-leg-pin-kinematics.mjs web2/tests/leg-pin-kinematics-report.test.mjs web2/data/leg-full-package/pin_revolute_v1_kinematics_report.json web2/data/leg-full-package/pin_revolute_v1_kinematics_report.csv
git commit -m "Report leg pin-joint kinematic compatibility"
```

### Task 4: 第二阶段新版静力矩阵与物理验收

**Files:**
- Modify: `web2/src/leg-gravity-statics.mjs`
- Create: `web2/tests/leg-pin-gravity-statics.test.mjs`
- Create: `web2/scripts/report-leg-pin-gravity-statics.mjs`
- Create: `web2/data/leg-full-package/pin_revolute_v1_gravity_statics_report.json`
- Create: `web2/data/leg-full-package/pin_revolute_v1_gravity_statics_report.csv`

**Interfaces:**
- Consumes: Task 1 pin model、Task 2 pin runtime、Task 3 `valid_scope`。
- Produces: 通过域基准姿态的 20 刚体静力解、131 级未知量矩阵诊断、25 运动副分量唯一性、三推杆保持力和 J1/J2/J3/J4 虚功结果。

- [ ] **Step 1: 写失败测试**

断言装配含 120 条平衡方程和 25 个运动副；相对旧模型恰增加 12 个端点 Revolute 被动扳手未知量，预计列数 131。测试不得预设右零空间等于 11，只断言 `right_nullity=columns-rank`、`left_nullity=rows-rank`，并逐分量验证零空间投影和 `null` 屏蔽。

- [ ] **Step 2: 运行测试确认 RED**

Run: `node web2/tests/leg-pin-gravity-statics.test.mjs`
Expected: FAIL，新模型尚未接入静力报告。

- [ ] **Step 3: 接入通用静力装配与新运动副类型**

不改变旧报告；pin model 的六个端部 Revolute 各用 5 维被动扳手，三个 Translational 保持原接口。输出左右零空间、分单位残差、内部作用反作用和整体平衡。

- [ ] **Step 4: 增加物理验收测试**

基准姿态必须检查：20 刚体六维平衡、矩阵秩稳定性、零/双倍重力线性、三推杆保持力、J1 竖直 Yaw 势能微分、J2/J3/J4 总广义推杆力矩唯一且通过虚功门。任何精确静力不相容均阻断理论反力进入后续摩擦。

- [ ] **Step 5: 生成报告并确认 GREEN**

Run: `node web2/scripts/report-leg-pin-gravity-statics.mjs`

Run: `node web2/tests/leg-pin-gravity-statics.test.mjs`

- [ ] **Step 6: 完整回归与模型隔离检查**

Run: `node --test 'web2/tests/*.test.mjs'`
Expected: 全部 PASS，且 legacy-ball 的 JSON/CSV 内容未被新脚本修改。

- [ ] **Step 7: 提交**

```bash
git add web2/src/leg-gravity-statics.mjs web2/tests/leg-pin-gravity-statics.test.mjs web2/scripts/report-leg-pin-gravity-statics.mjs web2/data/leg-full-package/pin_revolute_v1_gravity_statics_report.json web2/data/leg-full-package/pin_revolute_v1_gravity_statics_report.csv
git commit -m "Solve leg pin-joint gravity statics"
```
