# ET2 腿足平面理论摩擦圆分析设计

## 目的

在已经验收的 `pin-planar-ideal-v1` 上增加独立的理论摩擦分析模块，用于比较各真实销轴—轴套接触副在重力准静态载荷下折算到 J2/J3/J4 的等效阻力。结果用于确定优先验证对象，不代表实测启动力矩或整腿真实运行阻力。

## 范围

- 仅使用 `pin-planar-ideal-v1` 的平面运动学和重力准静态反力。
- J5=0；J2/J3/J4 各自 1 Hz、±10° 单轴激励；计算内核同时支持完整的三输出速度向量。
- 不读取 `pin-revolute-v1` 的未验收三维反力。
- 不修改旧球副模型、三维销轴模型、平面运动学或平面静力求解器。
- 第一版不包含惯性、装配预紧、摩擦对反力分配的反馈或三维支承载荷分配。
- 移动副保留独立参数和后续接触载荷接口，但不进入第一版正式总摩擦估算。

## 方案选择

采用中心差分计算每个连接副的相对运动雅可比，约束零空间只作交叉验证。

- 主算法：分别扰动 J2/J3/J4，计算两个连接刚体的相对角或移动量，得到局部速度对三个输出速度的雅可比。
- 交叉验证：将平面约束雅可比的三维零空间与 J2/J3/J4 输出方向对齐，核对相对速度雅可比。
- 未采用直接时序差分作为主算法，因为它依赖采样率，且换向点的数值符号不稳定。

## 25 个运动副与物理接触映射

映射必须同时使用 `joint_id` 和 `physical_contact_id`。同一坐标可以存在多个独立物理接触界面；只有确有独立接触且存在相对滑动时才分别计算。

| joint_id | 平面类型 | physical_contact_id | 第一版处理 |
|---|---|---|---|
| J_LEG_L_HIP_roll_RE | fixed boundary | leg.hip_roll.axis | J5=0 平面边界下无相对运动，不计运行摩擦 |
| J_LEG_L_HIP_yaw_RE | fixed boundary | leg.hip_yaw.axis | 平面边界下无相对运动，不计运行摩擦 |
| J_LEG_L_HIP_pitch_RE | revolute | leg.j2.output_pin | 销轴—轴套候选 |
| J_LEG_L_KNEE_pitch_RE | revolute | leg.j3.output_pin | 销轴—轴套候选 |
| J_LEG_L_ANKLE_pitch_RE | revolute | leg.j4.output_pin | 销轴—轴套候选 |
| J_LEG_L_TOE_pitch_RE | fixed boundary | leg.toe_pitch.axis | 保持状态，无相对运动，不计运行摩擦 |
| J_LEG_L_P009_RE_L02_L08 | revolute | leg.hip_rocker.p09 | 销轴—轴套候选 |
| J_LEG_L_P010_S_L09_L03 | planar revolute | leg.p10.unverified | 显示载荷；默认禁用，`real_joint_type_unverified` |
| J_LEG_L_HIP_pitch_T | prismatic | leg.hip_rod.guide | 移动副接口保留，默认不计正式估算 |
| J_LEG_L_P013_F_L8_L12 | fixed | leg.hip_rod.p13.carrier_fixed | 固定连接，不计算摩擦 |
| J_LEG_L_P013_RE_L08_L09 | revolute | leg.hip_rocker.p13 | 独立摇臂销轴候选 |
| J_LEG_L_KNEE_pitch_T | prismatic | leg.knee_rod.guide | 移动副接口保留，默认不计正式估算 |
| J_LEG_L_P016_F_L16_B15 | fixed | leg.knee_rod.p16.carrier_fixed | 固定连接，不计算摩擦 |
| J_LEG_L_P016_RE_L16_L17 | revolute | leg.knee_rocker.p16 | 独立摇臂销轴候选 |
| J_LEG_L_P017_RE_L03_L16 | revolute | leg.knee_rocker.p17 | 销轴—轴套候选 |
| J_LEG_L_P018_S_L17_L04 | planar revolute | leg.p18.unverified | 显示载荷；默认禁用，`real_joint_type_unverified` |
| J_LEG_L_ANKLE_pitch_T | prismatic | leg.ankle_rod.guide | 移动副接口保留，默认不计正式估算 |
| J_LEG_L_P021_F_L06_B20 | fixed | leg.ankle_rod.p21.carrier_fixed | 固定连接，不计算摩擦 |
| J_LEG_L_ANKLE_roll_RE | fixed boundary | leg.j5.roll_axis | J5=0 保持，无相对运动，不计运行摩擦 |
| J_LEG_L_P011_RE_L10_L03 | revolute | leg.hip_rod.p11.pin_bushing | 新销轴—轴套候选 |
| J_LEG_L_P013_RE_L12_L11 | revolute | leg.hip_rod.p13.pin_bushing | 与 P13 摇臂销轴是独立接触界面 |
| J_LEG_L_P014_RE_L03_L13 | revolute | leg.knee_rod.p14.pin_bushing | 新销轴—轴套候选 |
| J_LEG_L_P016_RE_B15_L14 | revolute | leg.knee_rod.p16.pin_bushing | 与 P16 摇臂销轴是独立接触界面 |
| J_LEG_L_P019_RE_L18_L04 | revolute | leg.ankle_rod.p19.pin_bushing | 新销轴—轴套候选 |
| J_LEG_L_P021_RE_B20_L19 | revolute | leg.ankle_rod.p21.pin_bushing | 与 P21 Fixed 辅助连接不同；仅此销轴计算滑动摩擦 |

候选并不等于自动产生数值。`mu_static`、`mu_kinetic` 或 `effective_radius_m` 缺失时，接触副状态为 `missing_parameters`，不进入合计。

## 数据结构

### 接触副配置

```js
{
  joint_id: "J_LEG_L_P013_RE_L12_L11",
  physical_contact_id: "leg.hip_rod.p13.pin_bushing",
  display_name: "髋推杆 P13 下端销轴—轴套",
  body_i: "leg_l_ball_link12",
  body_j: "leg_l_link11",
  contact_model: "pin_bushing_coulomb",
  verification_status: "geometry_assumption_pending_cad",
  enabled: true,
  include_in_official_total: true,
  parameters: {
    mu_static: null,
    mu_kinetic: null,
    effective_radius_m: null,
    provenance: "user_input"
  },
  bearing_sides: [
    { side_id: "A", load_fraction: 0.5 },
    { side_id: "B", load_fraction: 0.5 }
  ],
  load: {
    mode: "automatic",
    source: "planar_gravity_estimate",
    manual_override_N: null
  }
}
```

`bearing_sides[].load_fraction` 之和必须为 1。同一总载荷按比例分配后分别计算，禁止每侧重复使用总载荷。若两侧参数不同，可以在侧级覆盖 `mu_static`、`mu_kinetic` 和 `effective_radius_m`。

P10/P18 使用 `verification_status: "real_joint_type_unverified"`、`enabled: false`、`include_in_official_total: false`。

移动副使用独立结构：

```js
{
  contact_model: "prismatic_guide_coulomb",
  enabled: false,
  include_in_official_total: false,
  guide_contacts: [],
  required_geometry: ["support_spacing_m", "guide_direction", "preload_N"],
  load_status: "guide_contact_distribution_unresolved"
}
```

不得直接用静力解的 `F_normal` 代替导向接触 N1/N2；约束力矩、支承间距和预紧必须进入后续载荷分配。

### 时刻结果

```js
{
  time_s: 0.125,
  load_mode: "gravity_only",
  outputs_rad: { J2: 0, J3: 0, J4: 0 },
  output_rates_rad_s: { J2: 1, J3: 0, J4: 0 },
  contacts: [{
    joint_id: "...",
    physical_contact_id: "...",
    load_N: 120.0,
    load_source: "planar_gravity_estimate",
    relative_rate_jacobian: { J2: 0.8, J3: 0.0, J4: 0.0 },
    relative_velocity_rad_s: 0.8,
    motion_state: "sliding",
    kinetic_torque_Nm: -0.12,
    static_limit_Nm: 0.18,
    friction_circle_radius_m: 0.00099,
    equivalent_output_torque_Nm: { J2: -0.096, J3: 0, J4: 0 },
    dissipated_power_W: 0.096,
    status: "computed"
  }],
  totals: {
    included_contact_count: 1,
    equivalent_output_torque_Nm: { J2: -0.096, J3: 0, J4: 0 },
    dissipated_power_W: 0.096,
    definition: "enabled_and_fully_modelled_contacts_only"
  }
}
```

## 相对速度和虚功

对销轴接触副：

\[
g_{ik}=\frac{\partial(\theta_j-\theta_i)}{\partial q_k},\qquad
\omega_i=\sum_{k\in\{J2,J3,J4\}}g_{ik}\dot q_k
\]

中心差分的角度必须先做相位展开，避免跨越 ±π。独立零空间校验需将 57×60 平面约束雅可比的三维零空间与 J2/J3/J4 有限差分速度方向对齐，再比较每个接触副的相对角速度。

动摩擦仅在 `|omega_i| > velocity_epsilon` 时定义：

\[
T_i=-\operatorname{sign}(\omega_i)\mu_{k,i}N_ir_i
\]

多输出虚功折算：

\[
Q_k=\sum_i g_{ik}T_i,\qquad
P_i=T_i\omega_i\le0,\qquad
\sum_k Q_k\dot q_k=\sum_iP_i
\]

单轴贡献显示使用非负耗散等效阻力：

\[
R_{ik}=|g_{ik}|\mu_{k,i}N_ir_i
\]

多轴同时运动时不把单轴 `R_ik` 相加为总阻力；页面显示广义阻力向量 `Q`、总耗散功率和按 `-P_i` 计算的功率贡献占比。

换向或黏着候选状态不输出确定动摩擦力矩。仅输出局部静摩擦容量 `mu_static*N*r`，以及针对指定虚拟运动方向得到的输出静摩擦容量包络；不将其称为实际静摩擦力矩。

摩擦圆半径：

\[
\rho=\frac{r\mu}{\sqrt{1+\mu^2}}
\]

仅用于可视化。同一接触副的力矩始终采用 `mu*N*r`，不得再用 `R*rho` 叠加第二次。

## 载荷来源

- 自动模式：采用同一时刻平面静力结果中的销轴平面合反力，标记 `planar_gravity_estimate`。
- 人工覆盖：常量或逐时刻载荷数据，标记实际来源和备注。
- 自动与人工值同时保留，便于比较；计算只采用用户选择的一个来源。
- `load_mode` 第一版只允许 `gravity_only`。数据结构预留 `gravity_plus_inertia`，未实现时必须返回 `unsupported_load_mode`。

## 页面

独立页面 `leg-friction.html`，不替换现有 J2/J3/J4 页面。页面包含：

1. 模型边界及可信度说明。
2. J2/J3/J4 和机构姿态选择；1 Hz 动画。
3. 25 个运动副及物理接触界面对照；同点多界面明确分行。
4. 每个接触面的 μs、μk、r、双侧载荷分配、人工载荷覆盖。
5. 当前时刻的理论载荷、相对速度、摩擦力矩、摩擦圆和输出等效阻力。
6. 单接触副及总等效阻力曲线、贡献排名。
7. CSV 导出。

页面固定显示：结果仅为“重力准静态、无摩擦反力的一阶理论估算”，总阻力仅覆盖当前启用且完成建模的接触副。

## 验收

- μ=0 时所有摩擦力矩、等效阻力和耗散功率为零。
- 单接触副启用时满足 `Q·qdot = T*omega`。
- 任意非零速度组合下 `T*omega <= 0`，总耗散功率非负显示值为 `-ΣP`。
- 反向速度使动摩擦和输出广义阻力反向，阻力幅值不变。
- 换向点返回静摩擦容量，动摩擦结果为 null，不伪造符号。
- 双侧载荷分配总和为 1，改变分配不重复载荷。
- P10/P18、固定副、保持副、移动副不进入第一版正式合计。
- 中心差分相对速度雅可比与输出对齐后的约束零空间结果在规定容差内一致。
- 参数、载荷模式、载荷来源和模型假设随 JSON/CSV 结果可追踪。

