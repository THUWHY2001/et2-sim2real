// Offline ES module for a JS/Vite/Three.js project. Caller imports model.json.
// Only rigid-body data/closed-loop position kinematics, not dynamics integration.
export function makeNeck(data) {
  const sub = (a, b) => a.map((x, i) => x - b[i]);
  const add = (a, b) => a.map((x, i) => x + b[i]);
  const norm = (v) => Math.hypot(...v);
  const rx = (a, v) => [v[0], Math.cos(a) * v[1] - Math.sin(a) * v[2], Math.sin(a) * v[1] + Math.cos(a) * v[2]];
  const ry = (a, v) => [Math.cos(a) * v[0] + Math.sin(a) * v[2], v[1], -Math.sin(a) * v[0] + Math.cos(a) * v[2]];
  const k = data.rod_kinematics;
  function lengthsRad(roll, pitch) {
    const out = {};
    for (const side of ['L', 'R']) {
      const tip = add(k.roll_center_world0_mm,
        rx(roll, add(sub(k.pitch_center_world0_mm, k.roll_center_world0_mm),
          ry(pitch, sub(k.upper_ball_world0_mm[side], k.pitch_center_world0_mm)))));
      out[side] = norm(sub(tip, k.lower_ball_world0_mm[side]));
    }
    return out;
  }
  function poseToRods(rollDeg = 0, pitchDeg = 0, yawDeg = 0) {
    const len = lengthsRad(rollDeg * Math.PI / 180, pitchDeg * Math.PI / 180);
    return { yaw_deg: yawDeg, roll_deg: rollDeg, pitch_deg: pitchDeg, lengths_mm: len,
      extension_mm: {L: len.L - k.rod_length0_mm.L, R: len.R - k.rod_length0_mm.R} };
  }
  function rodsToPose(leftMM, rightMM) {
    const target = [k.rod_length0_mm.L + leftMM, k.rod_length0_mm.R + rightMM];
    if (target.some(x => x <= 0)) throw Error('Rod length must be positive');
    const [[a,b],[c,d]] = k.initial_jacobian_mm_per_rad;
    let det = a * d - b * c;
    if (Math.abs(det) < 1e-10) throw Error('Singular initial Jacobian');
    let x = [(d*leftMM - b*rightMM)/det, (-c*leftMM + a*rightMM)/det];
    function e(v) { const z=lengthsRad(...v);return [z.L-target[0],z.R-target[1]]; }
    for (let it=0; it<45; it++) {
      const r=e(x), size=norm(r);
      if (size<1e-8) return {roll_deg:x[0]*180/Math.PI,pitch_deg:x[1]*180/Math.PI,
        residual_max_mm:Math.max(...r.map(Math.abs)),iterations:it, branch:'near_initial'};
      const h=1e-6, dx=e([x[0]+h,x[1]]),dy=e([x[0],x[1]+h]);
      const a=(dx[0]-r[0])/h,b=(dy[0]-r[0])/h,c=(dx[1]-r[1])/h,d=(dy[1]-r[1])/h;
      det=a*d-b*c;if(Math.abs(det)<1e-10) break;
      const step=[(-d*r[0]+b*r[1])/det,(c*r[0]-a*r[1])/det];
      let accepted=false;
      for(let t=0;t<20;t++) {
        const f=2**(-t), n=[x[0]+f*step[0],x[1]+f*step[1]];
        if(norm(e(n))<size) {x=n;accepted=true;break;}
      }
      if(!accepted) break;
    }
    throw Error('No converged solution near initial pose; check branch/limits');
  }
  function summary() { return {model:data.meta.model,bodies:Object.keys(data.bodies).length,
      joints:Object.keys(data.joints).length,mass_kg:data.mass_total_kg,gravity:data.gravity_world_m_s2,
      active_motions:Object.entries(data.motions).filter(([n,m])=>m.enabled).map(([n])=>n)}; }
  function validate() {const m=Object.values(data.bodies).reduce((s,b)=>s+b.mass_kg,0),
    ik=poseToRods(5,3),fk=rodsToPose(ik.extension_mm.L,ik.extension_mm.R);
    return {pass:Math.abs(m-data.mass_total_kg)<1e-9 && Math.abs(fk.roll_deg-5)<1e-7 && Math.abs(fk.pitch_deg-3)<1e-7,
      mass_kg:m,roundtrip_residual_mm:fk.residual_max_mm}; }
  return {summary,validate,body:(name)=>data.bodies[name],joint:(name)=>data.joints[name],
    point:(name)=>data.hardpoints[name],poseToRods,rodsToPose,
    bodies:data.bodies,joints:data.joints,motions:data.motions,hardpoints:data.hardpoints,
    externalAnchors:data.external_anchors,gravity:data.gravity_world_m_s2};
}
