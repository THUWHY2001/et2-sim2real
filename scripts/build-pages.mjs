import { copyFileSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import path from "node:path";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const dist = path.join(root, "dist");
mkdirSync(dist, {recursive: true});

const siteTitle = "模块连接副-等效旋转轴的灵敏度分析";
const assetVersion = "20261030-1";
const modules = [
  ["index", "总览", "index.html"],
  ["leg", "腿足", "j2.html"],
  ["neck", "头颈", "web2/neck-j0.html"],
  ["torso", "躯干", "torso-j0.html"],
  ["arm", "手臂", "web2/arm-j0.html"],
];

const nav = current => `<header class="site-header"><div class="site-header-inner"><a class="site-brand" href="index.html">${siteTitle}</a><nav class="site-nav" aria-label="模块导航">${modules.map(([id,label,href])=>`<a href="${href}"${id===current?' aria-current="page"':''}>${label}</a>`).join("")}</nav></div></header>`;

const jointNav = current => `<nav class="joint-nav" aria-label="腿足等效旋转轴"><span>腿足</span>${["j2","j3","j4"].map(id=>`<a class="joint-tab" href="${id}.html"${id===current?' aria-current="page"':''}>${id.toUpperCase()}</a>`).join("")}</nav>`;
const armJointNav = current => `<nav class="joint-nav" aria-label="手臂等效旋转轴"><span>手臂</span>${[['arm-j3','J3'],['arm-j5','J5'],['arm-j6','J6']].map(([id,label])=>`<a class="joint-tab" href="${id}.html"${current===id?' aria-current="page"':''}>${label}</a>`).join('')}</nav>`;
const neckJointNav = current => `<nav class="joint-nav" aria-label="头颈等效旋转轴"><span>头颈</span>${[['neck-j0','J0'],['neck-j1','J1']].map(([id,label])=>`<a class="joint-tab" href="${id}.html"${current===id?' aria-current="page"':''}>${label}</a>`).join('')}</nav>`;
const torsoJointNav = current => `<nav class="joint-nav" aria-label="躯干等效旋转轴"><span>躯干</span>${[['torso-j0','J0'],['torso-j1','J1'],['torso-j2','J2'],['torso-combined','组合']].map(([id,label])=>`<a class="joint-tab" href="${id}.html"${current===id?' aria-current="page"':''}>${label}</a>`).join('')}</nav>`;

const chartResizeScript = `<script>(()=>{const initialized=new WeakSet();window.initChartResizeControls=(root=document)=>{root.querySelectorAll('[data-chart-panel]').forEach(panel=>{if(initialized.has(panel))return;initialized.add(panel);const stage=panel.querySelector('[data-chart-stage]'),output=panel.querySelector('[data-chart-scale]');let scale=1;const apply=()=>{stage.style.width=(scale*100)+'%';output.textContent=Math.round(scale*100)+'%';panel.querySelector('[data-chart-action="out"]').disabled=scale<=.75;panel.querySelector('[data-chart-action="in"]').disabled=scale>=2};panel.addEventListener('click',event=>{const button=event.target.closest('[data-chart-action]');if(!button)return;const action=button.dataset.chartAction;if(action==='out')scale=Math.max(.75,scale-.25);if(action==='in')scale=Math.min(2,scale+.25);if(action==='reset')scale=1;if(action==='fullscreen'){if(document.fullscreenElement===panel)document.exitFullscreen();else if(panel.requestFullscreen)panel.requestFullscreen();else panel.classList.toggle('chart-expanded')}apply()});document.addEventListener('fullscreenchange',()=>{const button=panel.querySelector('[data-chart-action="fullscreen"]');button.textContent=document.fullscreenElement===panel?'退出全屏':'全屏'});apply()})};window.initChartResizeControls(document)})();</script>`;

function document(title, current, body, joint = null, modelScript = null, modelVersion = assetVersion) {
  const scripts=joint?`<script src="directional-friction.js?v=${assetVersion}"></script><script src="link-tolerance.js?v=${assetVersion}"></script><script src="link-length-controls.js?v=${assetVersion}"></script>${modelScript?`<script src="${modelScript}?v=${modelVersion}"></script>`:''}${current==='neck'?`<script src="neck-3d-view.js?v=${assetVersion}"></script>`:''}${current==='torso'?`<script src="torso-3d-view.js?v=${modelVersion}"></script>`:''}`:'';
  const subnav=joint?(current==='arm'?armJointNav(joint):current==='neck'?neckJointNav(joint):current==='torso'?torsoJointNav(joint):jointNav(joint)):'';
  return `<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="referrer" content="no-referrer"><title>${title}</title><link rel="stylesheet" href="site.css?v=${assetVersion}">${scripts}</head><body>${nav(current)}${subnav}${body}${joint ? chartResizeScript : ""}</body></html>`;
}

copyFileSync(path.join(root, "src", "directional-friction.js"), path.join(dist, "directional-friction.js"));
copyFileSync(path.join(root, "src", "directional-friction.js"), path.join(root, "directional-friction.js"));
copyFileSync(path.join(root, "src", "link-tolerance.js"), path.join(dist, "link-tolerance.js"));
copyFileSync(path.join(root, "src", "link-tolerance.js"), path.join(root, "link-tolerance.js"));
copyFileSync(path.join(root, "src", "link-length-controls.js"), path.join(dist, "link-length-controls.js"));
copyFileSync(path.join(root, "src", "link-length-controls.js"), path.join(root, "link-length-controls.js"));
copyFileSync(path.join(root, "src", "j2-clearance-model.js"), path.join(dist, "j2-clearance-model.js"));
copyFileSync(path.join(root, "src", "j2-clearance-model.js"), path.join(root, "j2-clearance-model.js"));
copyFileSync(path.join(root, "src", "neck-model.js"), path.join(dist, "neck-model.js"));
copyFileSync(path.join(root, "src", "neck-model.js"), path.join(root, "neck-model.js"));
copyFileSync(path.join(root, "src", "neck-3d-view.js"), path.join(dist, "neck-3d-view.js"));
copyFileSync(path.join(root, "src", "neck-3d-view.js"), path.join(root, "neck-3d-view.js"));
copyFileSync(path.join(root, "src", "neck-tolerance-model.js"), path.join(dist, "neck-tolerance-model.js"));
copyFileSync(path.join(root, "src", "neck-tolerance-model.js"), path.join(root, "neck-tolerance-model.js"));
copyFileSync(path.join(root, "src", "arm-j3-model.js"), path.join(dist, "arm-j3-model.js"));
copyFileSync(path.join(root, "src", "arm-j3-model.js"), path.join(root, "arm-j3-model.js"));
copyFileSync(path.join(root, "src", "arm-j5-model.js"), path.join(dist, "arm-j5-model.js"));
copyFileSync(path.join(root, "src", "arm-j5-model.js"), path.join(root, "arm-j5-model.js"));
copyFileSync(path.join(root, "src", "arm-j6-model.js"), path.join(dist, "arm-j6-model.js"));
copyFileSync(path.join(root, "src", "arm-j6-model.js"), path.join(root, "arm-j6-model.js"));
copyFileSync(path.join(root, "src", "torso-model.js"), path.join(dist, "torso-model.js"));
copyFileSync(path.join(root, "src", "torso-model.js"), path.join(root, "torso-model.js"));
copyFileSync(path.join(root, "src", "torso-3d-view.js"), path.join(dist, "torso-3d-view.js"));
copyFileSync(path.join(root, "src", "torso-3d-view.js"), path.join(root, "torso-3d-view.js"));
copyFileSync(path.join(root, "site.css"), path.join(dist, "site.css"));

const models = [
  {id:"j2", title:"J2 二维机构简图", fragment:"j2.fragment.html", section:"leg"},
  {id:"j2-clearance", title:"J2 间隙与死区分析", fragment:"j2-clearance.fragment.html", section:"leg", navJoint:"j2", modelScript:"j2-clearance-model.js", modelVersion:"20261103-3"},
  {id:"j3", title:"J3 二维机构简图", fragment:"j3.fragment.html", section:"leg"},
  {id:"j4", title:"J4 二维机构简图", fragment:"j4.fragment.html", section:"leg"},
  {id:"neck-j0", title:"头颈 J0 Roll", fragment:"neck-joint.fragment.html", section:"neck", modelScript:"neck-model.js", transform:fragment=>fragment.replaceAll('__JOINT__','J0').replaceAll('__JOINT_LOWER__','j0').replaceAll('__MOTION__','Roll').replaceAll('__VIEW__','Y–Z').replaceAll('__AXIS_COORD__','R1 (-52,0,479.66) → R2 (-40,0,479.66)')},
  {id:"neck-j1", title:"头颈 J1 Pitch", fragment:"neck-joint.fragment.html", section:"neck", modelScript:"neck-model.js", transform:fragment=>fragment.replaceAll('__JOINT__','J1').replaceAll('__JOINT_LOWER__','j1').replaceAll('__MOTION__','Pitch').replaceAll('__VIEW__','X–Z').replaceAll('__AXIS_COORD__','P1 (-16,20,479.66) → P2 (-16,-20,479.66)')},
  {id:"neck-j0-tolerance", title:"头颈 J0 硬点公差分析", fragment:"neck-tolerance.fragment.html", section:"neck", navJoint:"neck-j0", modelScript:"neck-model.js", transform:fragment=>fragment.replaceAll('__JOINT__','J0').replaceAll('__JOINT_LOWER__','j0')},
  {id:"neck-j1-tolerance", title:"头颈 J1 硬点公差分析", fragment:"neck-tolerance.fragment.html", section:"neck", navJoint:"neck-j1", modelScript:"neck-model.js", transform:fragment=>fragment.replaceAll('__JOINT__','J1').replaceAll('__JOINT_LOWER__','j1')},
  {id:"torso-j0", title:"躯干 J0 Yaw 阻力分析", fragment:"torso-joint.fragment.html", section:"torso", modelScript:"torso-model.js", modelVersion:"20261104-1", transform:fragment=>fragment.replaceAll('__JOINT__','J0').replaceAll('__MOTION__','Yaw').replaceAll('__DESCRIPTION__','Yaw电机轴直接驱动，内部闭链无相对运动')},
  {id:"torso-j1", title:"躯干 J1 Roll 阻力分析", fragment:"torso-joint.fragment.html", section:"torso", modelScript:"torso-model.js", modelVersion:"20261104-1", transform:fragment=>fragment.replaceAll('__JOINT__','J1').replaceAll('__MOTION__','Roll').replaceAll('__DESCRIPTION__','左右曲柄反向驱动，伴随Pitch由闭链自然求解')},
  {id:"torso-j2", title:"躯干 J2 Pitch 阻力分析", fragment:"torso-joint.fragment.html", section:"torso", modelScript:"torso-model.js", modelVersion:"20261104-1", transform:fragment=>fragment.replaceAll('__JOINT__','J2').replaceAll('__MOTION__','Pitch').replaceAll('__DESCRIPTION__','左右曲柄同向驱动，伴随Roll由闭链自然求解')},
  {id:"torso-combined", title:"躯干三主动轴组合运动", fragment:"torso-combined.fragment.html", section:"torso", modelScript:"torso-model.js", modelVersion:"20261104-1"},
  {id:"arm-j3", title:"手臂 J3 机构简图", fragment:"arm-j3.fragment.html", section:"arm", modelScript:"arm-j3-model.js"},
  {id:"arm-j5", title:"手臂 J5 机构简图", fragment:"arm-j5.fragment.html", section:"arm", modelScript:"arm-j5-model.js"},
  {id:"arm-j6", title:"手臂 J6 机构简图", fragment:"arm-j5.fragment.html", section:"arm", modelScript:"arm-j6-model.js", transform:fragment=>fragment.replaceAll('arm-j5','arm-j6').replaceAll('ArmJ5Model','ArmJ6Model').replaceAll('J5','J6').replace('(25.986,335.600,−92.703)—(17.524,336.576,−95.609)','(25.282,328.524,−106.969)—(25.869,355.606,−96.874)').replaceAll('两根推杆同步伸缩','两根推杆反向伸缩').replace('手臂 J6 并联双推杆机构与阻力贡献','手臂 J6 手腕Pitch双推杆机构与阻力贡献')},
];

for (const model of models) {
  let fragment = readFileSync(path.join(root, "src", model.fragment), "utf8");
  if(model.transform)fragment=model.transform(fragment);
  const page = document(`${model.title}｜${siteTitle}`, model.section, `<main class="model-shell">${fragment}</main>`, model.navJoint||model.id, model.modelScript, model.modelVersion);
  writeFileSync(path.join(dist, `${model.id}.html`), page);
  writeFileSync(path.join(root, `${model.id}.html`), page);
}

const home = `<main class="model-shell"><section class="home-head"><h1>${siteTitle}</h1><p>按机器人模块组织机构模型，计算连接副阻力向控制模型等效旋转轴的折算结果，并比较各连接副的影响程度。</p></section><section class="system-modules" aria-label="机器人模块"><article class="system-module system-module-active" id="leg"><div class="module-heading"><div><span class="module-index">01</span><h2>腿足</h2></div><span class="module-status">已建立 J2–J4 模型</span></div><div class="joint-options"><a href="j2.html"><strong>J2</strong><span>髋部等效旋转轴</span><small>推杆、多连杆闭链与旋转副阻力折算</small></a><a href="j3.html"><strong>J3</strong><span>膝部等效旋转轴</span><small>J3–Q 输出、A 点几何闭合与推杆传动</small></a><a href="j4.html"><strong>J4</strong><span>踝部等效旋转轴</span><small>主动曲柄、鱼眼连接副与伸缩推杆组合</small></a></div></article><article class="system-module" id="head-neck"><div class="module-heading"><div><span class="module-index">02</span><h2>头颈</h2></div><span class="module-status module-status-pending">待建立机构模型</span></div></article><article class="system-module" id="torso"><div class="module-heading"><div><span class="module-index">03</span><h2>躯干</h2></div><span class="module-status module-status-pending">待建立机构模型</span></div></article><article class="system-module system-module-active" id="arm"><div class="module-heading"><div><span class="module-index">04</span><h2>手臂</h2></div><span class="module-status">已建立 J3、J5 模型</span></div><div class="joint-options"><a href="arm-j3.html"><strong>J3</strong><span>手臂 J3 等效旋转轴</span><small>平行轴四杆闭环、三维推杆摆转与双向阻力折算</small></a><a href="arm-j5.html"><strong>J5</strong><span>手臂 J5 等效旋转轴</span><small>并联双推杆、四鱼眼连接副与三维刚体旋转</small></a></div></article></section><aside class="scope-note"><strong>计算范围：</strong>1 Hz、±10° 单关节激励；输出传动系数、静摩擦启动力矩、运行阻力和连接副贡献度。</aside></main>`;
const homeWithArmJ6=home.replace('主动曲柄、鱼眼连接副与伸缩推杆组合','主动曲柄、滑套轴承与伸缩推杆组合').replace('已建立 J3、J5 模型','已建立 J3、J5、J6 模型').replace('</div></article></section><aside class="scope-note">','<a href="arm-j6.html"><strong>J6</strong><span>手臂 J6 等效旋转轴</span><small>手腕 Pitch、双推杆反向伸缩与四鱼眼摆转</small></a></div></article></section><aside class="scope-note">');
const homeWithNeck=homeWithArmJ6.replace('<article class="system-module" id="head-neck"><div class="module-heading"><div><span class="module-index">02</span><h2>头颈</h2></div><span class="module-status module-status-pending">待建立机构模型</span></div></article>','<article class="system-module system-module-active" id="head-neck"><div class="module-heading"><div><span class="module-index">02</span><h2>头颈</h2></div><span class="module-status">已建立 J0、J1 模型</span></div><div class="joint-options"><a href="neck-j0.html"><strong>J0</strong><span>头颈 Roll 等效旋转轴</span><small>双推杆反向伸缩、重力矩与连接副阻力折算</small></a><a href="neck-j1.html"><strong>J1</strong><span>头颈 Pitch 等效旋转轴</span><small>双推杆同步伸缩、重力反力与摩擦贡献</small></a></div></article>');
const homeWithTorso=homeWithNeck.replace('<article class="system-module" id="torso"><div class="module-heading"><div><span class="module-index">03</span><h2>躯干</h2></div><span class="module-status module-status-pending">待建立机构模型</span></div></article>','<article class="system-module system-module-active" id="torso"><div class="module-heading"><div><span class="module-index">03</span><h2>躯干</h2></div><span class="module-status">已建立 J0、J1、J2 闭链模型</span></div><div class="joint-options joint-options-four"><a href="torso-j0.html"><strong>J0</strong><span>Yaw等效旋转轴</span><small>真实Yaw主动轴与内部零相对运动校核</small></a><a href="torso-j1.html"><strong>J1</strong><span>Roll等效旋转轴</span><small>双曲柄反向驱动与有限角度耦合</small></a><a href="torso-j2.html"><strong>J2</strong><span>Pitch等效旋转轴</span><small>双曲柄同向驱动与闭链阻力折算</small></a><a href="torso-combined.html"><strong>组合</strong><span>三个真实主动输入</span><small>Yaw、左曲柄、右曲柄联合运动</small></a></div></article>');
const homePage = document(siteTitle, "index", homeWithTorso);
writeFileSync(path.join(dist, "index.html"), homePage);
writeFileSync(path.join(root, "index.html"), homePage);
