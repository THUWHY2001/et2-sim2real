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
  ["neck", "头颈", "neck-j0.html"],
  ["torso", "躯干", "torso-j0.html"],
  ["arm", "手臂", "arm-j0.html"],
];

const nav = current => `<header class="site-header"><div class="site-header-inner"><a class="site-brand" href="index.html">${siteTitle}</a><nav class="site-nav" aria-label="模块导航">${modules.map(([id,label,href])=>`<a href="${href}"${id===current?' aria-current="page"':''}>${label}</a>`).join("")}</nav></div></header>`;

const jointNav = current => `<nav class="joint-nav" aria-label="腿足等效旋转轴"><span>腿足</span>${["j2","j3","j4"].map(id=>`<a class="joint-tab" href="${id}.html"${id===current?' aria-current="page"':''}>${id.toUpperCase()}</a>`).join("")}</nav>`;
const armJointNav = current => `<nav class="joint-nav" aria-label="手臂等效旋转轴"><span>手臂</span>${Array.from({length:7},(_,index)=>[`arm-j${index}`,`J${index}`]).map(([id,label])=>`<a class="joint-tab" href="${id}.html"${current===id?' aria-current="page"':''}>${label}</a>`).join('')}</nav>`;
const neckJointNav = current => `<nav class="joint-nav" aria-label="头颈等效旋转轴"><span>头颈</span>${[['neck-j0','J0'],['neck-j1','J1'],['neck-j2','J2'],['neck-combined','组合']].map(([id,label])=>`<a class="joint-tab" href="${id}.html"${current===id?' aria-current="page"':''}>${label}</a>`).join('')}</nav>`;
const torsoJointNav = current => `<nav class="joint-nav" aria-label="躯干等效旋转轴"><span>躯干</span>${[['torso-j0','J0'],['torso-j1','J1'],['torso-j2','J2'],['torso-combined','组合']].map(([id,label])=>`<a class="joint-tab" href="${id}.html"${current===id?' aria-current="page"':''}>${label}</a>`).join('')}</nav>`;

const chartResizeScript = `<script>(()=>{const initialized=new WeakSet();window.initChartResizeControls=(root=document)=>{root.querySelectorAll('[data-chart-panel]').forEach(panel=>{if(initialized.has(panel))return;initialized.add(panel);const stage=panel.querySelector('[data-chart-stage]'),output=panel.querySelector('[data-chart-scale]');let scale=1;const apply=()=>{stage.style.width=(scale*100)+'%';output.textContent=Math.round(scale*100)+'%';panel.querySelector('[data-chart-action="out"]').disabled=scale<=.75;panel.querySelector('[data-chart-action="in"]').disabled=scale>=2};panel.addEventListener('click',event=>{const button=event.target.closest('[data-chart-action]');if(!button)return;const action=button.dataset.chartAction;if(action==='out')scale=Math.max(.75,scale-.25);if(action==='in')scale=Math.min(2,scale+.25);if(action==='reset')scale=1;if(action==='fullscreen'){if(document.fullscreenElement===panel)document.exitFullscreen();else if(panel.requestFullscreen)panel.requestFullscreen();else panel.classList.toggle('chart-expanded')}apply()});document.addEventListener('fullscreenchange',()=>{const button=panel.querySelector('[data-chart-action="fullscreen"]');button.textContent=document.fullscreenElement===panel?'退出全屏':'全屏'});apply()})};window.initChartResizeControls(document)})();</script>`;

function document(title, current, body, joint = null, modelScript = null, modelVersion = assetVersion) {
  const scripts=joint?`<script src="directional-friction.js?v=${assetVersion}"></script><script src="link-tolerance.js?v=${assetVersion}"></script><script src="link-length-controls.js?v=${assetVersion}"></script>${current==='torso'?`<script src="torso-dynamics-data.js?v=${modelVersion}"></script>`:''}${modelScript?`<script src="${modelScript}?v=${modelVersion}"></script>`:''}${current==='neck'?`<script src="neck-3d-view.js?v=${assetVersion}"></script>`:''}${current==='torso'?`<script src="torso-3d-view.js?v=${modelVersion}"></script>`:''}`:'';
  const subnav=joint?(current==='arm'?armJointNav(joint):current==='neck'?neckJointNav(joint):current==='torso'?torsoJointNav(joint):jointNav(joint)):'';
  const styleVersion=current==='torso'||current==='neck'||current==='arm'?modelVersion:assetVersion;
  return `<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="referrer" content="no-referrer"><title>${title}</title><link rel="stylesheet" href="site.css?v=${styleVersion}">${scripts}</head><body>${nav(current)}${subnav}${body}${joint ? chartResizeScript : ""}</body></html>`;
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
for(const file of ["neck-blackbox.mjs","neck-runtime.mjs"]){
  copyFileSync(path.join(root,"src",file),path.join(dist,file));
  copyFileSync(path.join(root,"src",file),path.join(root,file));
}
mkdirSync(path.join(dist,"data","neck"),{recursive:true});
copyFileSync(path.join(root,"data","neck","model.json"),path.join(dist,"data","neck","model.json"));
for(const file of ["left-arm-blackbox.mjs","arm-runtime.mjs"]){
  copyFileSync(path.join(root,"src",file),path.join(dist,file));
  copyFileSync(path.join(root,"src",file),path.join(root,file));
}
mkdirSync(path.join(dist,"data","arm"),{recursive:true});
for(const file of ["model.json","pose_reference.json"]){
  copyFileSync(path.join(root,"data","arm",file),path.join(dist,"data","arm",file));
}
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
copyFileSync(path.join(root, "src", "torso-dynamics-data.js"), path.join(dist, "torso-dynamics-data.js"));
copyFileSync(path.join(root, "src", "torso-dynamics-data.js"), path.join(root, "torso-dynamics-data.js"));
copyFileSync(path.join(root, "src", "torso-3d-view.js"), path.join(dist, "torso-3d-view.js"));
copyFileSync(path.join(root, "src", "torso-3d-view.js"), path.join(root, "torso-3d-view.js"));
copyFileSync(path.join(root, "site.css"), path.join(dist, "site.css"));
mkdirSync(path.join(dist,"data"),{recursive:true});
for(const file of ["ET2_waist_dynamics_addon.json","ET2_waist_dynamics_body_table.csv"])copyFileSync(path.join(root,"data",file),path.join(dist,"data",file));
mkdirSync(path.join(dist,"src"),{recursive:true});
copyFileSync(path.join(root,"src","leg-full-runtime.mjs"),path.join(dist,"src","leg-full-runtime.mjs"));
mkdirSync(path.join(dist,"data","leg-full-package"),{recursive:true});
for(const file of ["leg_full.mjs","model.json"]){
  copyFileSync(path.join(root,"data","leg-full-package",file),path.join(dist,"data","leg-full-package",file));
}

const models = [
  {id:"j2", title:"腿足 J2 Hip Pitch 阻力分析", fragment:"leg-full-joint.fragment.html", section:"leg", modelVersion:"20261106-leg-full-1", transform:fragment=>fragment.replaceAll('__JOINT__','J2').replaceAll('__MOTION__','Hip Pitch').replaceAll('__DESCRIPTION__','髋 Pitch 推杆主动伸缩，J2 为闭链被动旋转输出')},
  {id:"j2-clearance", title:"J2 间隙与死区分析", fragment:"j2-clearance.fragment.html", section:"leg", navJoint:"j2", modelScript:"j2-clearance-model.js", modelVersion:"20261103-3"},
  {id:"j3", title:"腿足 J3 Knee Pitch 阻力分析", fragment:"leg-full-joint.fragment.html", section:"leg", modelVersion:"20261106-leg-full-1", transform:fragment=>fragment.replaceAll('__JOINT__','J3').replaceAll('__MOTION__','Knee Pitch').replaceAll('__DESCRIPTION__','膝 Pitch 推杆主动伸缩，J3 为闭链被动旋转输出')},
  {id:"j4", title:"腿足 J4 Ankle Pitch 阻力分析", fragment:"leg-full-joint.fragment.html", section:"leg", modelVersion:"20261106-leg-full-1", transform:fragment=>fragment.replaceAll('__JOINT__','J4').replaceAll('__MOTION__','Ankle Pitch').replaceAll('__DESCRIPTION__','踝 Pitch 推杆主动伸缩，J4 为闭链被动旋转输出；保留 J5 踝 Roll 几何耦合')},
  {id:"neck-j0", title:"头颈 J0 Roll 阻力分析", fragment:"neck-joint.fragment.html", section:"neck", modelVersion:"20261105-blackbox-1", transform:fragment=>fragment.replaceAll('__JOINT__','J0').replaceAll('__JOINT_LOWER__','j0').replaceAll('__MOTION__','Roll').replaceAll('__DESCRIPTION__','左右推杆差动伸缩，Roll 为闭链被动输出').replaceAll('__TOLERANCE_LINK__','<a class="btn" href="neck-j0-tolerance.html">硬点公差分析</a>')},
  {id:"neck-j1", title:"头颈 J1 Pitch 阻力分析", fragment:"neck-joint.fragment.html", section:"neck", modelVersion:"20261105-blackbox-1", transform:fragment=>fragment.replaceAll('__JOINT__','J1').replaceAll('__JOINT_LOWER__','j1').replaceAll('__MOTION__','Pitch').replaceAll('__DESCRIPTION__','左右推杆同步同向伸缩，Pitch 为闭链被动输出').replaceAll('__TOLERANCE_LINK__','<a class="btn" href="neck-j1-tolerance.html">硬点公差分析</a>')},
  {id:"neck-j2", title:"头颈 J2 Yaw 阻力分析", fragment:"neck-joint.fragment.html", section:"neck", modelVersion:"20261105-blackbox-1", transform:fragment=>fragment.replaceAll('__JOINT__','J2').replaceAll('__JOINT_LOWER__','j2').replaceAll('__MOTION__','Yaw').replaceAll('__DESCRIPTION__','Yaw 主动旋转，Roll/Pitch 相对姿态及推杆行程保持不变').replaceAll('__TOLERANCE_LINK__','')},
  {id:"neck-combined", title:"头颈三个主动输入组合运动", fragment:"neck-combined.fragment.html", section:"neck", navJoint:"neck-combined", modelVersion:"20261105-blackbox-1"},
  {id:"neck-j0-tolerance", title:"头颈 J0 硬点公差分析", fragment:"neck-tolerance.fragment.html", section:"neck", navJoint:"neck-j0", modelScript:"neck-model.js", transform:fragment=>fragment.replaceAll('__JOINT__','J0').replaceAll('__JOINT_LOWER__','j0')},
  {id:"neck-j1-tolerance", title:"头颈 J1 硬点公差分析", fragment:"neck-tolerance.fragment.html", section:"neck", navJoint:"neck-j1", modelScript:"neck-model.js", transform:fragment=>fragment.replaceAll('__JOINT__','J1').replaceAll('__JOINT_LOWER__','j1')},
  {id:"torso-j0", title:"躯干 J0 Yaw 阻力分析", fragment:"torso-joint.fragment.html", section:"torso", modelScript:"torso-model.js", modelVersion:"20261105-load-1", transform:fragment=>fragment.replaceAll('__JOINT__','J0').replaceAll('__MOTION__','Yaw').replaceAll('__DESCRIPTION__','Yaw电机轴直接驱动，内部闭链无相对运动')},
  {id:"torso-j1", title:"躯干 J1 Roll 阻力分析", fragment:"torso-joint.fragment.html", section:"torso", modelScript:"torso-model.js", modelVersion:"20261105-load-1", transform:fragment=>fragment.replaceAll('__JOINT__','J1').replaceAll('__MOTION__','Roll').replaceAll('__DESCRIPTION__','左右曲柄反向驱动，伴随Pitch由闭链自然求解')},
  {id:"torso-j2", title:"躯干 J2 Pitch 阻力分析", fragment:"torso-joint.fragment.html", section:"torso", modelScript:"torso-model.js", modelVersion:"20261105-load-1", transform:fragment=>fragment.replaceAll('__JOINT__','J2').replaceAll('__MOTION__','Pitch').replaceAll('__DESCRIPTION__','左右曲柄同向驱动，伴随Roll由闭链自然求解')},
  {id:"torso-combined", title:"躯干三主动轴组合运动", fragment:"torso-combined.fragment.html", section:"torso", modelScript:"torso-model.js", modelVersion:"20261105-load-1"},
  ...[
    ['J0','Shoulder Pitch','肩部 J0 旋转轴直接驱动','真实旋转轴主动'],
    ['J1','Shoulder Roll','肩部 J1 旋转轴直接驱动','真实旋转轴主动'],
    ['J2','Shoulder Yaw','肩部 J2 旋转轴直接驱动','真实旋转轴主动'],
    ['J3','Elbow Pitch','肘部单推杆主动，J3 输出轴被动','肘部单推杆主动｜输出轴被动'],
    ['J4','Elbow Yaw','肘部 J4 旋转轴直接驱动','真实旋转轴主动'],
    ['J5','Wrist Roll','腕部双推杆主动，J5 输出轴被动','腕部双推杆主动｜输出轴被动'],
    ['J6','Wrist Pitch','腕部双推杆主动，J6 输出轴被动','腕部双推杆主动｜输出轴被动'],
  ].map(([joint,motion,description,badge])=>({id:`arm-${joint.toLowerCase()}`,title:`手臂 ${joint} ${motion} 阻力分析`,fragment:'arm-joint.fragment.html',section:'arm',modelVersion:'20261105-arm-axis-1',transform:fragment=>fragment.replaceAll('__JOINT__',joint).replaceAll('__MOTION__',motion).replaceAll('__DRIVER_DESCRIPTION__',description).replaceAll('__DRIVER_BADGE__',badge)})),
];

for (const model of models) {
  let fragment = readFileSync(path.join(root, "src", model.fragment), "utf8");
  if(model.transform)fragment=model.transform(fragment);
  const page = document(`${model.title}｜${siteTitle}`, model.section, `<main class="model-shell">${fragment}</main>`, model.navJoint||model.id, model.modelScript, model.modelVersion);
  writeFileSync(path.join(dist, `${model.id}.html`), page);
  writeFileSync(path.join(root, `${model.id}.html`), page);
  if(["j2","j3","j4"].includes(model.id)){
    const publicRootPage=page.replace("<head>",'<head><base href="web2/">');
    writeFileSync(path.join(root,"..",`${model.id}.html`),publicRootPage);
  }
}

const home = `<main class="model-shell"><section class="home-head"><h1>${siteTitle}</h1><p>按机器人模块组织机构模型，计算连接副阻力向控制模型等效旋转轴的折算结果，并比较各连接副的影响程度。</p></section><section class="system-modules" aria-label="机器人模块"><article class="system-module system-module-active" id="leg"><div class="module-heading"><div><span class="module-index">01</span><h2>腿足</h2></div><span class="module-status">已建立 J2–J4 模型</span></div><div class="joint-options"><a href="j2.html"><strong>J2</strong><span>髋部等效旋转轴</span><small>推杆、多连杆闭链与旋转副阻力折算</small></a><a href="j3.html"><strong>J3</strong><span>膝部等效旋转轴</span><small>J3–Q 输出、A 点几何闭合与推杆传动</small></a><a href="j4.html"><strong>J4</strong><span>踝部等效旋转轴</span><small>主动曲柄、鱼眼连接副与伸缩推杆组合</small></a></div></article><article class="system-module" id="head-neck"><div class="module-heading"><div><span class="module-index">02</span><h2>头颈</h2></div><span class="module-status module-status-pending">待建立机构模型</span></div></article><article class="system-module" id="torso"><div class="module-heading"><div><span class="module-index">03</span><h2>躯干</h2></div><span class="module-status module-status-pending">待建立机构模型</span></div></article><article class="system-module system-module-active" id="arm"><div class="module-heading"><div><span class="module-index">04</span><h2>手臂</h2></div><span class="module-status">已建立 J3、J5 模型</span></div><div class="joint-options"><a href="arm-j3.html"><strong>J3</strong><span>手臂 J3 等效旋转轴</span><small>平行轴四杆闭环、三维推杆摆转与双向阻力折算</small></a><a href="arm-j5.html"><strong>J5</strong><span>手臂 J5 等效旋转轴</span><small>并联双推杆、四鱼眼连接副与三维刚体旋转</small></a></div></article></section><aside class="scope-note"><strong>计算范围：</strong>1 Hz、±10° 单关节激励；输出传动系数、静摩擦启动力矩、运行阻力和连接副贡献度。</aside></main>`;
const armHome='<article class="system-module system-module-active" id="arm"><div class="module-heading"><div><span class="module-index">04</span><h2>手臂</h2></div><span class="module-status">已建立 J0–J6 黑盒模型</span></div><div class="joint-options joint-options-four">'+Array.from({length:7},(_,index)=>`<a href="arm-j${index}.html"><strong>J${index}</strong><span>左臂 J${index}</span><small>${index===3?'肘部单推杆闭链':index>=5?'腕部双推杆闭链':'真实主动旋转轴'}</small></a>`).join('')+'</div></article>';
const homeWithArmJ6=home.replace(/<article class="system-module system-module-active" id="arm">[\s\S]*?<\/article><\/section>/,armHome+'</section>');
const homeWithNeck=homeWithArmJ6.replace('<article class="system-module" id="head-neck"><div class="module-heading"><div><span class="module-index">02</span><h2>头颈</h2></div><span class="module-status module-status-pending">待建立机构模型</span></div></article>','<article class="system-module system-module-active" id="head-neck"><div class="module-heading"><div><span class="module-index">02</span><h2>头颈</h2></div><span class="module-status">已建立 J0、J1、J2 黑盒闭链模型</span></div><div class="joint-options joint-options-four"><a href="neck-j0.html"><strong>J0</strong><span>Roll 被动输出</span><small>双推杆差动伸缩与闭链阻力折算</small></a><a href="neck-j1.html"><strong>J1</strong><span>Pitch 被动输出</span><small>双推杆同步伸缩与闭链阻力折算</small></a><a href="neck-j2.html"><strong>J2</strong><span>Yaw 主动旋转</span><small>推杆行程与相对 Roll/Pitch 保持不变</small></a><a href="neck-combined.html"><strong>组合</strong><span>三个主动输入</span><small>Yaw、左推杆、右推杆联合运动</small></a></div></article>');
const homeWithTorso=homeWithNeck.replace('<article class="system-module" id="torso"><div class="module-heading"><div><span class="module-index">03</span><h2>躯干</h2></div><span class="module-status module-status-pending">待建立机构模型</span></div></article>','<article class="system-module system-module-active" id="torso"><div class="module-heading"><div><span class="module-index">03</span><h2>躯干</h2></div><span class="module-status">已建立 J0、J1、J2 闭链模型</span></div><div class="joint-options joint-options-four"><a href="torso-j0.html"><strong>J0</strong><span>Yaw等效旋转轴</span><small>真实Yaw主动轴与内部零相对运动校核</small></a><a href="torso-j1.html"><strong>J1</strong><span>Roll等效旋转轴</span><small>双曲柄反向驱动与有限角度耦合</small></a><a href="torso-j2.html"><strong>J2</strong><span>Pitch等效旋转轴</span><small>双曲柄同向驱动与闭链阻力折算</small></a><a href="torso-combined.html"><strong>组合</strong><span>三个真实主动输入</span><small>Yaw、左曲柄、右曲柄联合运动</small></a></div></article>');
const homePage = document(siteTitle, "index", homeWithTorso);
writeFileSync(path.join(dist, "index.html"), homePage);
writeFileSync(path.join(root, "index.html"), homePage);
