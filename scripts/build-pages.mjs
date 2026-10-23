import { copyFileSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import path from "node:path";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const dist = path.join(root, "dist");
mkdirSync(dist, {recursive: true});

const siteTitle = "模块连接副-等效旋转轴的灵敏度分析";
const assetVersion = "20261023-3";
const modules = [
  ["index", "总览", "index.html"],
  ["leg", "腿足", "j2.html"],
  ["neck", "头颈", "index.html#head-neck"],
  ["torso", "躯干", "index.html#torso"],
  ["arm", "手臂", "arm-j3.html"],
];

const nav = current => `<header class="site-header"><div class="site-header-inner"><a class="site-brand" href="index.html">${siteTitle}</a><nav class="site-nav" aria-label="模块导航">${modules.map(([id,label,href])=>`<a href="${href}"${id===current?' aria-current="page"':''}>${label}</a>`).join("")}</nav></div></header>`;

const jointNav = current => `<nav class="joint-nav" aria-label="腿足等效旋转轴"><span>腿足</span>${["j2","j3","j4"].map(id=>`<a class="joint-tab" href="${id}.html"${id===current?' aria-current="page"':''}>${id.toUpperCase()}</a>`).join("")}</nav>`;
const armJointNav = current => `<nav class="joint-nav" aria-label="手臂等效旋转轴"><span>手臂</span><a class="joint-tab" href="arm-j3.html"${current==='arm-j3'?' aria-current="page"':''}>J3</a></nav>`;

const chartResizeScript = `<script>(()=>{document.querySelectorAll('[data-chart-panel]').forEach(panel=>{const stage=panel.querySelector('[data-chart-stage]'),output=panel.querySelector('[data-chart-scale]');let scale=1;const apply=()=>{stage.style.width=(scale*100)+'%';output.textContent=Math.round(scale*100)+'%';panel.querySelector('[data-chart-action="out"]').disabled=scale<=.75;panel.querySelector('[data-chart-action="in"]').disabled=scale>=2};panel.addEventListener('click',event=>{const button=event.target.closest('[data-chart-action]');if(!button)return;const action=button.dataset.chartAction;if(action==='out')scale=Math.max(.75,scale-.25);if(action==='in')scale=Math.min(2,scale+.25);if(action==='reset')scale=1;if(action==='fullscreen'){if(document.fullscreenElement===panel)document.exitFullscreen();else if(panel.requestFullscreen)panel.requestFullscreen();else panel.classList.toggle('chart-expanded')}apply()});document.addEventListener('fullscreenchange',()=>{const button=panel.querySelector('[data-chart-action="fullscreen"]');button.textContent=document.fullscreenElement===panel?'退出全屏':'全屏'});apply()})})();</script>`;

function document(title, current, body, joint = null, modelScript = null) {
  const scripts=joint?`<script src="directional-friction.js?v=${assetVersion}"></script>${modelScript?`<script src="${modelScript}?v=${assetVersion}"></script>`:''}`:'';
  const subnav=joint?(current==='arm'?armJointNav(joint):jointNav(joint)):'';
  return `<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="referrer" content="no-referrer"><title>${title}</title><link rel="stylesheet" href="site.css?v=20261023-2">${scripts}</head><body>${nav(current)}${subnav}${body}${joint ? chartResizeScript : ""}</body></html>`;
}

copyFileSync(path.join(root, "src", "directional-friction.js"), path.join(dist, "directional-friction.js"));
copyFileSync(path.join(root, "src", "directional-friction.js"), path.join(root, "directional-friction.js"));
copyFileSync(path.join(root, "src", "arm-j3-model.js"), path.join(dist, "arm-j3-model.js"));
copyFileSync(path.join(root, "src", "arm-j3-model.js"), path.join(root, "arm-j3-model.js"));
copyFileSync(path.join(root, "site.css"), path.join(dist, "site.css"));

const models = [
  {id:"j2", title:"J2 二维机构简图", fragment:"j2.fragment.html", section:"leg"},
  {id:"j3", title:"J3 二维机构简图", fragment:"j3.fragment.html", section:"leg"},
  {id:"j4", title:"J4 二维机构简图", fragment:"j4.fragment.html", section:"leg"},
  {id:"arm-j3", title:"手臂 J3 机构简图", fragment:"arm-j3.fragment.html", section:"arm", modelScript:"arm-j3-model.js"},
];

for (const model of models) {
  const fragment = readFileSync(path.join(root, "src", model.fragment), "utf8");
  const page = document(`${model.title}｜${siteTitle}`, model.section, `<main class="model-shell">${fragment}</main>`, model.id, model.modelScript);
  writeFileSync(path.join(dist, `${model.id}.html`), page);
  writeFileSync(path.join(root, `${model.id}.html`), page);
}

const home = `<main class="model-shell"><section class="home-head"><h1>${siteTitle}</h1><p>按机器人模块组织机构模型，计算连接副阻力向控制模型等效旋转轴的折算结果，并比较各连接副的影响程度。</p></section><section class="system-modules" aria-label="机器人模块"><article class="system-module system-module-active" id="leg"><div class="module-heading"><div><span class="module-index">01</span><h2>腿足</h2></div><span class="module-status">已建立 J2–J4 模型</span></div><div class="joint-options"><a href="j2.html"><strong>J2</strong><span>髋部等效旋转轴</span><small>推杆、多连杆闭链与旋转副阻力折算</small></a><a href="j3.html"><strong>J3</strong><span>膝部等效旋转轴</span><small>J3–Q 输出、A 点几何闭合与推杆传动</small></a><a href="j4.html"><strong>J4</strong><span>踝部等效旋转轴</span><small>主动曲柄、鱼眼连接副与伸缩推杆组合</small></a></div></article><article class="system-module" id="head-neck"><div class="module-heading"><div><span class="module-index">02</span><h2>头颈</h2></div><span class="module-status module-status-pending">待建立机构模型</span></div></article><article class="system-module" id="torso"><div class="module-heading"><div><span class="module-index">03</span><h2>躯干</h2></div><span class="module-status module-status-pending">待建立机构模型</span></div></article><article class="system-module system-module-active" id="arm"><div class="module-heading"><div><span class="module-index">04</span><h2>手臂</h2></div><span class="module-status">已建立 J3 模型</span></div><div class="joint-options"><a href="arm-j3.html"><strong>J3</strong><span>手臂 J3 等效旋转轴</span><small>平行轴四杆闭环、三维推杆摆转与双向阻力折算</small></a></div></article></section><aside class="scope-note"><strong>计算范围：</strong>1 Hz、±10° 单关节激励；输出传动系数、静摩擦启动力矩、运行阻力和连接副贡献度。</aside></main>`;
const homePage = document(siteTitle, "index", home);
writeFileSync(path.join(dist, "index.html"), homePage);
writeFileSync(path.join(root, "index.html"), homePage);
