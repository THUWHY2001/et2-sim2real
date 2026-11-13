import fs from "node:fs";
import path from "node:path";

const endpoint="http://127.0.0.1:9223",pageUrl="http://127.0.0.1:8799/leg-friction.html",outDir=path.resolve("docs/verification/leg-friction");
fs.mkdirSync(outDir,{recursive:true});
const target=await fetch(`${endpoint}/json/new?${encodeURIComponent(pageUrl)}`,{method:"PUT"}).then(response=>response.json());
const socket=new WebSocket(target.webSocketDebuggerUrl),pending=new Map();let nextId=1;
await new Promise((resolve,reject)=>{socket.addEventListener("open",resolve,{once:true});socket.addEventListener("error",reject,{once:true})});
socket.addEventListener("message",event=>{const message=JSON.parse(event.data);if(message.id&&pending.has(message.id)){const {resolve,reject}=pending.get(message.id);pending.delete(message.id);message.error?reject(new Error(JSON.stringify(message.error))):resolve(message.result)}});
const call=(method,params={})=>new Promise((resolve,reject)=>{const id=nextId++;pending.set(id,{resolve,reject});socket.send(JSON.stringify({id,method,params}))});
const evaluate=async expression=>(await call("Runtime.evaluate",{expression,awaitPromise:true,returnByValue:true})).result.value;
const wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const waitFor=async(expression,timeout=30000)=>{const start=Date.now();while(Date.now()-start<timeout){if(await evaluate(expression))return;await wait(250)}throw new Error(`timeout: ${expression}`)};
const rectFor=async expression=>JSON.parse(await evaluate(`JSON.stringify((()=>{const e=${expression},r=e.getBoundingClientRect();return{x:Math.max(0,r.left+scrollX-12),y:Math.max(0,r.top+scrollY-12),width:Math.min(2176,r.width+24),height:r.height+24}})())`));
const capture=async(name,expression,maxHeight=1400)=>{const rect=await rectFor(expression);rect.height=Math.min(rect.height,maxHeight);const shot=await call("Page.captureScreenshot",{format:"png",captureBeyondViewport:true,fromSurface:true,clip:{...rect,scale:1}});fs.writeFileSync(path.join(outDir,name),Buffer.from(shot.data,"base64"))};

await call("Page.enable");await call("Runtime.enable");await call("Emulation.setDeviceMetricsOverride",{width:2200,height:1200,deviceScaleFactor:1,mobile:false});await call("Page.navigate",{url:pageUrl});
await waitFor(`document.querySelector('[data-leg-friction-example]') && document.querySelectorAll('[data-friction-joint]').length===25`);
await capture("01-scope-and-controls.png",`document.querySelector('[data-leg-friction-root]')`,720);
await evaluate(`document.querySelector('[data-leg-friction-example]').click();document.querySelector('[data-leg-friction-phase]').value='0.125';document.querySelector('[data-leg-friction-phase]').dispatchEvent(new Event('input',{bubbles:true}));true`);
await waitFor(`document.querySelector('[data-leg-friction-state]').textContent.includes('软件演示参数')`);
await evaluate(`document.querySelector('[data-leg-friction-export]').click();true`);await waitFor(`document.querySelector('[data-leg-friction-state]').textContent.includes('CSV 已导出')`);
await evaluate(`document.querySelectorAll('tr[data-friction-joint]').forEach(row=>row.hidden=!row.querySelector('[data-friction-mu-s]'));true`);
await capture("02-parameters-and-csv-export.png",`document.querySelector('[data-leg-friction-contact-table]').closest('section')`,1100);
for(const output of ["J2","J3","J4"]){
  await evaluate(`(()=>{const select=document.querySelector('[data-leg-friction-output]');select.value='${output}';select.dispatchEvent(new Event('change',{bubbles:true}));document.querySelector('[data-leg-friction-run]').click();return true})()`);
  await waitFor(`document.querySelector('[data-leg-friction-run]').textContent==='计算完整周期'`,60000);
  await waitFor(`document.querySelectorAll('[data-leg-friction-cycle-chart] path').length>=2`);
  await evaluate(`scrollTo(0,0);true`);
  await capture(`03-contribution-${output.toLowerCase()}.png`,`document.querySelector('[data-leg-friction-bars]').closest('section')`,1200);
}
socket.close();console.log(`acceptance screenshots written to ${outDir}`);
