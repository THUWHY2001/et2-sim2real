import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";

class Element {
  constructor(dataset={}){this.dataset={...dataset};this.value="0";this.textContent="";this.listeners={};}
  addEventListener(type,listener){(this.listeners[type]??=[]).push(listener)}
  dispatch(type){for(const listener of this.listeners[type]??[])listener({target:this})}
}

const inputs=[new Element({linkLength:"short"}),new Element({linkLength:"long"})];
const outputs=[new Element({linkLengthBase:"short"}),new Element({linkLengthBase:"long"})];
const status=new Element(),reset=new Element();
const root={
  querySelectorAll(selector){if(selector==="[data-link-length]")return inputs;if(selector==="[data-link-length-base]")return outputs;return[]},
  querySelector(selector){if(selector==="[data-link-length-status]")return status;if(selector==="[data-link-length-reset]")return reset;return null},
};

const context={window:{}};
vm.createContext(context);
vm.runInContext(readFileSync("src/link-length-controls.js","utf8"),context);
const events=[];
const controller=context.window.LinkLengthControls.mount({root,lengths:{short:80,long:350},links:[{key:"short",label:"短杆"},{key:"long",label:"长杆"}],onChange:active=>{events.push({...active});if(active.short<1)throw new Error("闭环无法成立")}});

assert.equal(outputs[0].textContent,"80.00 mm","standard short length is displayed");
assert.equal(outputs[1].textContent,"350.00 mm","standard long length is displayed");
assert.equal(status.textContent,"标准态｜全部 ΔL = 0 mm","initial state is standard");

inputs[0].value="1.5";inputs[0].dispatch("input");
assert.deepEqual(events.at(-1),{short:81.5,long:350},"input change emits active physical lengths");
assert.match(status.textContent,/变体态.*短杆 ΔL=1\.50 mm.*闭环有效/,"valid variation is reported");

inputs[0].value="-100";inputs[0].dispatch("input");
assert.match(status.textContent,/当前尺寸无解：闭环无法成立/,"invalid closure is visible");

reset.dispatch("click");
assert.deepEqual(inputs.map(input=>input.value),["0","0"],"reset clears every delta");
assert.deepEqual(JSON.parse(JSON.stringify(controller.current())),{short:80,long:350},"reset restores standard physical lengths");
assert.equal(status.textContent,"标准态｜全部 ΔL = 0 mm","reset restores standard status");
console.log("link length controls regression test passed");
