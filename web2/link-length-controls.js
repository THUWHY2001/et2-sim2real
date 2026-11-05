(()=>{
  function mount({root,lengths,links,onChange}){
    const inputs=[...root.querySelectorAll('[data-link-length]')],status=root.querySelector('[data-link-length-status]'),resetButton=root.querySelector('[data-link-length-reset]');
    const byKey=Object.fromEntries(inputs.map(input=>[input.dataset.linkLength,input]));
    const current=()=>Object.fromEntries(links.map(({key})=>[key,lengths[key]+(Number(byKey[key]?.value)||0)]));
    const show=active=>{
      const changed=links.filter(({key})=>Math.abs(active[key]-lengths[key])>1e-9);
      status.dataset.state=changed.length?'variant':'standard';
      status.textContent=changed.length?`变体态｜${changed.map(({key,label})=>`${label} ΔL=${(active[key]-lengths[key]).toFixed(2)} mm`).join('｜')}｜闭环有效`:'标准态｜全部 ΔL = 0 mm';
    };
    const fail=error=>{status.dataset.state='invalid';status.textContent=`当前尺寸无解：${error.message}`};
    const notify=()=>{const active=current();try{onChange(active);show(active);return true}catch(error){fail(error);return false}};
    root.querySelectorAll('[data-link-length-base]').forEach(output=>{output.textContent=lengths[output.dataset.linkLengthBase].toFixed(2)+' mm'});
    inputs.forEach(input=>input.addEventListener('input',notify));
    const reset=()=>{inputs.forEach(input=>input.value='0');notify()};
    resetButton.addEventListener('click',reset);
    show(current());
    return{current,show,fail,reset,refresh:notify};
  }
  window.LinkLengthControls={mount};
})();
