(()=>{
  function toleranceForLength(length){
    if(!Number.isFinite(length)||length<=0)throw new Error('杆长必须大于 0');
    return Math.max(1,Math.floor(length/100));
  }

  function solve3(matrix,vector){
    const a=matrix.map((row,index)=>[...row,vector[index]]);
    for(let column=0;column<3;column++){
      let pivot=column;
      for(let row=column+1;row<3;row++)if(Math.abs(a[row][column])>Math.abs(a[pivot][column]))pivot=row;
      [a[column],a[pivot]]=[a[pivot],a[column]];
      if(Math.abs(a[column][column])<1e-14)throw new Error('拟合矩阵不可逆');
      for(let row=column+1;row<3;row++){
        const factor=a[row][column]/a[column][column];
        for(let k=column;k<4;k++)a[row][k]-=factor*a[column][k];
      }
    }
    const result=[0,0,0];
    for(let row=2;row>=0;row--){
      let value=a[row][3];
      for(let k=row+1;k<3;k++)value-=a[row][k]*result[k];
      result[row]=value/a[row][row];
    }
    return result;
  }

  function quadraticFit(points){
    if(!Array.isArray(points)||points.length<3)throw new Error('二次拟合至少需要 3 个有效点');
    const sx=points.reduce((s,p)=>s+p.delta,0),sx2=points.reduce((s,p)=>s+p.delta**2,0),sx3=points.reduce((s,p)=>s+p.delta**3,0),sx4=points.reduce((s,p)=>s+p.delta**4,0),sy=points.reduce((s,p)=>s+p.torque,0),sxy=points.reduce((s,p)=>s+p.delta*p.torque,0),sx2y=points.reduce((s,p)=>s+p.delta**2*p.torque,0);
    const [a,b,c]=solve3([[sx4,sx3,sx2],[sx3,sx2,sx],[sx2,sx,points.length]],[sx2y,sxy,sy]),mean=sy/points.length,ssTotal=points.reduce((s,p)=>s+(p.torque-mean)**2,0),ssResidual=points.reduce((s,p)=>s+(p.torque-(a*p.delta**2+b*p.delta+c))**2,0),r2=ssTotal<1e-20?1:1-ssResidual/ssTotal;
    return{a,b,c,r2,predict:delta=>a*delta**2+b*delta+c};
  }

  const formatDecimal=(value,digits=6)=>Number(value).toFixed(digits);

  function analyze({model,links,loads,direction=1,sampleCount=41}){
    return links.map(link=>{
      const standardLength=model.lengths[link.key];
      if(!Number.isFinite(standardLength))throw new Error(`未知连杆：${link.key}`);
      const tolerance=toleranceForLength(standardLength),count=Math.max(3,Math.round(sampleCount));
      let invalidCount=0;
      const points=[];
      for(let index=0;index<count;index++){
        const delta=-tolerance+2*tolerance*index/(count-1),overrides={...model.lengths,[link.key]:standardLength+delta};
        try{
          let torque=-Infinity,peakAngle=0;
          for(let angleIndex=0;angleIndex<=200;angleIndex++){
            const angle=-10+angleIndex*.1,total=model.equivalentTorque(model.solve(angle,overrides),loads,direction).total;
            if(!Number.isFinite(total))throw new Error('启动力矩不是有限数');
            if(total>torque){torque=total;peakAngle=angle;}
          }
          points.push({delta,torque,peakAngle});
        }catch(error){invalidCount++;}
      }
      const fit=quadraticFit(points),standard=points.reduce((best,point)=>Math.abs(point.delta)<Math.abs(best.delta)?point:best),maxChangePercent=Math.max(...points.map(point=>Math.abs(point.torque-standard.torque)))/(Math.abs(standard.torque)||1)*100;
      return{...link,standardLength,tolerance,points,invalidCount,fit,standard,sensitivity:fit.b,maxChangePercent};
    });
  }

  function ensureStyles(){
    if(typeof document==='undefined'||document.getElementById('link-tolerance-styles'))return;
    const style=document.createElement('style');
    style.id='link-tolerance-styles';
    style.textContent='.link-tolerance-section{margin-top:22px;padding-top:18px;border-top:1px solid var(--border)}.link-tolerance-head h2{margin:0 0 4px}.link-tolerance-head p{margin:0 0 12px}.link-tolerance-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px}.link-tolerance-card{padding:12px;min-width:0}.link-tolerance-card h3{margin:0 0 2px}.link-tolerance-meta{margin:0 0 6px}.link-tolerance-chart{display:block;width:100%;height:auto;max-height:280px}.link-tolerance-chart [data-chart-frame]{fill:none;stroke:var(--border);stroke-width:1}.link-tolerance-chart text{fill:var(--foreground);font-size:12px;font-weight:400}.link-tolerance-chart .tolerance-grid line{stroke:var(--border);stroke-width:1}.link-tolerance-chart .tolerance-grid text{fill:var(--muted-foreground)}.link-tolerance-chart .tolerance-fit{fill:none;stroke:var(--viz-series-1);stroke-width:2.5}.link-tolerance-chart .tolerance-point{fill:var(--viz-series-3)}.link-tolerance-chart .tolerance-zero{stroke:var(--muted-foreground);stroke-width:1;stroke-dasharray:4 4}.link-tolerance-summary{margin:5px 0 0}@media(max-width:780px){.link-tolerance-grid{grid-template-columns:1fr}}';
    document.head.appendChild(style);
  }

  function coefficient(value){
    const magnitude=Math.abs(value),digits=magnitude&&magnitude<.001?8:magnitude<.01?6:4;
    return formatDecimal(value,digits);
  }
  const signedCoefficient=value=>`${value<0?'−':'+'} ${coefficient(Math.abs(value))}`;

  function cardHtml(item,jointLabel){
    const {fit}=item,width=520,height=255,left=66,right=18,top=25,bottom=48,plotWidth=width-left-right,plotHeight=height-top-bottom,fitPoints=Array.from({length:101},(_,index)=>{const delta=-item.tolerance+2*item.tolerance*index/100;return{delta,torque:fit.predict(delta)}}),allTorques=[...item.points.map(point=>point.torque),...fitPoints.map(point=>point.torque)],rawMin=Math.min(...allTorques),rawMax=Math.max(...allTorques),padding=Math.max((rawMax-rawMin)*.12,.002),yMin=rawMin-padding,yMax=rawMax+padding,x=delta=>left+(delta+item.tolerance)/(2*item.tolerance)*plotWidth,y=torque=>top+(yMax-torque)/(yMax-yMin)*plotHeight,path=fitPoints.map((point,index)=>`${index?'L':'M'}${x(point.delta).toFixed(2)} ${y(point.torque).toFixed(2)}`).join(' '),dots=item.points.map(point=>`<circle class="tolerance-point" cx="${x(point.delta).toFixed(2)}" cy="${y(point.torque).toFixed(2)}" r="1.7"></circle>`).join(''),yTicks=Array.from({length:5},(_,index)=>{const value=yMin+(yMax-yMin)*index/4,py=y(value);return`<line x1="${left}" y1="${py.toFixed(2)}" x2="${width-right}" y2="${py.toFixed(2)}"></line><text x="${left-8}" y="${(py+4).toFixed(2)}" text-anchor="end">${value.toFixed(3)}</text>`}).join(''),xTicks=[-item.tolerance,0,item.tolerance].map(value=>`<line x1="${x(value)}" y1="${height-bottom}" x2="${x(value)}" y2="${height-bottom+6}"></line><text x="${x(value)}" y="${height-24}" text-anchor="middle">${value>0?'+':''}${value}</text>`).join(''),minTorque=Math.min(...item.points.map(point=>point.torque)),maxTorque=Math.max(...item.points.map(point=>point.torque)),invalid=item.invalidCount?`｜${item.invalidCount} 个公差点无解`:'';
    return `<article class="card link-tolerance-card" data-chart-panel><h3>${item.label}</h3><p class="link-tolerance-meta text-small text-muted">标准长度 ${item.standardLength.toFixed(2)} mm｜公差范围 ±${item.tolerance.toFixed(0)} mm${invalid}</p><div class="chart-tools" aria-label="${item.label}公差图显示控制"><span>图表大小</span><button type="button" data-chart-action="out" aria-label="缩小${item.label}公差图">−</button><output data-chart-scale>100%</output><button type="button" data-chart-action="in" aria-label="放大${item.label}公差图">＋</button><button type="button" data-chart-action="reset">还原</button><button type="button" data-chart-action="fullscreen">全屏</button></div><div class="chart-viewport"><div class="chart-stage" data-chart-stage><svg class="link-tolerance-chart" viewBox="0 0 ${width} ${height}" role="img" aria-label="${item.label}长度公差与${jointLabel}峰值等效启动力矩"><rect data-chart-frame x="${left}" y="${top}" width="${plotWidth}" height="${plotHeight}"></rect><g class="tolerance-grid" aria-hidden="true">${yTicks}${xTicks}<text x="${width/2}" y="${height-4}" text-anchor="middle">长度偏差 ΔL / mm</text><text x="16" y="${top+plotHeight/2}" transform="rotate(-90 16 ${top+plotHeight/2})" text-anchor="middle">峰值等效启动力矩 / N·m</text></g><line class="tolerance-zero" x1="${x(0)}" y1="${top}" x2="${x(0)}" y2="${height-bottom}"></line><path class="tolerance-fit" d="${path}"></path><g>${dots}</g></svg></div></div><p class="link-tolerance-summary text-small">标准态 ${item.standard.torque.toFixed(3)} N·m｜公差区间 ${minTorque.toFixed(3)}–${maxTorque.toFixed(3)} N·m<br>拟合：τ = ${coefficient(fit.a)}ΔL² ${signedCoefficient(fit.b)}ΔL ${signedCoefficient(fit.c)}，R²=${fit.r2.toFixed(5)}<br>标准态灵敏度 ${formatDecimal(item.sensitivity,6)} N·m/mm｜最大变化率 ${item.maxChangePercent.toFixed(3)}%</p></article>`;
  }

  function render({host,model,links,loads,direction=1,jointLabel='等效轴'}){
    if(!host)throw new Error('缺少公差图容器');
    ensureStyles();
    const results=analyze({model,links,loads,direction});
    host.innerHTML=results.map(item=>cardHtml(item,jointLabel)).join('');
    if(window.initChartResizeControls)window.initChartResizeControls(host);
    return results;
  }

  window.LinkTolerance={toleranceForLength,quadraticFit,formatDecimal,analyze,render};
})();
