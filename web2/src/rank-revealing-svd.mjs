const EPS=Number.EPSILON;
const dot=(a,b)=>a.reduce((sum,value,index)=>sum+value*b[index],0);
const norm=value=>Math.hypot(...value);
const identity=size=>Array.from({length:size},(_,row)=>Array.from({length:size},(_,column)=>row===column?1:0));
const transpose=matrix=>matrix[0].map((_,column)=>matrix.map(row=>row[column]));
const multiplyVector=(matrix,vector)=>matrix.map(row=>dot(row,vector));

function validate(A,b){
  if(!Array.isArray(A)||!A.length||!Array.isArray(A[0])||!A[0].length)throw new Error("A must be a non-empty matrix");
  const columns=A[0].length;
  if(A.some(row=>row.length!==columns||row.some(value=>!Number.isFinite(value))))throw new Error("A must be rectangular and finite");
  if(!Array.isArray(b)||b.length!==A.length||b.some(value=>!Number.isFinite(value)))throw new Error("b must match A rows and be finite");
}

function jacobiSvd(matrix,options={}){
  const m=matrix.length,n=matrix[0].length,B=matrix.map(row=>row.slice()),V=identity(n);
  const tolerance=Number(options.jacobiTolerance)||1e-13,maxSweeps=Math.max(20,Number(options.maxSweeps)||Math.max(40,8*n));
  const frobeniusSquared=B.reduce((sum,row)=>sum+dot(row,row),0),zeroEnergy=tolerance*tolerance*frobeniusSquared;
  let sweeps=0,converged=false;
  for(;sweeps<maxSweeps;sweeps++){
    let maxCorrelation=0,changed=false;
    for(let p=0;p<n-1;p++)for(let q=p+1;q<n;q++){
      let alpha=0,beta=0,gamma=0;
      for(let row=0;row<m;row++){const x=B[row][p],y=B[row][q];alpha+=x*x;beta+=y*y;gamma+=x*y}
      if(alpha<=zeroEnergy||beta<=zeroEnergy)continue;
      const correlation=Math.abs(gamma)/Math.sqrt(alpha*beta);maxCorrelation=Math.max(maxCorrelation,correlation);
      if(correlation<=tolerance)continue;
      const zeta=(beta-alpha)/(2*gamma),t=(zeta>=0?1:-1)/(Math.abs(zeta)+Math.sqrt(1+zeta*zeta)),c=1/Math.sqrt(1+t*t),s=c*t;
      for(let row=0;row<m;row++){const x=B[row][p],y=B[row][q];B[row][p]=c*x-s*y;B[row][q]=s*x+c*y}
      for(let row=0;row<n;row++){const x=V[row][p],y=V[row][q];V[row][p]=c*x-s*y;V[row][q]=s*x+c*y}
      changed=true;
    }
    if(!changed||maxCorrelation<=tolerance){converged=true;break}
  }
  const entries=Array.from({length:n},(_,column)=>({
    sigma:Math.hypot(...B.map(row=>row[column])),
    v:V.map(row=>row[column]),
    b:B.map(row=>row[column]),
  })).sort((a,b)=>b.sigma-a.sigma);
  const singularValues=entries.map(entry=>entry.sigma),rightVectors=entries.map(entry=>entry.v),leftVectors=entries.map(entry=>entry.sigma?entry.b.map(value=>value/entry.sigma):Array(m).fill(0));
  return{singularValues,rightVectors,leftVectors,sweeps,converged};
}

function orthonormalize(vectors,tolerance=1e-12){
  const result=[];
  for(const source of vectors){
    const vector=source.slice();
    for(const basis of result){const projection=dot(vector,basis);for(let index=0;index<vector.length;index++)vector[index]-=projection*basis[index]}
    const length=norm(vector);if(length>tolerance)result.push(vector.map(value=>value/length));
  }
  return result;
}

export function solveRankRevealing(A,b,options={}){
  validate(A,b);
  const rows=A.length,columns=A[0].length,relativeTolerance=Number(options.relativeTolerance)||1e-10;
  const rowScales=A.map(row=>{const length=norm(row);return length?1/length:1});
  const rowScaled=A.map((row,index)=>row.map(value=>value*rowScales[index])),bScaled=b.map((value,index)=>value*rowScales[index]);
  const columnNorms=Array.from({length:columns},(_,column)=>Math.hypot(...rowScaled.map(row=>row[column]))),columnScales=columnNorms.map(value=>value||1);
  const scaled=rowScaled.map(row=>row.map((value,column)=>value/columnScales[column]));
  const svd=jacobiSvd(scaled,options),largest=svd.singularValues[0]||0,threshold=Math.max(relativeTolerance*largest,EPS*Math.max(rows,columns)*largest),rank=svd.singularValues.filter(value=>value>threshold).length;
  const y=Array(columns).fill(0);
  for(let index=0;index<rank;index++){
    const coefficient=dot(svd.leftVectors[index],bScaled)/svd.singularValues[index],v=svd.rightVectors[index];
    for(let column=0;column<columns;column++)y[column]+=coefficient*v[column];
  }
  const particular=y.map((value,column)=>value/columnScales[column]);
  const rawNull=svd.rightVectors.slice(rank).map(vector=>vector.map((value,column)=>value/columnScales[column])),nullspace=orthonormalize(rawNull);
  const residualVector=multiplyVector(A,particular).map((value,index)=>value-b[index]),residualNorm=norm(residualVector),matrixNorm=Math.sqrt(A.reduce((sum,row)=>sum+dot(row,row),0)),rhsNorm=norm(b),solutionNorm=norm(particular),consistencyTolerance=(Number(options.consistencyTolerance)||1e-9)*(matrixNorm*solutionNorm+rhsNorm+1),consistent=residualNorm<=consistencyTolerance;
  const smallest=rank?svd.singularValues[rank-1]:0,conditionNumber=rank&&smallest?largest/smallest:Infinity;
  return{
    consistent,rank,augmentedRank:rank+(consistent?0:1),nullity:columns-rank,conditionNumber,
    singularValues:svd.singularValues,threshold,particular,
    nullspace,residualVector,residualNorm,scaling:{rowScales,columnScales},svd:{sweeps:svd.sweeps,converged:svd.converged},
  };
}

export function evaluateLinearQuantity(solution,coefficients,options={}){
  if(!solution?.consistent)return{status:"inconsistent",value:null,nullProjection:Infinity};
  if(!Array.isArray(coefficients)||coefficients.length!==solution.particular.length)throw new Error("coefficient length must match solution unknowns");
  const projections=solution.nullspace.map(vector=>dot(coefficients,vector)),nullProjection=norm(projections),scale=Math.max(1,norm(coefficients)),tolerance=(Number(options.uniquenessTolerance)||1e-8)*scale;
  if(nullProjection>tolerance)return{status:"indeterminate",value:null,nullProjection,projections};
  return{status:"unique",value:dot(coefficients,solution.particular),nullProjection,projections};
}

export const __test={jacobiSvd,orthonormalize};
