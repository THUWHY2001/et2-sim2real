import assert from "node:assert/strict";
import {evaluateLinearQuantity,solveRankRevealing} from "../src/rank-revealing-svd.mjs";

const near=(actual,expected,tolerance=1e-8,label="value")=>assert.ok(Math.abs(actual-expected)<=tolerance,`${label}: expected ${expected}, got ${actual}`);
const multiply=(A,x)=>A.map(row=>row.reduce((sum,value,index)=>sum+value*x[index],0));
const dot=(a,b)=>a.reduce((sum,value,index)=>sum+value*b[index],0);
const norm=value=>Math.hypot(...value);

{
  const solved=solveRankRevealing([[2,0],[0,4]],[6,8]);
  assert.equal(solved.consistent,true);
  assert.equal(solved.rank,2);
  assert.equal(solved.nullity,0);
  near(solved.particular[0],3,1e-10,"full-rank x0");
  near(solved.particular[1],2,1e-10,"full-rank x1");
  assert.equal(evaluateLinearQuantity(solved,[1,0]).status,"unique");
}

{
  const solved=solveRankRevealing([[1,2],[2,4]],[3,6]);
  assert.equal(solved.consistent,true);
  assert.equal(solved.rank,1);
  assert.equal(solved.nullity,1);
  assert.equal(evaluateLinearQuantity(solved,[1,0]).status,"indeterminate");
  assert.equal(evaluateLinearQuantity(solved,[1,0]).value,null);
  const invariant=evaluateLinearQuantity(solved,[1,2]);
  assert.equal(invariant.status,"unique");
  near(invariant.value,3,1e-8,"rank-deficient invariant");
}

{
  const solved=solveRankRevealing([[1,2],[2,4]],[3,7]);
  assert.equal(solved.consistent,false);
  assert.ok(solved.augmentedRank>solved.rank);
}

{
  const solved=solveRankRevealing([[1e-9,0],[0,1e9]],[2e-9,3e9]);
  assert.equal(solved.rank,2);
  near(solved.particular[0],2,1e-8,"scaled x0");
  near(solved.particular[1],3,1e-8,"scaled x1");
  assert.ok(Number.isFinite(solved.conditionNumber));
}

{
  const rows=120,columns=119,A=Array.from({length:rows},()=>Array(columns).fill(0));
  for(let index=0;index<117;index++)A[index][index]=10**((index%9)-4);
  for(let row=0;row<rows;row++){
    A[row][117]=A[row][0]+2*A[row][1];
    A[row][118]=-3*A[row][2]+A[row][3];
  }
  const x0=Array.from({length:columns},(_,index)=>((index%7)-3)/5),b=multiply(A,x0);
  const solved=solveRankRevealing(A,b,{relativeTolerance:1e-10});
  assert.equal(solved.svd.converged,true);
  assert.equal(solved.rank,117);
  assert.equal(solved.nullity,2);
  assert.equal(solved.nullspace.length,2);
  for(const vector of solved.nullspace)near(norm(multiply(A,vector)),0,1e-7,"large nullspace residual");
  for(let i=0;i<solved.nullspace.length;i++)for(let j=0;j<solved.nullspace.length;j++)near(dot(solved.nullspace[i],solved.nullspace[j]),i===j?1:0,1e-7,"large nullspace orthogonality");
  const unique=evaluateLinearQuantity(solved,A[0]);
  assert.equal(unique.status,"unique");
  near(unique.value,b[0],1e-7,"large scaled invariant");
  const perturbed=A.map(row=>row.slice());
  perturbed[0][0]*=1+1e-6;
  perturbed[1][1]*=1-1e-6;
  for(let row=0;row<rows;row++){
    perturbed[row][117]=perturbed[row][0]+2*perturbed[row][1];
    perturbed[row][118]=-3*perturbed[row][2]+perturbed[row][3];
  }
  const perturbedB=multiply(perturbed,x0);
  const perturbedSolved=solveRankRevealing(perturbed,perturbedB,{relativeTolerance:1e-10});
  assert.equal(perturbedSolved.rank,117);
  assert.equal(evaluateLinearQuantity(perturbedSolved,perturbed[0]).status,"unique");
}

console.log("rank-revealing SVD regression test passed");
