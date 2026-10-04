'use strict';
// MPC transcription only. The numerical solver is the unmodified upstream
// quadprog@1.6.1 Goldfarb-Idnani implementation (MIT); see THIRD_PARTY_NOTICES.
const MpcQP=(()=>{
  const qp=typeof module!=='undefined'?require('quadprog'):globalThis.Quadprog;
  const cache=new Map(),clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
  const mv=(A,x)=>A.map(r=>r.reduce((s,a,j)=>s+a*x[j],0));
  const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
  function condensed(A,B,Q,R,Qf,N){
    const key=JSON.stringify([A,B,Q,R,Qf,N]);if(cache.has(key))return cache.get(key);
    const n=A.length,H=Array.from({length:N},(_,i)=>Array.from({length:N},(_,j)=>i===j?2*R:0)),Gs=[];
    let G=Array.from({length:n},()=>Array(N).fill(0));
    for(let k=0;k<=N;k++){
      Gs.push(G);const W=k===N?Qf:Q;
      for(let i=0;i<N;i++)for(let j=0;j<N;j++)for(let a=0;a<n;a++)for(let b=0;b<n;b++)H[i][j]+=2*G[a][i]*W[a][b]*G[b][j];
      if(k<N){const next=A.map(row=>Array.from({length:N},(_,j)=>row.reduce((s,a,i)=>s+a*G[i][j],0)));for(let i=0;i<n;i++)next[i][k]+=B[i][0];G=next;}
    }
    const value={H,Gs};if(cache.size>=32)cache.delete(cache.keys().next().value);cache.set(key,value);return value;
  }
  function solve(A,B,Q,R,Qf,x0,N,limit,stateBound=null){
    if(!qp||typeof qp.solveQP!=='function')throw Error('Pinned upstream QP solver is unavailable');
    if(!x0.every(Number.isFinite)||!Number.isFinite(limit)||limit<=0||!Number.isFinite(R)||R<=0)throw Error('Invalid MPC problem');
    const rail=stateBound?.positionLimit,goal=stateBound?.goal??0;
    if(stateBound&&(!Number.isFinite(rail)||rail<=0||!Number.isFinite(goal)||Math.abs(x0[0]+goal)>rail+1e-10))throw Error('QP rejected: initial state outside physical rail');
    const {H,Gs}=condensed(A,B,Q,R,Qf,N),f=Array(N).fill(0),offsets=[];let x=x0.slice();
    for(let k=0;k<=N;k++){
      offsets.push(x);const W=k===N?Qf:Q,wx=mv(W,x),G=Gs[k];
      for(let j=0;j<N;j++)for(let i=0;i<x.length;i++)f[j]+=2*G[i][j]*wx[i];
      x=mv(A,x);
    }
    // Each column describes a'U >= b. State offsets are in WORLD coordinates,
    // even when the tracking objective uses goal-relative state.
    const columns=[],lower=[];
    for(const sign of [1,-1])for(let i=0;i<N;i++){columns.push(Array.from({length:N},(_,j)=>i===j?sign:0));lower.push(-limit);}
    if(stateBound)for(let k=1;k<=N;k++){
      columns.push(Gs[k][0].slice());lower.push(-rail-offsets[k][0]-goal);
      columns.push(Gs[k][0].map(v=>-v));lower.push(offsets[k][0]+goal-rail);
    }
    // Upstream API is ONE-indexed and minimizes .5 U'HU - d'U.
    const D=[[],...H.map(r=>[0,...r])],d=[0,...f.map(v=>-v)],b=[0,...lower],C=[[]];
    for(let i=0;i<N;i++)C.push([0,...columns.map(c=>c[i])]);
    const sol=qp.solveQP(D,d,C,b,0);
    if(sol.message)throw Error('QP rejected: '+sol.message);
    const U=sol.solution.slice(1),lambda=sol.Lagrangian.slice(1);
    if(U.length!==N||!U.every(Number.isFinite)||!lambda.every(Number.isFinite))throw Error('QP returned invalid solution');
    const gradient=mv(H,U).map((v,i)=>v+f[i]),slack=columns.map((c,j)=>dot(c,U)-lower[j]);
    const stationarity=Math.max(...gradient.map((v,i)=>Math.abs(v-columns.reduce((s,c,j)=>s+c[i]*lambda[j],0))));
    const complementarity=Math.max(...slack.map((s,j)=>Math.abs(s*lambda[j]))),dualViolation=Math.max(0,...lambda.map(v=>-v));
    const kktResidual=Math.max(stationarity,complementarity,dualViolation),primalResidual=Math.max(0,...slack.map(v=>-v));
    if(kktResidual>1e-7||primalResidual>1e-8)throw Error('QP acceptance residual exceeded');
    return {U:U.map(u=>clamp(u,-limit,limit)),iterations:sol.iterations?.[1]??0,converged:true,kktResidual,primalResidual,
      stateConstrained:!!stateBound,solver:'quadprog-goldfarb-idnani'};
  }
  return {solve};
})();
if(typeof module!=='undefined')module.exports=MpcQP;
