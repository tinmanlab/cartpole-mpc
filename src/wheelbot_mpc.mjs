// Exact LQR-coordinate transcription of the finite-horizon MIMO torque QP.
// No optimizer here: pinned upstream quadprog 1.6.1 uses 1-indexed arrays,
// min .5 v'Hv - d'v, C'v >= b. No state/contact/friction constraints.
import '../vendor/quadprog/quadprog.js';
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),mv=(A,x)=>A.map(r=>dot(r,x));
const zeros=(n,m)=>Array.from({length:n},()=>Array(m).fill(0));
const mul=(A,B)=>A.map(r=>B[0].map((_,j)=>r.reduce((s,v,k)=>s+v*B[k][j],0)));
const T=A=>A[0].map((_,i)=>A.map(r=>r[i]));
const norm=x=>Math.max(0,...x.flat().map(Math.abs));
const vec=(x,n)=>Array.isArray(x)&&x.length===n&&x.every(Number.isFinite);
const mat=(x,n,m)=>Array.isArray(x)&&x.length===n&&x.every(r=>vec(r,m));
export function createWheelbotMPC(profile,limits,{N=20}={}){
 if(!profile||typeof profile!=='object'||!Array.isArray(limits))throw Error('Invalid MPC profile/limits');
 const p=structuredClone(profile);limits=limits.slice();
 if(!Number.isInteger(N)||N<1||N>20||!vec(limits,3)||limits.some(v=>v<=0)||!vec(p.uref,3)||!vec(p.qref,6)||!mat(p.A,11,11)||!mat(p.B,11,3)||!mat(p.Q,11,11)||!mat(p.P,11,11)||!mat(p.R,3,3)||!mat(p.K,3,11)||p.controlDt!==.01||!p.designAvailable)throw Error('Invalid MPC profile/dimensions');
 if(p.uref.some((v,i)=>Math.abs(v)>limits[i])||JSON.stringify(limits)!==JSON.stringify(p.limitsNm)||JSON.stringify(p.controlledIndices)!==JSON.stringify([0,1,2,3,4,6,7,8,9,10,11]))throw Error('Invalid MPC profile mapping/limits');
 for(const W of [p.Q,p.R,p.P])if(norm(W.map((r,i)=>r.map((v,j)=>v-W[j][i])))>1e-8)throw Error('Invalid asymmetric cost');
 if(p.assetSha256!=='fe62740619c7fdc2a744a27ffcb4b4589cedee5cf3af39c404e66198faa6961a'||![p.dareNormalizedResidual,p.closedLoopRadius,p.trimQaccInf].every(Number.isFinite)||p.dareNormalizedResidual>1e-8||p.closedLoopRadius>=1||p.trimQaccInf>1e-7)throw Error('Invalid MPC profile mismatch/design');
 const {A,B,Q,R,P,K}=p,nv=3*N,BK=mul(B,K),F=A.map((r,i)=>r.map((v,j)=>v-BK[i][j]));
 // World-x translation is a symmetry: A[:,0]=unit_x (also checked on plant).
 if(norm(A.map((r,i)=>r[0]-(i===0?1:0)))>1e-7)throw Error('Invalid translation symmetry');
 const G=[zeros(11,nv)],M=[],H=zeros(nv,nv);
 function costMap(G,W){const WG=mul(W,G);for(let i=0;i<nv;i++)for(let j=0;j<nv;j++)for(let a=0;a<G.length;a++)H[i][j]+=2*G[a][i]*WG[a][j];}
 for(let k=0;k<N;k++){
  const U=mul(K,G[k]).map(r=>r.map(v=>-v));for(let i=0;i<3;i++)U[i][3*k+i]+=1;M.push(U);
  costMap(G[k],Q);costMap(U,R);const next=mul(F,G[k]);for(let i=0;i<11;i++)for(let j=0;j<3;j++)next[i][3*k+j]+=B[i][j];G.push(next);
 }
 costMap(G[N],P);for(let i=0;i<nv;i++)for(let j=0;j<i;j++)H[i][j]=H[j][i]=(H[i][j]+H[j][i])/2;
 const columns=M.flatMap(U=>U.flatMap(r=>[r,r.map(v=>-v)]));
 const ref=p.controlledIndices.map(i=>i<6?p.qref[i]:0);
 return {solve(estimate,goal=0){
  const start=performance.now();
  if(!vec(estimate,11)||!Number.isFinite(goal))throw Error('Invalid MPC state/goal');
  const e0=estimate.map((v,i)=>v-ref[i]-(i===0?goal:0)),offset=[e0],uoff=[],f=Array(nv).fill(0),lower=[];
  for(let k=0;k<=N;k++){
   const w=mv(k===N?P:Q,offset[k]);for(let j=0;j<nv;j++)for(let i=0;i<11;i++)f[j]+=2*G[k][i][j]*w[i];
   if(k<N){const u=mv(K,offset[k]).map(v=>-v);uoff.push(u);const ru=mv(R,u);for(let j=0;j<nv;j++)for(let i=0;i<3;i++)f[j]+=2*M[k][i][j]*ru[i];offset.push(mv(F,offset[k]));for(let i=0;i<3;i++)lower.push(-limits[i]-p.uref[i]-u[i],-limits[i]+p.uref[i]+u[i]);}
  }
  if(!f.every(Number.isFinite)||!lower.every(Number.isFinite))throw Error('QP rejected: nonfinite assembled data');
  const result=globalThis.Quadprog.solveQP([[],...H.map(r=>[0,...r])],[0,...f.map(v=>-v)],[[],...T(columns).map(r=>[0,...r])],[0,...lower],0);
  if(!result||typeof result!=='object')throw Error('QP rejected: invalid solver command');
  if(result.message)throw Error('QP rejected: '+result.message);
  if(!Array.isArray(result.solution)||!Array.isArray(result.Lagrangian))throw Error('QP rejected: invalid solver command');
  const V=result.solution.slice(1),lambda=result.Lagrangian.slice(1);
  if(!vec(V,nv)||!vec(lambda,6*N))throw Error('QP rejected: invalid solver command');
  const slack=columns.map((r,i)=>dot(r,V)-lower[i]),gradient=mv(H,V).map((v,i)=>v+f[i]-columns.reduce((s,r,j)=>s+r[i]*lambda[j],0));
  const primalResidual=Math.max(0,...slack.map(v=>-v)),complementarity=norm(slack.map((v,i)=>v*lambda[i]));
  const kktResidual=Math.max(norm(gradient),complementarity,...lambda.map(v=>-v));
  if(kktResidual>1e-7||primalResidual>1e-8)throw Error('QP rejected: KKT/primal residual '+JSON.stringify({kktResidual,primalResidual}));
  const E=offset.map((z,k)=>z.map((v,i)=>v+dot(G[k][i],V))),DU=uoff.map((u,k)=>u.map((v,i)=>v+dot(M[k][i],V))),U=DU.map(u=>u.map((v,i)=>v+p.uref[i]));
  const physicalViolation=Math.max(0,...U.flatMap(u=>u.map((v,i)=>Math.abs(v)-limits[i])));
  const dynamicsDefect=norm(E.slice(0,N).map((e,k)=>mv(A,e).map((v,i)=>v+dot(B[i],DU[k])-E[k+1][i])));
  let adj=mv(P,E[N]).map(v=>2*v),originalStationarity=0,J=dot(E[N],mv(P,E[N]));
  for(let k=N-1;k>=0;k--){const ru=mv(R,DU[k]),bt=mv(T(B),adj);originalStationarity=Math.max(originalStationarity,norm(ru.map((v,i)=>2*v+bt[i]-lambda[6*k+2*i]+lambda[6*k+2*i+1])));const qx=mv(Q,E[k]);adj=mv(T(A),adj).map((v,i)=>v+2*qx[i]);J+=dot(E[k],qx)+dot(DU[k],ru);}
  if(!Number.isFinite(J)||physicalViolation>1e-8||dynamicsDefect>1e-8||originalStationarity>1e-7)throw Error('QP rejected: original-unit reconstruction/stationarity');
  const forecastActiveCountsByMotor=[0,1,2].map(j=>U.filter(u=>Math.abs(u[j])>=limits[j]-1e-8).length);
  return {forecastActiveCountsByMotor,forecastConstraintActive:forecastActiveCountsByMotor.some(n=>n>0),accepted:true,u:U[0],requested:U[0].slice(),saturated:U[0].some((v,i)=>Math.abs(v)>=limits[i]-1e-8),E,U,J,kktResidual,primalResidual,complementarity,physicalViolation,dynamicsDefect,originalStationarity,solveMs:performance.now()-start,solver:'quadprog 1.6.1 / LQR coordinates',horizon:N};
 }};
}
