// A coordinate transformation/transcription, NOT a new numerical solver.
// u_k = -K e_k + v_k; e_(k+1)=(A-BK)e_k+Bv_k.
// Full original finite-horizon objective/physical constraints are retained.
// Uses pinned upstream quadprog (Goldfarb-Idnani), as does the direct formulation.
const require=typeof window==='undefined'?(await import('node:module')).createRequire(import.meta.url):null;
const L=require?require('./engine.js'):ControlLab,qp=require?require('quadprog'):globalThis.Quadprog;
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),mv=(A,x)=>A.map(r=>dot(r,x));
const zeros=(n,m)=>Array.from({length:n},()=>Array(m).fill(0));
const norm=x=>Math.max(0,...x.map(Math.abs));
export function createPrestabilizedQP(p){
 if(!p.designAvailable)throw Error('Unadmitted design: Riccati numerical gate');
 const n=p.nx,Hn=p.horizon,A=p.A,B=p.B,Q=p.Q,P=p.P,R=p.R,K=p.K[0],F=A.map((r,i)=>r.map((v,j)=>v-B[i][0]*K[j]));
 if(!Number.isInteger(Hn)||Hn<1||!Number.isFinite(R)||R<=0||K.length!==n)throw Error('Invalid pre-stabilized profile');
 const Xmap=[zeros(n,Hn)],Umap=[],H= zeros(Hn,Hn);
 function addQuadratic(G,W){const WG=L.mul(W,G);for(let i=0;i<Hn;i++)for(let j=0;j<Hn;j++)for(let k=0;k<n;k++)H[i][j]+=2*G[k][i]*WG[k][j];}
 for(let k=0;k<Hn;k++){
  const G=Xmap[k],gu=Array.from({length:Hn},(_,j)=>(j===k?1:0)-dot(K,G.map(r=>r[j])));Umap.push(gu);
  addQuadratic(G,Q);for(let i=0;i<Hn;i++)for(let j=0;j<Hn;j++)H[i][j]+=2*R*gu[i]*gu[j];
  const next=L.mul(F,G);for(let i=0;i<n;i++)next[i][k]+=B[i][0];Xmap.push(next);
 }
 addQuadratic(Xmap[Hn],P);
 // Symmetrize round-off only; no regularization, cost retuning or ignored Riccati defect.
 for(let i=0;i<Hn;i++)for(let j=0;j<i;j++){const v=.5*(H[i][j]+H[j][i]);H[i][j]=H[j][i]=v;}
 // Upstream quadprog rejects a non-positive-definite Hessian; no private math helper is copied.
 function assemble(world,goal,checkWorldRail=true){
  if(!Array.isArray(world)||world.length!==n||!world.every(Number.isFinite)||!Number.isFinite(goal))throw Error('Invalid MPC state or goal');
  if(checkWorldRail&&Math.abs(world[0])>p.railLimit+1e-10)throw Error('QP rejected: initial state outside physical rail');
  const e0=world.slice();e0[0]-=goal;const offsets=[e0],uoff=[],f=Array(Hn).fill(0),columns=[],lower=[];
  let constant=0;
  for(let k=0;k<=Hn;k++){
   const z=offsets[k],W=k===Hn?P:Q,wz=mv(W,z),G=Xmap[k];constant+=dot(z,wz);
   for(let j=0;j<Hn;j++)for(let i=0;i<n;i++)f[j]+=2*G[i][j]*wz[i];
   if(k<Hn){const u=-dot(K,z);uoff.push(u);constant+=R*u*u;for(let j=0;j<Hn;j++)f[j]+=2*R*Umap[k][j]*u;offsets.push(mv(F,z));}
  }
  // Bounds remain on ACTUAL u, not on the correction v. World rail stays absolute.
  for(const sign of [1,-1])for(let k=0;k<Hn;k++){columns.push(Umap[k].map(v=>sign*v));lower.push(-p.forceLimit-sign*uoff[k]);}
  for(let k=1;k<=Hn;k++){
   columns.push(Xmap[k][0].slice());lower.push(-p.railLimit-offsets[k][0]-goal);
   columns.push(Xmap[k][0].map(v=>-v));lower.push(offsets[k][0]+goal-p.railLimit);
  }
  return {e0,offsets,uoff,f,columns,lower,constant};
 }
 function trajectory(q,V){return {E:q.offsets.map((z,k)=>z.map((v,i)=>v+dot(Xmap[k][i],V))),U:q.uoff.map((v,k)=>v+dot(Umap[k],V))};}
 function evaluateCoordinates(e0,V){
  const q=assemble(e0,0,false),{E,U}=trajectory(q,V);let J=0;for(let k=0;k<Hn;k++)J+=dot(E[k],mv(Q,E[k]))+R*U[k]**2;J+=dot(E[Hn],mv(P,E[Hn]));
  const transformed=.5*dot(V,mv(H,V))+dot(q.f,V)+q.constant;
  const reconstructed=E.slice(0,-1).map((e,k)=>U[k]+dot(K,e));
  return {originalCost:J,transformedCost:transformed,costIdentityRelativeError:Math.abs(J-transformed)/Math.max(1,Math.abs(J)),transformError:norm(V.map((v,i)=>v-reconstructed[i]))};
 }
 function solve(world,goal=0){
  const q=assemble(world,goal),D=[[],...H.map(r=>[0,...r])],d=[0,...q.f.map(v=>-v)],C=[[],...Array.from({length:Hn},(_,i)=>[0,...q.columns.map(r=>r[i])])];
  const result=qp.solveQP(D,d,C,[0,...q.lower],0);if(result.message)throw Error('QP rejected: '+result.message);
  const V=result.solution.slice(1),lambda=result.Lagrangian.slice(1);
  if(V.length!==Hn||!V.every(Number.isFinite)||!lambda.every(Number.isFinite))throw Error('QP returned invalid pre-stabilized result');
  const slack=q.columns.map((r,i)=>dot(r,V)-q.lower[i]);
  const gradient=mv(H,V).map((v,i)=>v+q.f[i]);
  const kkt=norm(gradient.map((v,i)=>v-q.columns.reduce((s,r,j)=>s+r[i]*lambda[j],0)));
  const comp=norm(slack.map((v,j)=>v*lambda[j])),dual=Math.max(0,...lambda.map(v=>-v));
  const kktResidual=Math.max(kkt,comp,dual),primalResidual=Math.max(0,...slack.map(v=>-v));
  if(kktResidual>1e-7||primalResidual>1e-8)throw Error('QP acceptance residual exceeded in stabilized coordinates');
  const {E,U}=trajectory(q,V),X=E.map(e=>{const x=e.slice();x[0]+=goal;return x;});
  const physicalViolation=Math.max(0,...U.map(v=>Math.abs(v)-p.forceLimit),...X.map(x=>Math.abs(x[0])-p.railLimit));
  const dynamicsDefect=Math.max(...E.slice(0,-1).map((e,k)=>norm(mv(A,e).map((v,i)=>v+B[i][0]*U[k]-E[k+1][i]))));
  const check=evaluateCoordinates(q.e0,V);
  if(physicalViolation>1e-8||dynamicsDefect>1e-8||check.costIdentityRelativeError>1e-7)throw Error('QP physical reconstruction gate failed');
  // Recompute adjoint stationarity in original state/input units, independently of condensed H.
  let adj=mv(P,E[Hn]).map((v,i)=>2*v-(i===0?lambda[2*Hn+2*(Hn-1)]-lambda[2*Hn+2*(Hn-1)+1]:0));
  let originalStationarity=0,originalStationarityScale=1;
  for(let k=Hn-1;k>=0;k--){
   const bt=dot(B.map(r=>r[0]),adj),constraint=lambda[k]-lambda[Hn+k],ru=2*R*U[k];
   originalStationarity=Math.max(originalStationarity,Math.abs(ru+bt-constraint));originalStationarityScale=Math.max(originalStationarityScale,Math.abs(ru),Math.abs(bt),Math.abs(constraint));
   const next=mv(L.T(A),adj),qx=mv(Q,E[k]);adj=next.map((v,i)=>v+2*qx[i]-(i===0&&k>0?lambda[2*Hn+2*(k-1)]-lambda[2*Hn+2*(k-1)+1]:0));
  }
  const originalNormalizedStationarity=originalStationarity/originalStationarityScale;
  if(originalNormalizedStationarity>1e-7)throw Error('QP original-unit stationarity gate failed');
  return {accepted:true,U,X,V,J:check.originalCost,kktResidual,primalResidual,physicalViolation,dynamicsDefect,originalStationarity,originalNormalizedStationarity,
   iterations:result.iterations?.[1]??0,solver:'quadprog + exact LQR-coordinate transcription',scope:'same nominal finite-horizon cost and physical limits; not nonlinear or robust MPC',costIdentityRelativeError:check.costIdentityRelativeError};
 }
 return {solve,evaluateCoordinates,exportProblem(world,goal=0){const q=assemble(world,goal);return {H,f:q.f,columns:q.columns,lower:q.lower,constant:q.constant,F,K,Umap,Xmap};}};
}
