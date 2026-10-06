// Finite-horizon affine LQ tracking, using the pinned upstream QP factorization.
// This plans around supplied local models. Nonlinear preview still decides
// admissibility: Riccati algebra alone is not a contact/saturation guarantee.
import '../vendor/quadprog/quadprog.js';
const T=A=>A[0].map((_,j)=>A.map(r=>r[j]));
const mul=(A,B)=>A.map(r=>B[0].map((_,j)=>r.reduce((s,v,k)=>s+v*B[k][j],0)));
const mv=(A,x)=>A.map(r=>r.reduce((s,v,j)=>s+v*x[j],0));
const add=(A,B)=>A.map((r,i)=>r.map((v,j)=>v+B[i][j]));
const sub=(A,B)=>A.map((r,i)=>r.map((v,j)=>v-B[i][j]));
const sym=A=>A.map((r,i)=>r.map((v,j)=>(v+A[j][i])/2));
const finite=x=>Array.isArray(x)&&x.every(v=>Array.isArray(v)?finite(v):Number.isFinite(v));
function inverseSPD(H){
 const n=H.length,columns=[];
 for(let j=0;j<n;j++){
  const linear=Array(n).fill(0);linear[j]=1;
  const r=globalThis.Quadprog.solveQP([[],...H.map(row=>[0,...row])],[0,...linear],[[],...H.map(()=>[0])],[0],0);
  if(r.message||!finite(r.solution.slice(1)))throw Error('Affine tracking factorization rejected');
  columns.push(r.solution.slice(1));
 }
 return T(columns);
}
export function planAffineTracking(stages,terminalP,terminalReference){
 if(!Array.isArray(stages)||!stages.length||stages.length>1600||!finite(terminalP)||!finite(terminalReference))throw Error('Invalid affine tracking horizon');
 let P=terminalP.map(r=>r.slice()),linear=Array(P.length).fill(0);const out=Array(stages.length);
 for(let k=stages.length-1;k>=0;k--){
  const s=stages[k],next=k+1<stages.length?stages[k+1].reference:terminalReference;
  if(!s||!['A','B','Q','R','trim','uref','reference'].every(name=>finite(s[name]))||s.trim.length!==P.length||s.reference.length!==P.length)throw Error('Invalid affine stage');
  const {A,B,Q,R,trim,reference}=s,BT=T(B),AT=T(A),H=add(R,mul(mul(BT,P),B)),inverse=inverseSPD(sym(H));
  const K=mul(mul(mul(inverse,BT),P),A),F=sub(A,mul(B,K));
  const c=mv(A,reference.map((v,j)=>v-trim[j])).map((v,j)=>v+trim[j]-next[j]);
  const v=mv(P,c).map((value,j)=>value+linear[j]);
  const feedforward=mv(mul(inverse,BT),v).map(value=>-value);
  out[k]={K,feedforward,reference:reference.slice(),uref:s.uref.slice()};
  linear=mv(T(F),v);
  P=sym(add(Q,mul(mul(AT,P),F)));
  if(!finite(P)||!finite(linear)||!finite(feedforward))throw Error('Nonfinite affine tracking solution');
 }
 return out;
}
