// N-dimensional EKF using the existing MuJoCo map/Jacobians and existing matrix solver.
// Local relative-angle chart; all joint encoders. Not a contact InEKF or calibrated hardware filter.
const L=typeof window==='undefined'?(await import('node:module')).createRequire(import.meta.url)('./engine.js'):ControlLab;
const clone=A=>A.map(r=>r.slice()),wrap=x=>Math.atan2(Math.sin(x),Math.cos(x));
export class ChainEKF{
 constructor(backend,profile,measurement){
  this.b=backend;this.p=profile;this.n=profile.nx;this.dof=profile.dof;
  if(backend.nx!==this.n||backend.assetSha256!==profile.assetSha256)throw Error('Observer model identity mismatch');
  if(!Array.isArray(measurement)||measurement.length!==this.dof||!measurement.every(Number.isFinite))throw Error('Invalid initial measurement');
  this.x=[...measurement,...Array(this.dof).fill(0)];this.P=clone(profile.kalmanSteadyPosterior);this.last={measurementUsed:false,nis:null,S:null,innovation:null};
 }
 snapshot(){return {x:this.x.slice(),P:clone(this.P),last:{...this.last},scope:'local nonlinear EKF with fixed simulation Qe/Re; output covariance is not certified calibration'};}
 step(y,u){
  if(!Number.isFinite(u))throw Error('Invalid observer control input');
  if(y!==null&&(!Array.isArray(y)||y.length!==this.dof||!y.every(Number.isFinite)))throw Error('Invalid measurement');
  const F=this.b.linearize(this.x,u).A,xm=this.b.step(this.x,u),Pm=L.add(L.mul(L.mul(F,this.P),L.T(F)),this.p.Qe);
  let x=xm,P=Pm,last={measurementUsed:false,nis:null,S:null,innovation:null};
  if(y!==null){
   const H=this.p.H,R=this.p.Re,PHt=L.mul(Pm,L.T(H)),S=L.add(L.mul(H,PHt),R);
   const K=PHt.map(row=>L.solveLinearSystem(S,row)),innovation=y.map((v,i)=>i?wrap(v-xm[i]):v-xm[i]);
   const correction=L.mv(K,innovation);x=xm.map((v,i)=>i>0&&i<this.dof?wrap(v+correction[i]):v+correction[i]);
   const Iminus=L.sub(L.eye(this.n),L.mul(K,H));P=L.add(L.mul(L.mul(Iminus,Pm),L.T(Iminus)),L.mul(L.mul(K,R),L.T(K)));
   const score=L.solveLinearSystem(S,innovation);const nis=innovation.reduce((s,v,i)=>s+v*score[i],0);
   last={measurementUsed:true,innovation,S,nis};
  }
  P=L.scale(L.add(P,L.T(P)),.5);
  if(!x.every(Number.isFinite)||!P.flat().every(Number.isFinite)||P.some((row,i)=>row[i]<0))throw Error('Invalid EKF posterior');
  this.x=x;this.P=P;this.last=last;return this.snapshot();
 }
}
