// Dimension-aware control/observation adapter. Reuses the existing upstream QP
// transcription; Riccati and stationary Kalman gains come from verified SciPy profiles.
const L=typeof window==='undefined'?(await import('node:module')).createRequire(import.meta.url)('./engine.js'):ControlLab;
const wrap=x=>Math.atan2(Math.sin(x),Math.cos(x));
export function absoluteAngles(x,poles){let a=0;return Array.from({length:poles},(_,i)=>{a+=x[i+1];return wrap(a);});}
export function relativeAngles(angles){return angles.map((v,i)=>i?v-angles[i-1]:v);}
export function createChainTrial(backend,profile,{controller='mpc',observer='steady_kf',seed=4103,goal=.1,initialState,noise=2e-4}={}){
 if(!['mpc','lqr'].includes(controller)||!['oracle','steady_kf'].includes(observer))throw Error('Unsupported N-link controller/observer; legacy policies are four-state only');
 if(backend.assetSha256!==profile.assetSha256||backend.nx!==profile.nx)throw Error('Model/profile identity mismatch');
 if(!profile.designAvailable)throw Error('Design rejected: '+(profile.reason||'unverified Riccati result'));
 const n=profile.nx,dof=profile.dof;let x=initialState?.slice()??Array(n).fill(0),xh,steps=0,last=null;
 if(x.length!==n||!x.every(Number.isFinite)||!Number.isFinite(goal)||!Number.isFinite(noise)||noise<0)throw Error('Invalid trial state/goal/noise');
 let rng=seed>>>0;const uniform=()=>{rng=(Math.imul(1664525,rng)+1013904223)>>>0;return(rng+.5)/4294967296;};
 const gaussian=()=>Math.sqrt(-2*Math.log(uniform()))*Math.cos(2*Math.PI*uniform());
 const measure=()=>x.slice(0,dof).map((v,i)=>i?wrap(v+noise*gaussian()):v+noise*gaussian());
 const y0=measure();xh=observer==='oracle'?x.slice():[...y0,...Array(dof).fill(0)];
 const input=estimate=>{
  const e=estimate.slice();e[0]-=goal;
  if(controller==='lqr'){const u=-L.mv(profile.K,e)[0];return {u:Math.max(-profile.forceLimit,Math.min(profile.forceLimit,u)),solver:'SciPy-DARE gain + force saturation',kktResidual:null,primalResidual:null};}
  const solution=L.solveBoxLinearMpc(profile.A,profile.B,profile.Q,profile.R,profile.P,e,Array(profile.horizon).fill(0),profile.forceLimit,30,{positionLimit:profile.railLimit,goal});
  return {u:solution.U[0],solver:solution.solver,kktResidual:solution.kktResidual,primalResidual:solution.primalResidual};
 };
 return {
  snapshot:()=>({truth:x.slice(),estimate:xh.slice(),goal,steps,last,absoluteAngles:absoluteAngles(x,profile.poles),controller,observer,information:observer==='oracle'?'simulator truth supplied to controller':'cart encoder + every relative joint encoder; velocities unmeasured',uncertainty:'stationary local linear KF design, not online covariance calibration'}),
  step(external=0){
   if(!Number.isFinite(external))throw Error('Invalid external force');
   const t=performance.now();const command=input(xh);
   if(!Number.isFinite(command.u)||Math.abs(command.u)>profile.forceLimit+1e-7)throw Error('Invalid control rejected before plant');
   const solveMs=performance.now()-t;x=backend.step(x,command.u,external);const y=measure();
   if(observer==='oracle')xh=x.slice();else{
    const pred=L.mv(profile.A,xh).map((v,i)=>v+profile.B[i][0]*command.u),innovation=y.map((v,i)=>i?wrap(v-pred[i]):v-pred[i]);
    xh=pred.map((v,i)=>v+L.mv(profile.kalmanGain,innovation)[i]);for(let i=1;i<dof;i++)xh[i]=wrap(xh[i]);
   }
   if(!xh.every(Number.isFinite))throw Error('Invalid estimator state');steps++;
   const angles=absoluteAngles(x,profile.poles);last={...command,solveMs,computeMs:performance.now()-t,external,failed:Math.abs(x[0])>profile.railLimit||angles.some(v=>Math.abs(v)>35*Math.PI/180)};
   return this.snapshot();
  }
 };
}
