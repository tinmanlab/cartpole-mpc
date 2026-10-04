import {CONTROLLED_INDICES} from './wheelbot_backend.mjs';
// Same matrix-vector/KF equations as the CartPole teaching code, with an explicit
// 11-dimensional local state. The original four-state observer is not reused.
const mv=(A,x)=>A.map(r=>r.reduce((s,v,j)=>s+v*x[j],0));
const valid=(x,n)=>Array.isArray(x)&&x.length===n&&x.every(Number.isFinite);
const matrix=(a,n,m)=>Array.isArray(a)&&a.length===n&&a.every(r=>valid(r,m));
export function validateWheelbotProfile(b,p){
 if(p.assetSha256!==b.assetSha256)throw Error('Asset/profile hash mismatch');
 if(![p.trimQaccInf,p.dareNormalizedResidual,p.closedLoopRadius,p.observerErrorRadius].every(Number.isFinite)||p.trimQaccInf>1e-7||p.dareNormalizedResidual>1e-8||p.closedLoopRadius>=1||p.observerErrorRadius>=1)throw Error('Profile numerical design gates rejected');
 if(!p.designAvailable)throw Error('Design rejected: '+(p.reason??'unverified'));
 if(!valid(p.qref,6)||!valid(p.uref,3)||!matrix(p.A,11,11)||!matrix(p.B,11,3)||!matrix(p.K,3,11)||!matrix(p.L,11,5)||!valid(p.measurementSigma,5))throw Error('Invalid profile dimensions/values');
 if(JSON.stringify(p.controlledIndices)!==JSON.stringify(CONTROLLED_INDICES)||p.controlDt!==.01)throw Error('Unsupported coordinate/time mapping');
 if(p.uref.some((u,i)=>Math.abs(u)>b.limits[i])||p.measurementSigma.some(v=>v<=0))throw Error('Invalid reference/noise limits');
 return true;
}
export function observerUpdate(p,estimate,u,y){
 if(!valid(estimate,11)||!valid(u,3)||!valid(y,5))throw Error('Invalid observer input');
 const ref=CONTROLLED_INDICES.map(i=>i<6?p.qref[i]:0);
 const dx=estimate.map((v,i)=>v-ref[i]),ad=mv(p.A,dx),bu=mv(p.B,u.map((v,i)=>v-p.uref[i]));
 const prediction=ref.map((v,i)=>v+ad[i]+bu[i]);
 const correction=mv(p.L,y.map((v,i)=>v-prediction[i]));
 return prediction.map((v,i)=>v+correction[i]);
}
export function lqrTorque(p,estimate,goal,limits){
 if(!valid(estimate,11)||!Number.isFinite(goal))throw Error('Invalid controller input');
 const ref=CONTROLLED_INDICES.map(i=>i<6?p.qref[i]:0);ref[0]+=goal;
 const feedback=mv(p.K,estimate.map((v,i)=>v-ref[i]));
 const requested=p.uref.map((v,i)=>v-feedback[i]);
 return {requested,u:requested.map((v,i)=>Math.max(-limits[i],Math.min(limits[i],v))),saturated:requested.some((v,i)=>Math.abs(v)>limits[i])};
}
export function createWheelbotTrial(backend,p,{seed=1,goal=0,initialState=[...p.qref,0,0,0,0,0,0],mode='lqr_kf'}={}){
 validateWheelbotProfile(backend,p);
 if(!['lqr_kf','passive'].includes(mode))throw Error('NOT_YET_SUPPORTED: '+mode);
 if(!valid(initialState,12)||!Number.isFinite(goal))throw Error('Invalid trial initial state/goal');
 let rng=seed>>>0;
 const uniform=()=>{rng=(Math.imul(1664525,rng)+1013904223)>>>0;return (rng+.5)/4294967296;};
 const noise=()=>Math.sqrt(-2*Math.log(uniform()))*Math.cos(2*Math.PI*uniform());
 const measure=x=>x.slice(0,5).map((v,i)=>v+p.measurementSigma[i]*noise());
 let truth=initialState.slice(),estimate=[...measure(truth),0,0,0,0,0,0],steps=0,last=null,failed=false;
 const snapshot=()=>({truth:truth.slice(),estimate:estimate.slice(),steps,goal,mode,last,failed,estimateIndices:CONTROLLED_INDICES});
 return {snapshot,step(externalX=0){
  if(failed)throw Error('Trial already failed');
  const action=mode==='passive'?{u:[0,0,0],requested:[0,0,0],saturated:false}:lqrTorque(p,estimate,goal,backend.limits);
  truth=backend.step(truth,action.u,externalX);const measurement=measure(truth);
  estimate=observerUpdate(p,estimate,action.u,measurement);steps++;
  failed=!truth.every(Number.isFinite)||Math.abs(truth[0]-p.qref[0])>1||Math.abs(truth[2]-p.qref[2])>.6||truth[1]<.12;
  last={...action,measurement,contact:backend.contact(truth),externalX};return snapshot();
 }};
}
