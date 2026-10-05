import {createWheelbotMPC} from './wheelbot_mpc.mjs';
import {CONTROLLED_INDICES} from './wheelbot_backend.mjs';
// Same matrix-vector/KF equations as the CartPole teaching code, with an explicit
// 11-dimensional local state. The original four-state observer is not reused.
const mv=(A,x)=>A.map(r=>r.reduce((s,v,j)=>s+v*x[j],0));
const valid=(x,n)=>Array.isArray(x)&&x.length===n&&x.every(Number.isFinite);
const matrix=(a,n,m)=>Array.isArray(a)&&a.length===n&&a.every(r=>valid(r,m));
export function validateWheelbotProfile(b,p){
 if(!b||!p||typeof p!=='object')throw Error('Invalid profile/backend');
 if(p.assetSha256!==b.assetSha256)throw Error('Asset/profile hash mismatch');
 if(![p.trimQaccInf,p.dareNormalizedResidual,p.closedLoopRadius,p.observerErrorRadius].every(Number.isFinite)||p.trimQaccInf>1e-7||p.dareNormalizedResidual>1e-8||p.closedLoopRadius>=1||p.observerErrorRadius>=1)throw Error('Profile numerical design gates rejected');
 if(!p.designAvailable)throw Error('Design rejected: '+(p.reason??'unverified'));
 const measured=p.measurementIndices??[0,1,2,3,4];
 if(!['0,1,2,3,4','0,1,2,3,4,11'].includes(measured.join(',')))throw Error('Unsupported measurement mapping');
 if(!valid(p.qref,6)||!valid(p.uref,3)||!matrix(p.A,11,11)||!matrix(p.B,11,3)||!matrix(p.K,3,11)||!matrix(p.L,11,measured.length)||!valid(p.measurementSigma,measured.length))throw Error('Invalid profile dimensions/values');
 if(JSON.stringify(p.controlledIndices)!==JSON.stringify(CONTROLLED_INDICES)||p.controlDt!==.01)throw Error('Unsupported coordinate/time mapping');
 if(p.uref.some((u,i)=>Math.abs(u)>b.limits[i])||p.measurementSigma.some(v=>v<=0))throw Error('Invalid reference/noise limits');
 return true;
}
export function observerUpdate(p,estimate,u,y){
 const measured=p.measurementIndices??[0,1,2,3,4];
 if(!valid(estimate,11)||!valid(u,3)||!valid(y,measured.length))throw Error('Invalid observer input');
 const ref=CONTROLLED_INDICES.map(i=>i<6?p.qref[i]:0);
 const dx=estimate.map((v,i)=>v-ref[i]),ad=mv(p.A,dx),bu=mv(p.B,u.map((v,i)=>v-p.uref[i]));
 const prediction=ref.map((v,i)=>v+ad[i]+bu[i]);
 const correction=mv(p.L,y.map((v,i)=>v-prediction[CONTROLLED_INDICES.indexOf(measured[i])]));
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
 if(!['lqr_kf','mpc_kf','passive'].includes(mode))throw Error('NOT_YET_SUPPORTED: '+mode);
 if(!valid(initialState,12)||!Number.isFinite(goal))throw Error('Invalid trial initial state/goal');
 const mpc=mode==='mpc_kf'?createWheelbotMPC(p,backend.limits):null;
 let rng=seed>>>0;
 const uniform=()=>{rng=(Math.imul(1664525,rng)+1013904223)>>>0;return (rng+.5)/4294967296;};
 const noise=()=>Math.sqrt(-2*Math.log(uniform()))*Math.cos(2*Math.PI*uniform());
 const measured=p.measurementIndices??[0,1,2,3,4];
 const measure=x=>measured.map((index,i)=>x[index]+p.measurementSigma[i]*noise());
 let truth=initialState.slice(),estimate=[...measure(truth).slice(0,5),0,0,0,0,0,0],steps=0,last=null,failed=false;
 const snapshot=()=>({truth:truth.slice(),estimate:estimate.slice(),steps,goal,mode,last,failed,estimateIndices:CONTROLLED_INDICES});
 return {snapshot,setGoal(value){if(!Number.isFinite(value)||Math.abs(value)>.1)throw Error('Live goal outside ±0.1m');goal=value;return snapshot();},step(externalX=0){
  if(failed)throw Error('Trial already failed');
  const action=mode==='passive'?{u:[0,0,0],requested:[0,0,0],saturated:false}:mpc?mpc.solve(estimate,goal):lqrTorque(p,estimate,goal,backend.limits);
  if(!valid(action.u,3)||action.u.some((v,i)=>Math.abs(v)>backend.limits[i]+1e-8))throw Error('Invalid controller command');
  truth=backend.step(truth,action.u,externalX);const measurement=measure(truth);
  estimate=observerUpdate(p,estimate,action.u,measurement);steps++;
  failed=!truth.every(Number.isFinite)||Math.abs(truth[0]-p.qref[0])>1||Math.abs(truth[2]-p.qref[2])>.6||truth[1]<.12;
  last={...action,measurement,contact:backend.contact(truth),externalX};return snapshot();
 }};
}

// Check the optional one-variable design against the exact baseline bytes loaded
// by the page. This is stale/mismatched-data detection, not a trust signature.
export function validateWheelbotResponseProfile(backend,baseline,response,baselineSha256){
 if(typeof baselineSha256!=='string'||!/^[0-9a-f]{64}$/.test(baselineSha256))throw Error('Invalid baseline hash');
 validateWheelbotProfile(backend,baseline);validateWheelbotProfile(backend,response);
 const d=response.responseDesign;
 if(!d||d.sourceProfileSha256!==baselineSha256||d.changedPreference!=='Qc[0,0] only'||!Number.isFinite(d.factor)||d.factor<=0)throw Error('Response design does not match the loaded baseline');
 if(!matrix(response.Q,11,11)||!matrix(response.P,11,11)||!matrix(baseline.Q,11,11))throw Error('Invalid response cost dimensions');
 const allowed=new Set(['Q','P','K','dareNormalizedResidual','closedLoopRadius','responseDesign']);
 for(const key of new Set([...Object.keys(baseline),...Object.keys(response)])){
  if(!allowed.has(key)&&JSON.stringify(response[key])!==JSON.stringify(baseline[key]))throw Error('Response changed baseline field: '+key);
 }
 for(let i=0;i<11;i++)for(let j=0;j<11;j++){
  const expected=baseline.Q[i][j]*(i===0&&j===0?d.factor:1);
  if(!Number.isFinite(expected)||Math.abs(response.Q[i][j]-expected)>1e-12*Math.max(1,Math.abs(expected)))throw Error('Response cost is not the declared Qx-only change');
 }
 return true;
}
