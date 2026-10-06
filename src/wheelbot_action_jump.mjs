// A jump on the active compact model. No plant reset or alternate asset.
import {CONTROLLED_INDICES as ix} from './wheelbot_backend.mjs';
const vector=(x,n)=>Array.isArray(x)&&x.length===n&&x.every(Number.isFinite);
const matrix=(a,n,m)=>Array.isArray(a)&&a.length===n&&a.every(r=>vector(r,m));
const mv=(a,x)=>a.map(r=>r.reduce((s,v,j)=>s+v*x[j],0));
const clip=(v,limit)=>Math.max(-limit,Math.min(limit,v));
export function validateActionJump(backend,base,p){
 if(!p||p.schema!=='wheelbot-action-jump/v1'||p.assetSha256!==backend.assetSha256||p.assetSha256!==base.assetSha256||p.steps!==600||p.controlDt!==.01||p.physicsDt!==.002||JSON.stringify(p.controlledIndices)!==JSON.stringify(ix)||JSON.stringify(p.measurementIndices)!=='[0,1,2,3,4,11]')throw Error('Jump profile does not match this robot');
 if(JSON.stringify(p.limitsNm)!==JSON.stringify(backend.limits)||JSON.stringify(p.externalForce)!=='[0,0,0]'||JSON.stringify(p.measurementSigma)!==JSON.stringify(base.measurementSigma)||!p.referenceMetrics?.passed)throw Error('Jump physics or measurement contract rejected');
 if(!matrix(p.ref,601,12)||!matrix(p.u,600,3)||!['A','B','K','L'].every(k=>Array.isArray(p[k])&&p[k].length===600))throw Error('Malformed jump trajectory');
 for(let k=0;k<600;k++)if(!matrix(p.A[k],11,11)||!matrix(p.B[k],11,3)||!matrix(p.K[k],3,11)||!matrix(p.L[k],11,6)||p.u[k].some((v,j)=>Math.abs(v)>backend.limits[j]+1e-10))throw Error('Invalid jump gains or torque');
 return true;
}
export function admitOptionalJump(backend,base,profile){
 try{validateActionJump(backend,base,profile);return {profile,reason:null};}
 catch(error){return {profile:null,reason:String(error.message??error)};}
}
function references(p,b,k,origin){
 if(!Number.isInteger(k)||k<0||k>=p.steps||!Number.isFinite(origin))throw Error('Invalid jump phase');
 const terminal=k>=200;
 const state=terminal?[...b.qref.slice(0,5),0,0,0,0,0,0]:ix.map(j=>p.ref[k][j]);state[0]+=origin;
 const next=terminal?state.slice():ix.map(j=>p.ref[k+1][j]);if(!terminal)next[0]+=origin;
 return {state,next,A:terminal?b.A:p.A[k],B:terminal?b.B:p.B[k],K:terminal?b.K:p.K[k],L:terminal?b.L:p.L[k],u:terminal?b.uref:p.u[k]};
}
export function actionJumpCommand(p,b,k,estimate,origin){
 if(!vector(estimate,11))throw Error('Invalid jump estimate');
 const r=references(p,b,k,origin),feedback=mv(r.K,estimate.map((v,j)=>v-r.state[j]));const requested=r.u.map((v,j)=>v-feedback[j]);
 return {requested,u:requested.map((v,j)=>clip(v,b.limitsNm[j])),saturated:requested.some((v,j)=>Math.abs(v)>b.limitsNm[j])};
}
export function actionJumpObserver(p,b,k,estimate,u,y,origin){
 if(!vector(estimate,11)||!vector(u,3)||!vector(y,6))throw Error('Invalid jump observer input');
 const r=references(p,b,k,origin),ad=mv(r.A,estimate.map((v,j)=>v-r.state[j])),bu=mv(r.B,u.map((v,j)=>v-r.u[j]));
 const pred=r.next.map((v,j)=>v+ad[j]+bu[j]),innovation=y.map((v,j)=>v-pred[j===5?10:j]);
 const correction=mv(r.L,innovation);return pred.map((v,j)=>v+correction[j]);
}
export function jumpEntry(estimate,base){
 return vector(estimate,11)&&Math.abs(estimate[1]-base.qref[1])<=.008&&Math.abs(estimate[2]-base.qref[2])<=.025&&Math.abs(estimate[3]-base.qref[3])<=.04&&Math.abs(estimate[4]-base.qref[4])<=.04&&estimate.slice(5).every(v=>Math.abs(v)<.15);
}
