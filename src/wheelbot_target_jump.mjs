// Pure scheduled feedback planner. The caller owns the plant and six-channel KF.
const vector=(a,n)=>Array.isArray(a)&&a.length===n&&a.every(Number.isFinite);
const matrix=(a,n,m)=>Array.isArray(a)&&a.length===n&&a.every(r=>vector(r,m));
const mv=(a,x)=>a.map(r=>r.reduce((s,v,j)=>s+v*x[j],0));
const ix=[0,1,2,3,4,6,7,8,9,10,11];
export function createTargetJump(backend,base,bundle,{diagnostic=false}={}){
 bundle=structuredClone(bundle);
 if(!bundle||bundle.schema!=='wheelbot-target-jump/v1'||bundle.assetSha256!==backend.assetSha256||bundle.assetSha256!==base.assetSha256||bundle.controlDt!==.01||bundle.physicsDt!==.002||JSON.stringify(bundle.controlledIndices)!==JSON.stringify(ix)||bundle.steps!==600||bundle.scheduledSteps!==140||JSON.stringify(bundle.externalForce)!=='[0,0,0]'||JSON.stringify(bundle.limitsNm)!==JSON.stringify(backend.limits)||JSON.stringify(bundle.measurementIndices)!=='[0,1,2,3,4,11]'||JSON.stringify(bundle.measurementSigma)!==JSON.stringify(base.measurementSigma)||!Array.isArray(bundle.profiles)||!bundle.profiles.length)throw Error('Target jump profile/model mismatch');
 if(bundle.nativeVersion!==backend.diagnostics().version)throw Error('Target jump generation/runtime version mismatch');
 if(!['height','pitch','joints','rates'].every(k=>Number.isFinite(bundle.entryTolerance?.[k])&&bundle.entryTolerance[k]>0))throw Error('Invalid target jump entry tolerance');
 if(!diagnostic&&!bundle.releaseValidated)throw Error('Target jump family has not passed release validation');
 if(!diagnostic&&!(Number.isFinite(bundle.protocol?.maxLandingTangentialSpeed)&&bundle.protocol.maxLandingTangentialSpeed>0))throw Error('Jump landing speed bound has not been declared and validated');
 for(const p of bundle.profiles){
  if(!vector(p.parameters,6)||!vector(p.entry,12)||!matrix(p.ref,141,12)||!matrix(p.u,140,3)||!p.referenceMetrics?.passed||!vector(p.referenceMetrics.apex,2)||!vector(p.referenceMetrics.landing,2)||!Array.isArray(p.referenceMetrics.wheelPath)||!p.referenceMetrics.wheelPath.every(v=>vector(v,2)))throw Error('Invalid target jump reference');
  for(const [key,n,m]of [['A',11,11],['B',11,3],['K',3,11],['L',11,6]])if(!Array.isArray(p[key])||p[key].length!==140||!p[key].every(v=>matrix(v,n,m))||!matrix(p.terminal?.[key],n,m))throw Error('Invalid target jump schedule');
  if(!vector(p.terminal.qref,6)||!vector(p.terminal.uref,3)||p.u.some(u=>u.some((v,j)=>Math.abs(v)>backend.limits[j])))throw Error('Invalid target jump torque');
 }
 const freeze=v=>{if(v&&typeof v==='object'){Object.values(v).forEach(freeze);Object.freeze(v);}};freeze(bundle);
 function start(estimate,target){
  if(!vector(estimate,11)||!target||!Number.isFinite(target.x)||!Number.isFinite(target.z)||target.mode&&target.mode!=='wheel')throw Error('Expected estimated state and wheel-centre apex target');
  const tol=bundle.entryTolerance;
  const eligible=bundle.profiles.filter(p=>Math.abs(estimate[1]-p.entry[1])<=tol.height&&Math.abs(estimate[2]-p.entry[2])<=tol.pitch&&[3,4].every(j=>Math.abs(estimate[j]-p.entry[j])<=tol.joints)&&estimate.slice(5).every(v=>Math.abs(v)<=tol.rates));
  if(!eligible.length){const near=bundle.profiles.reduce((a,b)=>Math.abs(estimate[1]-a.entry[1])<Math.abs(estimate[1]-b.entry[1])?a:b);const missing=[];if(Math.abs(estimate[1]-near.entry[1])>tol.height)missing.push('높이 ±'+tol.height+' m');if(Math.abs(estimate[2]-near.entry[2])>tol.pitch)missing.push('기울기 ±'+tol.pitch+' rad');if([3,4].some(j=>Math.abs(estimate[j]-near.entry[j])>tol.joints))missing.push('관절 ±'+tol.joints+' rad');if(estimate.slice(5).some(v=>Math.abs(v)>tol.rates))missing.push('속도 ≤'+tol.rates+' (현재 '+Math.max(...estimate.slice(5).map(Math.abs)).toFixed(3)+')');throw Error('Current state outside checked jump entry domain: '+missing.join(', '));}
  const origin=estimate[0];const distance=p=>Math.hypot(origin+p.referenceMetrics.apex[0]-target.x,p.referenceMetrics.apex[1]-target.z);
  const p=eligible.reduce((a,b)=>distance(a)<=distance(b)?a:b);
  const translate=v=>[v[0]+origin,v[1]];
  const accepted={apex:translate(p.referenceMetrics.apex),landing:translate(p.referenceMetrics.landing),wheelPath:p.referenceMetrics.wheelPath.map(translate),basePath:p.ref.filter((_,k)=>k%5===0).map(x=>[x[0]+origin,x[1]]),projection:'nearest sampled eligible apex; not globally closest'};
  function reference(k){
   if(!Number.isInteger(k)||k<0||k>=bundle.steps)throw Error('Invalid target jump sample');
   const terminal=k>=(p.terminalStep??140),source=terminal?p.terminal:p;
   const state=terminal?ix.map(j=>j<6?source.qref[j]:0):ix.map(j=>p.ref[k][j]);state[0]+=origin;
   const next=terminal?state.slice():ix.map(j=>p.ref[k+1][j]);if(!terminal)next[0]+=origin;
   return {state,next,u:terminal?source.uref:source.u[k],...Object.fromEntries(['A','B','K','L'].map(key=>[key,terminal?source[key]:source[key][k]])),phase:terminal?'recover':k<18?'crouch':k<Math.round((.18+p.parameters[1])*100)?'push':'flight/landing'};
  }
  function command(k,stateEstimate){
   if(!vector(stateEstimate,11))throw Error('Invalid target jump estimate');
   const r=reference(k),feedback=mv(r.K,stateEstimate.map((v,j)=>v-r.state[j]));
   return {...r,u:r.u.map((v,j)=>Math.max(-backend.limits[j],Math.min(backend.limits[j],v-feedback[j])))};
  }
  function observe(k,estimate,u,measurement){
   if(!vector(estimate,11)||!vector(u,3)||!vector(measurement,6))throw Error('Invalid target jump observer input');
   const r=reference(k),a=mv(r.A,estimate.map((v,j)=>v-r.state[j])),b=mv(r.B,u.map((v,j)=>v-r.u[j]));
   const pred=r.next.map((v,j)=>v+a[j]+b[j]),innovation=measurement.map((v,j)=>v-pred[j===5?10:j]);const correction=mv(r.L,innovation);return pred.map((v,j)=>v+correction[j]);
  }
  return {profileId:p.id,requested:{...target},accepted,entryEstimate:estimate.slice(),origin,steps:600,controlDt:.01,landingSpeedBound:bundle.protocol.maxLandingTangentialSpeed,reference,command,observe};
 }
 return {start};
}

export function admitOptionalTargetJump(backend,base,bundle,baselineSha256){
 try{if(!baselineSha256||bundle?.baselineSha256!==baselineSha256)throw Error('Target jump baseline hash mismatch');return {planner:createTargetJump(backend,base,bundle),reason:null};}
 catch(e){return {planner:null,reason:e.message};}
}
