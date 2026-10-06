const ix=[0,1,2,3,4,6,7,8,9,10,11];
const mv=(a,x)=>a.map(r=>r.reduce((s,v,j)=>s+v*x[j],0));
const vector=(x,n)=>Array.isArray(x)&&x.length===n&&x.every(Number.isFinite);
const matrix=(a,n,m)=>Array.isArray(a)&&a.length===n&&a.every(r=>vector(r,m));
const clip=(v,l)=>Math.max(-l,Math.min(l,v));

export function createStationaryJump(backend,base,bundle){
 bundle=structuredClone(bundle);
 const asset=backend.robotAssetSha256??backend.assetSha256,runtime=backend.diagnostics?.().version;
 if(!bundle||bundle.schema!=='wheelbot-stationary-jump/v2'||bundle.assetSha256!==asset||bundle.nativeVersion!==runtime)throw Error('Stationary jump identity mismatch');
 if(bundle.steps!==600||bundle.scheduledSteps!==140||bundle.controlDt!==.01||bundle.physicsDt!==.002||JSON.stringify(bundle.controlledIndices)!==JSON.stringify(ix)||JSON.stringify(bundle.measurementIndices)!==JSON.stringify(base.measurementIndices)||JSON.stringify(bundle.measurementSigma)!==JSON.stringify(base.measurementSigma)||JSON.stringify(bundle.limitsNm)!==JSON.stringify(backend.limits))throw Error('Stationary jump contract mismatch');
 if(!Array.isArray(bundle.profiles)||bundle.profiles.length!==3)throw Error('Stationary jump requires low/medium/high schedules');
 for(const p of bundle.profiles){
  if(!vector(p.entry,12)||!matrix(p.ref,141,12)||!matrix(p.u,140,3)||!p.referenceMetrics?.passed||!Number.isFinite(p.referenceMetrics.wheelClearanceM))throw Error('Malformed stationary jump profile');
  for(const [key,n,m]of [['A',11,11],['B',11,3],['K',3,11],['L',11,6]])if(!Array.isArray(p[key])||p[key].length!==140||!p[key].every(v=>matrix(v,n,m))||!matrix(bundle.terminal?.[key],n,m))throw Error('Malformed stationary jump gains');
  if(p.u.some(u=>u.some((v,j)=>Math.abs(v)>backend.limits[j]+1e-10)))throw Error('Stationary jump torque limit mismatch');
 }
 if(!vector(bundle.terminal?.qref,6)||!vector(bundle.terminal?.uref,3))throw Error('Invalid stationary jump terminal');
 const levels=bundle.profiles.map(p=>p.referenceMetrics.wheelClearanceM);
 return {levels,start(estimate,fraction,support={height:0,slope:0}){
  if(!vector(estimate,11)||!Number.isFinite(fraction)||fraction<0||fraction>1||!Number.isFinite(support.height)||!Number.isFinite(support.slope))throw Error('Invalid stationary jump request');
  if(Math.abs(support.slope)>.02)throw Error('Stationary jump requires locally level support');
  const tol=bundle.entryTolerance,p=bundle.profiles[Math.min(bundle.profiles.length-1,Math.round(fraction*(bundle.profiles.length-1)))];
  if(Math.abs(estimate[1]-support.height-p.entry[1])>tol.height||Math.abs(estimate[2]-p.entry[2])>tol.pitch||[3,4].some(j=>Math.abs(estimate[j]-p.entry[j])>tol.joints)||estimate.slice(5).some(v=>Math.abs(v)>tol.rates))throw Error('Stationary jump entry outside calibrated pose/rate envelope');
  const origin=estimate[0],height=support.height,acceptedHeight=p.referenceMetrics.wheelClearanceM;
  function reference(k){
   if(!Number.isInteger(k)||k<0||k>=bundle.steps)throw Error('Invalid stationary jump sample');
   const terminal=k>=bundle.scheduledSteps,src=terminal?bundle.terminal:p;
   const state=terminal?ix.map(j=>j<6?src.qref[j]:0):ix.map(j=>p.ref[k][j]);
   state[0]+=origin;state[1]+=height;
   const next=terminal?state.slice():ix.map(j=>p.ref[k+1][j]);
   if(!terminal){next[0]+=origin;next[1]+=height;}
   return {state,next,u:terminal?src.uref:p.u[k],A:terminal?src.A:p.A[k],B:terminal?src.B:p.B[k],K:terminal?src.K:p.K[k],L:terminal?src.L:p.L[k]};
  }
  return {profileId:'stationary-'+p.id,stationary:true,vx:0,origin,height,acceptedHeight,reference,command(k,estimate){const r=reference(k),f=mv(r.K,estimate.map((v,j)=>v-r.state[j]));return {...r,u:r.u.map((v,j)=>clip(v-f[j],backend.limits[j]))};},observe(k,estimate,u,y){const r=reference(k),a=mv(r.A,estimate.map((v,j)=>v-r.state[j])),b=mv(r.B,u.map((v,j)=>v-r.u[j])),pred=r.next.map((v,j)=>v+a[j]+b[j]),innovation=y.map((v,j)=>v-pred[j===5?10:j]),c=mv(r.L,innovation);return pred.map((v,j)=>v+c[j]);}};
 }};
}
