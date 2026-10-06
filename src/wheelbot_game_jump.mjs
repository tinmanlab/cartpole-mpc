import {createTargetJump} from './wheelbot_target_jump.mjs';
const ix=[0,1,2,3,4,6,7,8,9,10,11];
const mv=(a,x)=>a.map(r=>r.reduce((s,v,j)=>s+v*x[j],0));
export function createGameJump(backend,base,bundle){
 bundle=structuredClone(bundle);
 // Reuse the complete existing schedule validation; canonical robot identity is
 // distinct from a trusted world's scene digest. Neither profile is relabeled.
 createTargetJump({...backend,assetSha256:backend.robotAssetSha256??backend.assetSha256},base,bundle);
 // Two source schedules are continuously interpolated. The monotone map below
 // was measured in the canonical 3.15.0 model; noisy intermediate rollouts are a separate gate.
 const levels=[1,5].map(id=>bundle.profiles.find(p=>p.id===id));
 if(levels.some(p=>!p))throw Error('Game jump requires calibrated zero-bias levels 1 and 5');
 const calibrated=[.021160458834776588,.02606231891066675,.03117390101639382,.03687487459987608,.045377086215782556];
 const blend=(a,b,f)=>Array.isArray(a)?a.map((v,j)=>blend(v,b[j],f)):a*(1-f)+b*f;
 return {levels:[calibrated[0],calibrated.at(-1)],start(estimate,fraction,support={height:0,slope:0}){
  if(!Array.isArray(estimate)||estimate.length!==11||!estimate.every(Number.isFinite)||(!Number.isFinite(fraction)||fraction<0||fraction>1)||!Number.isFinite(support.height)||!Number.isFinite(support.slope))throw Error('Invalid game jump state');
  if(Math.abs(support.slope)>.02)throw Error('Jump calibrated only on locally level support');
  const acceptedHeight=calibrated[0]+fraction*(calibrated.at(-1)-calibrated[0]);
  let interval=0;while(interval<3&&acceptedHeight>calibrated[interval+1])interval++;
  const mix=(interval+(acceptedHeight-calibrated[interval])/(calibrated[interval+1]-calibrated[interval]))/4;
  const p={...levels[0],id:mix===0?1:mix===1?5:'interpolated',terminal:levels[0].terminal};
  for(const key of ['ref','u','A','B','K','L'])p[key]=mix===0?levels[0][key]:mix===1?levels[1][key]:blend(levels[0][key],levels[1][key],mix);
  const origin=estimate[0],vx=estimate[5],height=support.height;
  if(Math.abs(vx)>.25||Math.abs(estimate[1]-height-p.entry[1])>.015||Math.abs(estimate[2])>.04||[3,4].some(j=>Math.abs(estimate[j]-p.entry[j])>.06)||estimate.slice(6,10).some(v=>Math.abs(v)>.3))throw Error('Jump entry outside calibrated pose/rate envelope');
  function reference(k){
   if(!Number.isInteger(k)||k<0||k>=600)throw Error('Invalid jump sample');
   const terminal=k>=140,s=terminal?p.terminal:p;
   const ref=n=>{const x=terminal?ix.map(j=>j<6?s.qref[j]:0):ix.map(j=>p.ref[n][j]);x[0]+=origin+vx*n*.01;x[1]+=height;x[5]+=vx;x[10]+=vx/.05;return x;};
   const u=(terminal?s.uref:s.u[k]).slice();u[2]+=.015*vx/.05;
   return {state:ref(k),next:ref(k+1),u,...Object.fromEntries(['A','B','K','L'].map(key=>[key,terminal?s[key]:s[key][k]]))};
  }
  return {profileId:p.id,vx,origin,height,acceptedHeight,interpolation:mix,reference,command(k,estimate){const r=reference(k),f=mv(r.K,estimate.map((v,j)=>v-r.state[j]));return {...r,u:r.u.map((v,j)=>Math.max(-backend.limits[j],Math.min(backend.limits[j],v-f[j])))};},observe(k,estimate,u,y){const r=reference(k),a=mv(r.A,estimate.map((v,j)=>v-r.state[j])),b=mv(r.B,u.map((v,j)=>v-r.u[j])),pred=r.next.map((v,j)=>v+a[j]+b[j]),c=mv(r.L,y.map((v,j)=>v-pred[j===5?10:j]));return pred.map((v,j)=>v+c[j]);}};
 }};
}
