// Offline plan + scheduled feedback, exact simulator state; no online optimizer.
const admittedBundles=new WeakSet();
const indices=[0,1,2,3,4,6,7,8,9,10,11],limits=[16,16,1.7];
export const trackingProfileHash='8ecc3f2719e63c5dfc3835279871506ba13e04158f5a81b170c2544926d0a67c';
export const hashText=async text=>Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',new TextEncoder().encode(text))),v=>v.toString(16).padStart(2,'0')).join('');
const freeze=x=>{if(x&&typeof x==='object'){Object.values(x).forEach(freeze);Object.freeze(x);}return x;};
const shape=(a,d)=>d.length?Array.isArray(a)&&a.length===d[0]&&a.every(v=>shape(v,d.slice(1))):typeof a==='number'&&Number.isFinite(a);
export function createTrackingController(profile,identities){
 const p=structuredClone(profile);
 if(p?.schema!=='wheelbot-contact-tracking/v1'||p.branchDiagnostics?.selected_derivatives_validated!==true||p.dt!==.002||p.steps!==250||JSON.stringify(p.controlledIndices)!==JSON.stringify(indices)||JSON.stringify(p.limitsNm)!==JSON.stringify(limits))throw Error('Invalid tracking metadata');
 for(const key of ['model_sha256','cost_sha256','trace_sha256','protocol_sha256'])if(!identities?.[key]||p.identities?.[key]!==identities[key])throw Error('Tracking identity mismatch: '+key);
 for(const sign of [-1,1]){const r=p.trajectories?.[sign];if(!r||!shape(r.ref,[251,12])||!shape(r.u,[250,3])||!shape(r.K,[250,3,11])||r.u.some(u=>u.some((v,i)=>Math.abs(v)>limits[i])))throw Error('Malformed tracking trajectory');}
 return {initial(sign){if(![-1,1].includes(sign))throw Error('Invalid sign');return p.trajectories[sign].ref[0].slice();},command(x,k,sign,stage='tracking',feedback=true){
  if(stage!=='tracking'||!Number.isInteger(k)||k<0||k>=250||![-1,1].includes(sign)||typeof feedback!=='boolean'||!shape(x,[12]))throw Error('Invalid state, stage, sign or expired schedule');
  const r=p.trajectories[sign],requested=r.u[k].map((v,i)=>v-(feedback?r.K[k][i].reduce((sum,g,j)=>sum+g*(x[indices[j]]-r.ref[k][indices[j]]),0):0));
  if(!requested.every(Number.isFinite))throw Error('Nonfinite torque');
  const u=requested.map((v,i)=>Math.max(-limits[i],Math.min(limits[i],v)));
  return {u,requested,saturated:u.some((v,i)=>v!==requested[i])};
 }};
}
export async function loadTrackingBundle({profileText,xml,costText,protocolText,traceSha256}){
 const p=JSON.parse(profileText),protocol=JSON.parse(protocolText);
 const identities={model_sha256:await hashText(xml),cost_sha256:await hashText(costText),protocol_sha256:await hashText(protocolText),trace_sha256:traceSha256};
 if(await hashText(profileText)!==trackingProfileHash||protocol.model_sha256!==identities.model_sha256)throw Error('Unapproved tracking source');
 const controller=createTrackingController(p,identities);
 const bundle=freeze({controller,identities,profileSha256:trackingProfileHash,protocol});
 admittedBundles.add(bundle);return bundle;
}
export function createTrackingTrial(backend,bundle,{sign=1,variant='nominal',feedback=true}={}){
 if(!admittedBundles.has(bundle))throw Error('Unapproved tracking bundle');
 if(backend.assetSha256!==bundle.identities.model_sha256)throw Error('Active XML mismatch; current state retained');
 const v=bundle.protocol.variants.find(v=>v.name===variant);if(!v||typeof feedback!=='boolean')throw Error('Undeclared tracking variant');
 let x=bundle.controller.initial(sign);x[v.index]+=v.offset;
 // Validate all choices before any plant call. INITIALRESET is explicit setup.
 bundle.controller.command(x,0,sign,'tracking',feedback);
 const initial=x.slice(),g0=backend.trackingGeometry(x,sign),history=[];
 let steps=0,last=null,failed=g0.penetration>.02||g0.excursion>.05;
 let peakRise=0,peakPen=g0.penetration,peakExc=g0.excursion,overshoot=0,tailError=0,tailRate=0,saturatedCommands=0;
 const result=()=>({saturatedCommands,tracking_success:steps===250&&!failed&&peakRise>=.005&&tailError<=.02&&tailRate<=.1&&overshoot<=.02,final_window_angle_error_rad:tailError,final_window_pitch_speed_rad_s:tailRate,overshoot_rad:overshoot,edge_rise_m:peakRise,max_penetration_m:peakPen,max_joint_excursion_rad:peakExc});
 const snapshot=()=>structuredClone({truth:x,estimate:null,steps,last,done:steps===250||failed,failed,tracking:true,controlDt:.002,elapsedSeconds:steps*.002,trackingResult:result(),initial,sign,variant,feedback,sourceHashes:bundle.identities,profileSha256:bundle.profileSha256});
 return {controlDt:.002,snapshot,get history(){return structuredClone(history);},step(externalX=0){
  if(!Number.isFinite(externalX)||externalX!==0)throw Error('Tracking does not accept external force');
  if(steps===250||failed)throw Error('Tracking expired or physical allowance exceeded');
  const command=bundle.controller.command(x,steps,sign,'tracking',feedback),next=backend.physicsStep(x,command.u);
  x=next.truth;steps++;if(command.saturated)saturatedCommands++;
  const g=backend.trackingGeometry(x,sign),rise=g.edge-g0.edge,rotation=sign*(x[2]-initial[2]);
  peakRise=Math.max(peakRise,rise);peakPen=Math.max(peakPen,g.penetration,next.penetration);peakExc=Math.max(peakExc,g.excursion);overshoot=Math.max(overshoot,rotation-.08);
  if(steps>200){tailError=Math.max(tailError,Math.abs(rotation-.08));tailRate=Math.max(tailRate,Math.abs(x[8]));}
  failed=peakPen>.02||peakExc>.05||!next.externalForcesZero;
  last={...command,edgeRiseM:rise,rotationRad:rotation,pitchRate:x[8],allContacts:backend.contactDetails(x,command.u),externalForcesZero:next.externalForcesZero};
  history.push({time_s:steps*.002,after:x.slice(),tau:command.u.slice(),requested:command.requested.slice(),saturated:command.saturated,edge_rise_m:rise,penetration_m:Math.max(g.penetration,next.penetration),joint_excursion_rad:g.excursion,external_forces_zero:next.externalForcesZero});
  return snapshot();
 }};
}
