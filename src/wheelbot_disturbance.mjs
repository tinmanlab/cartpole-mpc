// A task-specific force protocol on the existing physical trial; no new dynamics.
import {createWheelbotTrial} from './wheelbot_control.mjs';
const clone=x=>JSON.parse(JSON.stringify(x));
const freeze=x=>{if(x&&typeof x==='object'){Object.values(x).forEach(freeze);Object.freeze(x);}return x;};
const key=r=>[r.amplitudeN,r.mode,r.seed,r.direction].join('/');
export function validateForceProtocol(p){
 if(p?.schema!=='wheelbot-force-envelope-protocol/v1'||p.controlDt!==.01||p.steps!==300||p.pulseStart!==100||p.pulseSteps!==20)throw Error('Unsupported force task/time');
 if(JSON.stringify(p.controllers)!=='["lqr_kf","mpc_kf"]'||JSON.stringify(p.directions)!=='[-1,1]')throw Error('Unsupported controller/direction coverage');
 if(!Array.isArray(p.amplitudesN)||p.amplitudesN.some((a,i)=>!Number.isFinite(a)||a<=0||(i&&a<=p.amplitudesN[i-1])))throw Error('Invalid increasing amplitude screen');
 const seeds=[...p.screenSeeds,...p.assessmentSeeds];if(seeds.some(s=>!Number.isInteger(s)||s<0)||new Set(seeds).size!==seeds.length)throw Error('Screen and assessment seeds must be disjoint');
 if(p.selection.amplitudeMarginFraction!==.8||p.selection.resolutionN!==.1||p.stressN!==40)throw Error('Changed force-selection rule');
 if(p.task.tailSteps!==50||p.task.positionToleranceM!==.015||p.task.pitchToleranceRad!==.025||JSON.stringify(p.task.failure)!=='{"positionMagnitudeM":1,"pitchErrorRad":0.6,"minimumZM":0.12}')throw Error('Changed physical/task acceptance');
 return true;
}
export function createForceTrial(backend,profile,protocol,{amplitudeN,direction=1,mode='lqr_kf',seed=901}={}){
 validateForceProtocol(protocol);
 if(!Number.isFinite(amplitudeN)||amplitudeN<0||amplitudeN>protocol.stressN||!protocol.directions.includes(direction)||!protocol.controllers.includes(mode)||!Number.isInteger(seed)||seed<0)throw Error('Invalid force recipe');
 const trial=createWheelbotTrial(backend,profile,{mode,seed,goal:0});let last=trial.snapshot(),rejection=null;const history=[];
 const snapshot=()=>({...last,done:last.failed||last.steps>=protocol.steps||rejection!==null,forceTask:{amplitudeN,direction,durationSeconds:protocol.pulseSteps*protocol.controlDt,point:protocol.forcePoint,rejection}});
 return {history,snapshot,step(externalOverride=0){
  if(externalOverride!==0)throw Error('Force task does not accept an additional hidden disturbance');
  if(snapshot().done)throw Error('Force task complete/rejected; explicit reset required');
  const force=last.steps>=protocol.pulseStart&&last.steps<protocol.pulseStart+protocol.pulseSteps?amplitudeN*direction:0;
  try{last=trial.step(force);history.push(clone(last));}
  catch(e){if(!String(e.message).startsWith('QP rejected:'))throw e;rejection=e.message;}
  return snapshot();
 },result(){
  const tail=history.slice(-protocol.task.tailSteps),completed=last.steps===protocol.steps&&!last.failed&&!rejection;
  const tailPositionMax=tail.length?Math.max(...tail.map(s=>Math.abs(s.truth[0]-profile.qref[0]))):null;
  const tailPitchMax=tail.length?Math.max(...tail.map(s=>Math.abs(s.truth[2]-profile.qref[2]))):null;
  const taskPassed=completed&&tail.length===protocol.task.tailSteps&&tailPositionMax<protocol.task.positionToleranceM&&tailPitchMax<protocol.task.pitchToleranceRad;
  const contactLoss=history.filter(s=>!s.last.contact.wheelContacts).length;
  return {amplitudeN,direction,mode,seed,steps:last.steps,completed,taskPassed,normalPassed:taskPassed&&contactLoss===0,contactLoss,physicalFailure:last.failed,qpRejected:rejection!==null,rejection,tailPositionMax,tailPitchMax,maxPosition:history.length?Math.max(...history.map(s=>Math.abs(s.truth[0]-profile.qref[0]))):null,maxPitch:history.length?Math.max(...history.map(s=>Math.abs(s.truth[2]-profile.qref[2]))):null,peakTorqueNm:history.length?[0,1,2].map(j=>Math.max(...history.map(s=>Math.abs(s.last.u[j])))):null,final:last.truth.slice(),fullPulseImpulseNs:amplitudeN*protocol.pulseSteps*protocol.controlDt,appliedSignedImpulseNs:history.reduce((sum,s)=>sum+s.last.externalX*protocol.controlDt,0)};
 }};
}
export function selectNormalForce(p,rows){
 validateForceProtocol(p);
 const expected=[0,...p.amplitudesN].flatMap(amplitudeN=>p.controllers.flatMap(mode=>p.screenSeeds.flatMap(seed=>p.directions.map(direction=>({amplitudeN,mode,seed,direction})))));
 if(rows.length!==expected.length||new Set(rows.map(key)).size!==expected.length||expected.some(r=>!rows.some(s=>key(r)===key(s))))throw Error('Incomplete/duplicate screen coverage');
 const groups=[0,...p.amplitudesN].map(amplitudeN=>{const rr=rows.filter(r=>r.amplitudeN===amplitudeN);return{amplitudeN,passed:rr.filter(r=>r.normalPassed).length,cases:rr.length,allPassed:rr.every(r=>r.normalPassed),completed:rr.filter(r=>r.completed).length,contactLossCases:rr.filter(r=>r.contactLoss>0).length};});
 let upper=0;
 if(groups[0].allPassed)for(const g of groups.slice(1)){if(!g.allPassed)break;upper=g.amplitudeN;}
 const normal=upper>0?Math.floor((upper*p.selection.amplitudeMarginFraction+1e-10)/p.selection.resolutionN)*p.selection.resolutionN:null;
 return freeze({schema:'wheelbot-force-selection/v1',normalAmplitudeN:normal===null?null:Number(normal.toFixed(1)),screenUpperN:upper,groups,assessmentUsedForSelection:false,protocolIdentity:JSON.stringify(p),method:p.selection.method,scope:p.normalScope});
}
export function admitNormalForce(p,selection,rows){
 if(!Object.isFrozen(selection)||selection.protocolIdentity!==JSON.stringify(p))throw Error('Invalid frozen force selection');
 const expected=p.controllers.flatMap(mode=>p.assessmentSeeds.flatMap(seed=>p.directions.map(direction=>({amplitudeN:selection.normalAmplitudeN,mode,seed,direction}))));
 const complete=rows.length===expected.length&&new Set(rows.map(key)).size===expected.length&&expected.every(r=>rows.some(s=>key(r)===key(s)));
 return freeze({accepted:selection.normalAmplitudeN!==null&&complete&&rows.every(r=>r.normalPassed),normalAmplitudeN:selection.normalAmplitudeN,expectedCases:expected.length,actualCases:rows.length,completeCoverage:complete,passed:rows.filter(r=>r.normalPassed).length,testCanReselect:false,scope:'Only declared model, same pulse point/duration, assessed signs/modes/noise seeds; not global recovery, physical OOD or hardware admission.'});
}

// Validate the optional UI receipt against the exact loaded profile and protocol.
// This is consistency checking, not a cryptographic authorization boundary.
export function validateForceReceipt(protocol,receipt,{assetSha256,profileSha256,protocolSha256}){
 if(receipt?.schema!=='wheelbot-force-envelope/v1'||receipt.assetSha256!==assetSha256||receipt.profileSha256!==profileSha256||receipt.protocolSha256!==protocolSha256)throw Error('Force receipt/model/profile identity mismatch');
 const choice=selectNormalForce(protocol,receipt.screen);
 const verdict=admitNormalForce(protocol,choice,receipt.assessment);
 if(!verdict.accepted||receipt.admission?.accepted!==true||receipt.normal?.amplitudeN!==choice.normalAmplitudeN||receipt.normal.durationSeconds!==protocol.pulseSteps*protocol.controlDt||receipt.stress?.amplitudeN!==protocol.stressN)throw Error('Force operating range is not admitted by the declared assessment');
 return freeze({amplitudeN:choice.normalAmplitudeN,durationSeconds:receipt.normal.durationSeconds,forceToWeight:receipt.normal.forceToWeight,impulseNs:receipt.normal.impulseNs,screenUpperN:choice.screenUpperN,scope:verdict.scope});
}
