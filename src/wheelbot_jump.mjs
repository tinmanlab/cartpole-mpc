// Planned nonlinear motor trajectory + scheduled estimated-state tracking.
// No optimizer, hidden physics, external jump impulse or true-state feedback.
import {CONTROLLED_INDICES} from './wheelbot_backend.mjs';
const finite=(x,n)=>Array.isArray(x)&&x.length===n&&x.every(Number.isFinite);
const matrix=(x,n,m)=>Array.isArray(x)&&x.length===n&&x.every(row=>finite(row,m));
const mv=(A,x)=>A.map(row=>row.reduce((sum,v,j)=>sum+v*x[j],0));
const add=(x,y)=>x.map((v,i)=>v+y[i]);
export function validateJumpProfile(backend,p){
 if(!backend||!p||p.schema!=='wheelbot-jump-profile/v1'||p.assetSha256!==backend.assetSha256)throw Error('Jump profile/model mismatch');
 if(p.controlDt!==.01||p.physicsDt!==.002||p.steps!==400||JSON.stringify(p.controlledIndices)!==JSON.stringify(CONTROLLED_INDICES)||JSON.stringify(p.measurementIndices)!=='[0,1,2,3,4]')throw Error('Jump time/state/measurement contract mismatch');
 if(JSON.stringify(p.limitsNm)!==JSON.stringify(backend.limits)||JSON.stringify(p.externalForce)!=='[0,0,0]')throw Error('Jump must use only the existing three torques');
 if(!Array.isArray(p.ref)||p.ref.length!==p.steps+1||!p.ref.every(x=>finite(x,12))||!Array.isArray(p.u)||p.u.length!==p.steps||!p.u.every(u=>finite(u,3)&&u.every((v,j)=>Math.abs(v)<=backend.limits[j]+1e-10)))throw Error('Invalid jump reference or torque');
 for(const [name,n,m]of [['A',11,11],['B',11,3],['K',3,11],['L',11,5]])if(!Array.isArray(p[name])||p[name].length!==p.steps||!p[name].every(x=>matrix(x,n,m)))throw Error('Invalid jump '+name+' schedule');
 if(!Array.isArray(p.phases)||p.phases.length!==p.steps||!p.phases.every(s=>['stand','crouch','thrust','settle'].includes(s)))throw Error('Invalid jump phase schedule');
 if(p.derivativeDiagnostics?.branchCrossings!==0||!Number.isFinite(p.derivativeDiagnostics?.largestGain))throw Error('Jump derivative schedule not validated');
 if(!finite(p.measurementSigma,5)||p.measurementSigma.some(x=>x<=0)||!p.target||!p.envelope)throw Error('Invalid jump sensor/target contract');
 // Fixed v1 physical acceptance; the fixture equality is checked independently.
 const target={comApexAboveInitialM:.01,wheelClearanceM:.005,continuousFlightSeconds:.03,settleSeconds:.5,positionErrorM:.015,pitchErrorRad:.025,hipErrorRad:.03,kneeErrorRad:.03};
 const envelope={positionMagnitudeM:1,pitchErrorRad:.6,minimumRootZM:.12,minimumNonadjacentSeparationM:0};
 for(const [name,want]of [['target',target],['envelope',envelope]])if(Object.keys(p[name]).length!==Object.keys(want).length||Object.entries(want).some(([key,value])=>p[name][key]!==value))throw Error('Jump '+name+' protocol mismatch');
 return true;
}
export function createJumpTrial(backend,profile,{seed=701,mode='tvlqr_kf',initialState=profile.ref[0]}={}){
 validateJumpProfile(backend,profile);
 if(!['tvlqr_kf','nominal_torque'].includes(mode))throw Error('Unsupported jump mode');
 if(!Number.isInteger(seed)||seed<0||seed>4294967295)throw Error('Invalid measurement seed');
 if(!finite(initialState,12))throw Error('Invalid jump initial state');
 const p=structuredClone(profile),idx=CONTROLLED_INDICES.slice();let rng=seed>>>0;
 const uniform=()=>{rng=(Math.imul(1664525,rng)+1013904223)>>>0;return(rng+.5)/4294967296;};
 const measure=x=>x.slice(0,5).map((v,i)=>v+p.measurementSigma[i]*Math.sqrt(-2*Math.log(uniform()))*Math.cos(2*Math.PI*uniform()));
 let truth=initialState.slice(),measurement=measure(truth),estimate=[...measurement,...Array(6).fill(0)],steps=0,failed=false,last=null;
 const history=[];const q0=p.ref[0];
 const snapshot=()=>({truth:truth.slice(),estimate:estimate.slice(),measurement:measurement.slice(),steps,failed,done:failed||steps===p.steps,last:structuredClone(last),mode,goal:0,externalForce:0,estimateIndices:idx.slice()});
 // Return isolated bounded history; callers cannot rewrite evaluated outcomes.
 return {snapshot,get history(){return structuredClone(history);},step(externalX=0){
  if(externalX!==0)throw Error('Jump experiment forbids an external boost; separate disturbance test required');
  if(failed||steps>=p.steps)throw Error('Jump trial is complete or failed; explicit reset required');
  const k=steps,reference=p.ref[k],nextReference=p.ref[k+1],error=estimate.map((v,j)=>v-reference[idx[j]]);
  const feedback=mode==='tvlqr_kf'?mv(p.K[k],error):[0,0,0];
  const requested=p.u[k].map((v,j)=>v-feedback[j]);
  const u=requested.map((v,j)=>Math.max(-p.limitsNm[j],Math.min(p.limitsNm[j],v)));
  if(!finite(u,3))throw Error('Invalid jump torque; plant not advanced');
  const previousEstimate=estimate.slice();
  truth=backend.step(truth,u,0);measurement=measure(truth);
  const delta=add(mv(p.A[k],error),mv(p.B[k],u.map((v,j)=>v-p.u[k][j])));
  const prediction=idx.map((full,j)=>nextReference[full]+delta[j]);
  const innovation=measurement.map((v,j)=>v-prediction[j]);estimate=add(prediction,mv(p.L[k],innovation));steps++;
  const geometry=backend.jumpTelemetry(truth);
  failed=!truth.every(Number.isFinite)||!estimate.every(Number.isFinite)||Math.abs(truth[0]-q0[0])>p.envelope.positionMagnitudeM||Math.abs(truth[2]-q0[2])>p.envelope.pitchErrorRad||truth[1]<p.envelope.minimumRootZM||geometry.minimumBodyFloorClearanceM<0||geometry.minimumNonadjacentDistanceM<0;
  last={u,requested,previousEstimate,prediction,innovation,measurement:measurement.slice(),saturated:requested.some((v,j)=>Math.abs(v)>p.limitsNm[j]),reference:nextReference.slice(),plannedPhase:p.phases[k],geometry,externalX:0};
  const s=snapshot();history.push(s);return structuredClone(s);
 }};
}
export function jumpMetrics(profile,history,initialTelemetry){
 if(!history.length)return{completed:false,passed:false,steps:0,comApexM:null,longestFlightSeconds:null};
 const q0=profile.ref[0],target=profile.target,dt=profile.controlDt;let flight=0,longest=0,takeoff=null,landing=null;
 for(const s of history){
  const g=s.last.geometry,air=g.wheelContacts===0&&g.wheelClearanceM>target.wheelClearanceM;
  flight=air?flight+1:0;longest=Math.max(longest,flight);
  if(air&&takeoff===null)takeoff=s.steps;
  if(takeoff!==null&&s.steps>takeoff&&g.wheelContacts>0&&landing===null)landing=s.steps;
 }
 const tail=history.slice(-Math.round(target.settleSeconds/dt)),last=history.at(-1);
 const errors=[0,2,3,4].map(i=>Math.max(...tail.map(s=>Math.abs(s.truth[i]-q0[i]))));
 const completed=last.steps===profile.steps&&!last.failed;
 const settled=completed&&tail.every(s=>s.last.geometry.wheelContacts>0)&&errors[0]<target.positionErrorM&&errors[1]<target.pitchErrorRad&&errors[2]<target.hipErrorRad&&errors[3]<target.kneeErrorRad;
 const apex=Math.max(0,...history.map(s=>s.last.geometry.com[2]-initialTelemetry.com[2]));
 return {completed,passed:completed&&settled&&apex>=target.comApexAboveInitialM&&longest*dt>=target.continuousFlightSeconds,
  steps:history.length,comApexM:apex,longestFlightSeconds:longest*dt,takeoffSample:takeoff,landingSample:landing,settled,
  tailPositionM:errors[0],tailPitchRad:errors[1],tailHipRad:errors[2],tailKneeRad:errors[3],
  maxWheelClearanceM:Math.max(...history.map(s=>s.last.geometry.wheelClearanceM)),
  peakTorqueNm:[0,1,2].map(j=>Math.max(...history.map(s=>Math.abs(s.last.u[j])))),
  minimumBodyFloorClearanceM:Math.min(...history.map(s=>s.last.geometry.minimumBodyFloorClearanceM)),
  minimumNonadjacentDistanceM:Math.min(...history.map(s=>s.last.geometry.minimumNonadjacentDistanceM)),externalForceIsZero:history.every(s=>s.last.externalX===0)};
}
