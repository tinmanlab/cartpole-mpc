import assert from 'node:assert/strict';
import fs from 'node:fs';
import {createHash}from 'node:crypto';
import {createTargetJump} from '../src/wheelbot_target_jump.mjs';
import {createWheelbotContactBackend} from '../src/wheelbot_contact_backend.mjs';
const read=p=>JSON.parse(fs.readFileSync(p));
const bundle=read('assets/wheelbot/target_jump.json'),base=read('assets/wheelbot/live_profile.json');
const backend=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));
try{
 assert.equal(bundle.nativeVersion,backend.diagnostics().version,'Actual WASM/generation version disagreement; rebuild, never relabel');
 const counter=backend.diagnostics().steps;
 for(const bad of [{...bundle,assetSha256:'old-ellipsoid'},{...bundle,externalForce:[1,0,0]}])assert.throws(()=>createTargetJump(backend,base,bad,{diagnostic:true}));
 const corrupt=structuredClone(bundle);corrupt.profiles[0].K[0][0][0]=NaN;assert.throws(()=>createTargetJump(backend,base,corrupt,{diagnostic:true}));
 if(bundle.releaseValidated)assert(createTargetJump(backend,base,bundle));
 const planner=createTargetJump(backend,base,bundle,{diagnostic:true});
 const invalid=Array(11).fill(0);const copy=invalid.slice();assert.throws(()=>planner.start(invalid,{x:0,z:.1}));assert.deepEqual(invalid,copy);assert.equal(backend.diagnostics().steps,counter);
 const raw=read('test-results/target-jump-native.json');assert.equal(raw.report.profileSha256,createHash('sha256').update(fs.readFileSync('assets/wheelbot/target_jump.json')).digest('hex'));
 assert.equal(raw.report.nativeVersion,backend.diagnostics().version);
 const results=[],planning=[],feedback=[];let maxState=0,maxTorque=0,maxEstimate=0,maxSnapshot=0;
 for(const trial of raw.traces){
  const pr=bundle.profiles.find(p=>p.id===trial.profileId),target={x:pr.referenceMetrics.apex[0]+trial.initialEstimate[0],z:pr.referenceMetrics.apex[1],mode:'wheel'};
  const a=performance.now(),plan=planner.start(trial.initialEstimate,target);planning.push(performance.now()-a);assert.equal(plan.profileId,trial.profileId);assert.deepEqual(plan.entryEstimate,trial.initialEstimate);
  for(const dx of [-.06,0,.06])for(const z of [.08,.10]){const request={x:trial.initialEstimate[0]+dx,z};const projected=planner.start(trial.initialEstimate,request);assert.deepEqual(projected.requested,request);assert(projected.accepted.wheelPath.every(v=>v.length===2&&v.every(Number.isFinite)));assert(projected.accepted.basePath.every(v=>v.length===2&&v.every(Number.isFinite)));}
  let x=trial.initial.slice(),estimate=trial.initialEstimate.slice(),spin=0,pen=0,exc=0,pitch=0,flight=0,longest=0,apex=[0,0],landing=null,work=0,landSpeed=null,wasAir=false;const tail=[];
  for(const [k,native]of trial.trace.entries()){
   const a=performance.now(),action=plan.command(k,estimate);feedback.push(performance.now()-a);maxTorque=Math.max(maxTorque,...action.u.map((v,j)=>Math.abs(v-native.u[j])));
   for(let j=0;j<5;j++){
    const step=backend.physicsStep(x,action.u);assert(step.externalForcesZero);x=step.truth;const g=backend.geometry(x),contact=backend.contactDetails(x,action.u),air=g.wheel[2]>.055&&!contact.pairs.some(p=>p.geom1==='wheel_visual'||p.geom2==='wheel_visual');
    if(air){wasAir=true;work+=Math.abs(action.u[2]*x[11])*.002;}else if(wasAir&&!landing&&contact.pairs.some(p=>p.geom1==='wheel_visual'||p.geom2==='wheel_visual')){landing=[g.wheel[0],g.wheel[2]];const dt=1e-7,next=x.slice();for(let j=0;j<6;j++)next[j]+=dt*x[j+6];landSpeed=(backend.geometry(next).wheel[0]-g.wheel[0])/dt-.05*x.slice(8).reduce((a,b)=>a+b,0);}
    flight=air?flight+.002:0;longest=Math.max(longest,flight);if(air)spin=Math.max(spin,Math.abs(x.slice(8).reduce((s,v)=>s+v,0)));if(g.wheel[2]>apex[1])apex=[g.wheel[0],g.wheel[2]];pen=Math.max(pen,contact.maximumPenetrationM);exc=Math.max(exc,0,Math.abs(x[3])-1.26,Math.abs(x[4])-2.51);pitch=Math.max(pitch,Math.abs(x[2]));
   }
   // Reuse the native random draws, applied to this runtime's actual sensors.
   const measurement=bundle.measurementIndices.map((j,i)=>x[j]+native.measurement[i]-native.after[j]);estimate=plan.observe(k,estimate,action.u,measurement);
   maxState=Math.max(maxState,...x.map((v,j)=>Math.abs(v-native.after[j])));maxEstimate=Math.max(maxEstimate,...estimate.map((v,j)=>Math.abs(v-native.estimate[j])));tail.push(x);
  }
  const terminalRate=Math.max(...tail.slice(-50).flatMap(x=>x.slice(6).map(Math.abs))),apexError=Math.hypot(apex[0]-plan.accepted.apex[0],apex[1]-plan.accepted.apex[1]);
  const landingError=landing?Math.hypot(landing[0]-plan.accepted.landing[0],landing[1]-plan.accepted.landing[1]):Infinity;
  for(const h of trial.trace.filter((_,k)=>k%25===0)){const snapshot=backend.stepWrench(h.before,h.u);maxSnapshot=Math.max(maxSnapshot,...snapshot.map((v,j)=>Math.abs(v-h.after[j])));}
  const passed=Number.isFinite(landSpeed)&&Math.abs(landSpeed)<=bundle.protocol.maxLandingTangentialSpeed&&landingError<=.03&&trial.trace.length===600&&longest>=.03&&apex[1]>=.07&&spin<=25&&pen<=.01&&exc<=.02&&pitch<=.6&&terminalRate<=.3&&apexError<=.03;
  results.push({profileId:trial.profileId,seed:trial.seed,passed,flightS:longest,apex,apexError,landing,landingError,landingTangentialSpeed:landSpeed,airborneAbsoluteMotorWorkJ:work,airborneWheelRate:spin,penetration:pen,jointOverrun:exc,pitch,terminalRate});console.log(JSON.stringify(results.at(-1)));
 }
 const p95=a=>a.sort((x,y)=>x-y)[Math.floor(a.length*.95)];const report={engineVersion:backend.diagnostics().version,manifestSha256:createHash('sha256').update(fs.readFileSync('vendor/manifest.json')).digest('hex'),results,maxSnapshot,maxState,maxTorque,maxEstimate,planningP95Ms:p95(planning),feedbackP95Ms:p95(feedback),scope:'Node official WASM; no HTTP or browser; measurements use same random draws as native'};
 const evidence=read('evidence/wheelbot_target_jump.json');evidence.wasm=report;fs.writeFileSync('evidence/wheelbot_target_jump.json',JSON.stringify(evidence,null,2)+'\n');
 fs.writeFileSync('test-results/target-jump-wasm.json',JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report));assert(results.every(r=>r.passed));assert(maxSnapshot<1e-8);assert(maxState<1e-4&&maxTorque<1e-4&&maxEstimate<1e-4,'Native/WASM parity');
}finally{backend.dispose();}
