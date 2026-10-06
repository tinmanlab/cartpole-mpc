import assert from 'node:assert/strict';
import fs from 'node:fs';
import {createGameController} from '../src/wheelbot_game_control.mjs';
import {createWheelbotContactBackend} from '../src/wheelbot_contact_backend.mjs';
const read=n=>JSON.parse(fs.readFileSync(`assets/wheelbot/${n}.json`));
const b=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8')),results=[],traces=[];
try{
 for(const seed of [7,42])for(const vx of [0,-.1,.1,-.2,.2])for(const duration of vx===0?[.1,.5,1]:[.1,1]){
  const c=createGameController(b,read('live_profile'),read('pose_profiles'),read('target_jump'),{seed});c.setInput({horizontal:vx/.2});
  const trace=[];const tick=()=>{const before=c.snapshot().truth,s=c.step();trace.push({before,u:s.last.u,after:s.truth,measurement:s.last.measurement,estimate:s.estimate});return s;};
  for(let k=0;k<200;k++)tick();c.beginCharge();for(let k=0;k<Math.round(duration*100);k++)tick();const before=c.snapshot();const released=c.releaseCharge();assert.deepEqual(released.truth,before.truth);assert.deepEqual(released.estimate,before.estimate);assert.equal(released.phase,'jump',released.status);
  let pen=0,spin=0,exc=0;for(let k=0;k<400;k++){const s=tick();pen=Math.max(pen,s.last.physical.maximumPenetrationM);spin=Math.max(spin,s.last.physical.maximumAirborneWheelRate);exc=Math.max(exc,s.last.physical.jointLimitExcursionRad);if(s.failed)break;}
  const s=c.snapshot(),r={seed,vx,duration,failed:s.failed,phase:s.phase,...s.jump,pen,spin,exc,finalVx:s.truth[6],beforeVx:before.truth[6],translation:s.truth[0]-before.truth[0]};results.push(r);traces.push({seed,vx,duration,trace});console.log(r);
 }
 fs.writeFileSync('evidence/wheelbot_game_jump.json',JSON.stringify({engine:b.diagnostics().version,results},null,2)+'\n');fs.writeFileSync('test-results/controller-wasm-traces.json',JSON.stringify(traces));assert(results.every(r=>!r.failed&&r.success&&r.flightS>=.03&&Math.abs(r.finalVx-r.vx)<=.04));
}finally{b.dispose();}
// Native generator is independent: it reads only checked-in XML and profiles.
const {createGameJump}=await import('../src/wheelbot_game_jump.mjs');
const native=JSON.parse(fs.readFileSync('test-results/controller-native.json'));
const parityBackend=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));
let maxStateError=0,maxEstimateError=0,maxTorqueError=0;
try{
 const planner=createGameJump(parityBackend,read('live_profile'),read('target_jump'));
 for(const t of native.traces){let x=t.initial.slice(),e=t.initialEstimate.slice();const plan=planner.start(e,t.profileId===1?0:1);for(const [k,n] of t.trace.entries()){const a=plan.command(k,e);for(let j=0;j<5;j++)x=parityBackend.physicsStep(x,a.u).truth;const y=[0,1,2,3,4,11].map((j,i)=>x[j]+n.measurement[i]-n.after[j]);e=plan.observe(k,e,a.u,y);maxStateError=Math.max(maxStateError,...x.map((v,j)=>Math.abs(v-n.after[j])));maxEstimateError=Math.max(maxEstimateError,...e.map((v,j)=>Math.abs(v-n.estimate[j])));maxTorqueError=Math.max(maxTorqueError,...a.u.map((v,j)=>Math.abs(v-n.u[j])));}}
 assert(maxStateError<1e-5&&maxEstimateError<1e-5&&maxTorqueError<1e-5);
 const file='evidence/wheelbot_game_jump.json',evidence=JSON.parse(fs.readFileSync(file));Object.assign(evidence,{native:native.report,parity:{maxStateError,maxEstimateError,maxTorqueError}});fs.writeFileSync(file,JSON.stringify(evidence,null,2)+'\n');console.log('Native/WASM parity',evidence.parity);
}finally{parityBackend.dispose();}
