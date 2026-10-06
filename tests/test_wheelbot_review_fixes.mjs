import fs from 'node:fs';
import assert from 'node:assert/strict';
import {createWheelbotContactBackend} from '../src/wheelbot_contact_backend.mjs';
import {createWheelbotActions,simplifyPath} from '../src/wheelbot_actions.mjs';
const p=JSON.parse(fs.readFileSync('assets/wheelbot/live_profile.json')),a=JSON.parse(fs.readFileSync('assets/wheelbot/pose_profiles.json'));
const b=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));
try{
 const line=Array.from({length:40},(_,i)=>({x:i/39,z:.425})),simple=simplifyPath(line);assert.equal(simple.points.length,2);assert(simple.maxDeviationM<1e-12);
 const corner=[{x:0,z:.4},{x:.1,z:.4},{x:.1,z:.45}];assert.deepEqual(simplifyPath(corner).points,corner);
 assert.throws(()=>createWheelbotActions(b,p,a,{initial:{truth:Array(12).fill(NaN),estimate:Array(11).fill(0),appliedTarget:{x:0,z:.4,pitch:0}}}));
 const t=createWheelbotActions(b,p,a);const cold=t.snapshot();await t.requestPath([{x:.05,z:.45}]);assert(!t.snapshot().path.available&&t.snapshot().path.reason.includes('0.20'));assert.deepEqual(t.snapshot().truth,cold.truth);assert.equal(b.diagnostics().steps,0);
 for(let k=0;k<100;k++)t.step();await t.requestPath(line,{yieldTask:async()=>{}});assert.equal(t.snapshot().path.compression.output,2);assert(t.snapshot().path.attempts.some(a=>a.steps>0),'Compressed drag must enter nonlinear preview within the 12 s bound');
 for(const kind of ['push','gust','twist'])for(const strength of ['light','medium','strong']){
  const trial=createWheelbotActions(b,p,a);for(let k=0;k<100;k++)trial.step();const before=trial.snapshot();trial.disturb({kind,strength,direction:-1});assert.deepEqual(trial.snapshot().truth,before.truth);assert.deepEqual(trial.snapshot().estimate,before.estimate);
  let nonzero=0;for(let k=0;k<26;k++){const s=trial.step(),w=s.last.externalWrench;assert(Math.abs(w[0])<=4&&Math.abs(w[2])<=.1);if(w.some(v=>v))nonzero++;}
  assert(nonzero>0&&nonzero<=25);assert.equal(trial.snapshot().disturbance,null);assert.deepEqual(trial.step().last.externalWrench,[0,0,0]);
 }
 console.log('Cold admission, path compression/corners, injected-state rejection and all 9 disturbance combinations PASS');
}finally{b.dispose();}
