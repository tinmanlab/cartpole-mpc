import fs from 'node:fs';
import assert from 'node:assert/strict';
import {createWheelbotContactBackend} from '../src/wheelbot_contact_backend.mjs';
import {createWheelbotActions} from '../src/wheelbot_actions.mjs';
import {createTargetJump} from '../src/wheelbot_target_jump.mjs';
const read=p=>JSON.parse(fs.readFileSync(p));
const base=read('assets/wheelbot/live_profile.json'),atlas=read('assets/wheelbot/pose_profiles.json'),bundle=read('assets/wheelbot/target_jump.json');
const b=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));
try{
 const planner=createTargetJump(b,base,bundle,{diagnostic:!bundle.releaseValidated});
 const t=createWheelbotActions(b,base,atlas,{jumpPlanner:planner,seed:7});
 for(let k=0;k<100;k++)t.step();
 let before=t.snapshot();await t.requestPath([{x:.03,z:.09}],{mode:'wheel',action:'jump'});assert(!t.snapshot().path.available);assert.deepEqual(t.snapshot().truth,before.truth);assert.deepEqual(t.snapshot().estimate,before.estimate);
 await t.requestPath([{x:0,z:.3925}],{pitch:0,yieldTask:async()=>{}});assert(t.snapshot().path.available,t.snapshot().actionStatus);
 for(let k=0;k<800;k++)assert(!t.step().failed);
 let waitSteps=0;for(;waitSteps<300;waitSteps++){try{planner.start(t.snapshot().estimate,{x:.03,z:.09,mode:'wheel'});break;}catch{t.step();}}
 before=t.snapshot();await t.requestPath([{x:.03,z:.09}],{mode:'wheel',action:'jump'});let s=t.snapshot();assert(s.path.available,s.actionStatus);assert.equal(s.phase,'jump');assert.deepEqual(s.truth,before.truth);assert.deepEqual(s.estimate,before.estimate);
 let maxSpin=0,maxPen=0;
 for(let k=0;k<600;k++){s=t.step();assert(!s.failed,JSON.stringify(s.last.physical));maxSpin=Math.max(maxSpin,s.last.physical.maximumAirborneWheelRate);maxPen=Math.max(maxPen,s.last.physical.maximumPenetrationM);}
 assert.equal(s.phase,'standing');assert(Math.abs(s.appliedTarget.z-.3925)<.001);
 for(let k=0;k<200;k++)assert(!t.step().failed);
 const report={engine:b.diagnostics().version,passed:true,noReset:true,ordinaryLowPose:true,waitSteps,commands:600,maxSpin,maxPen,terminal:t.snapshot().truth};
 fs.writeFileSync('test-results/jump-actions315.json',JSON.stringify(report,null,2));console.log(JSON.stringify(report));
}finally{b.dispose();}
