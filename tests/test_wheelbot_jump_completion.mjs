import assert from 'node:assert/strict';
import {jumpCompleted} from '../src/wheelbot_actions.mjs';
const truth=[0,.3925,0,0,0,0,0,0,0,0,0,0], reference=[0,.3925,0,0,0,0,0,0,0,0,0];
const good={flight:true,landed:true,maxClearance:.025,truth,reference};
assert.equal(jumpCompleted({...good,flight:false,landed:false,maxClearance:0}),false,'600 ground commands are not a jump');
assert.equal(jumpCompleted({...good,landed:false}),false);
assert.equal(jumpCompleted({...good,truth:truth.map((v,i)=>i===6?1:v)}),false);
assert.equal(jumpCompleted({...good,truth:truth.map((v,i)=>i===1?v+.1:v)}),false);
for(const patch of [{truth:[]},{reference:[]},{maxClearance:Infinity},{truth:null},{flight:1},{landed:'yes'}])assert.equal(jumpCompleted({...good,...patch}),false,'Malformed completion data must fail closed');
for(const record of [undefined,null,0,[],{}, {...good,truth:Array(12)}, {...good,reference:Array(11)}, {...good,truth:[...truth,0]}, {...good,reference:reference.map((v,i)=>i===3?NaN:v)}])assert.equal(jumpCompleted(record),false,'Missing, sparse or nonfinite data must not pass');
assert.equal(jumpCompleted(good),true);
console.log('Observed flight, landing and terminal balance completion PASS');
// Freeze the viewer's default seed using the actual action manager and MuJoCo.
const {readFileSync}=await import('node:fs');
const {createWheelbotContactBackend}=await import('../src/wheelbot_contact_backend.mjs');
const {createWheelbotActions}=await import('../src/wheelbot_actions.mjs');
const {createTargetJump}=await import('../src/wheelbot_target_jump.mjs');
const read=p=>JSON.parse(readFileSync(p));
const base=read('assets/wheelbot/live_profile.json'),atlas=read('assets/wheelbot/pose_profiles.json');
const backend=await createWheelbotContactBackend(readFileSync('assets/wheelbot/live_model.xml','utf8'));
try{
 const planner=createTargetJump(backend,base,read('assets/wheelbot/target_jump.json'));
 const actions=createWheelbotActions(backend,base,atlas,{seed:7,jumpPlanner:planner});
 for(let k=0;k<100;k++)actions.step();
 await actions.requestPath([{x:0,z:.3925}],{pitch:0,yieldTask:async()=>{}});
 assert(actions.snapshot().path.available,actions.snapshot().actionStatus);
 for(let k=0;k<800;k++)assert(!actions.step().failed);
 let settling=0;
 while(!actions.jumpAdmission().ready&&settling<300){assert(!actions.step().failed);settling++;}
 assert(actions.jumpAdmission().ready,actions.jumpAdmission().reason);
 const before=actions.snapshot();
 await actions.requestPath([{x:.03,z:.09}],{mode:'wheel',action:'jump'});
 assert.equal(actions.snapshot().phase,'jump');assert.deepEqual(actions.snapshot().truth,before.truth);
 assert.deepEqual(actions.snapshot().estimate,before.estimate);
 for(let k=0;k<600;k++)assert(!actions.step().failed);
 assert(actions.snapshot().jumpOutcome.success,JSON.stringify(actions.snapshot().jumpOutcome));
 for(let k=0;k<200;k++)assert(!actions.step().failed);
 console.log('Seed 7 ordinary low pose, settling '+settling+', physical jump and terminal balance PASS');
}finally{backend.dispose();}
