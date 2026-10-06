import fs from 'node:fs';
import assert from 'node:assert/strict';
import {createWheelbotContactBackend} from '../src/wheelbot_contact_backend.mjs';
import {createWheelbotActions} from '../src/wheelbot_actions.mjs';
const read=p=>JSON.parse(fs.readFileSync(p));
const base=read('assets/wheelbot/live_profile.json'),poses=read('assets/wheelbot/pose_profiles.json'),jump=read('assets/wheelbot/action_jump.json');
const b=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));
try{
 const t=createWheelbotActions(b,base,poses,{seed:17,jumpProfile:jump});
 assert.equal(t.snapshot().capabilities.jump,true);
 for(let k=0;k<200;k++)t.step();
 const before=t.snapshot();t.jump();const requested=t.snapshot();
 assert.deepEqual(requested.truth,before.truth,'Jump request must not teleport');
 assert.deepEqual(requested.estimate,before.estimate,'Jump request must not reset the observer');
 assert.equal(requested.steps,before.steps);
 assert.throws(()=>t.setTarget({x:.3}),'Target editing during a prepared jump must not change the active plan');
 let maxClear=0,flight=0,longest=0;let seenJump=false;
 for(let k=0;k<1200;k++){
  const s=t.step();seenJump ||= s.phase==='jumping';
  const g=b.actionTelemetry(s.truth,s.last.u);maxClear=Math.max(maxClear,g.wheelClearanceM??0);
  flight=s.last.contact.wheelContacts===0?flight+1:0;longest=Math.max(longest,flight);
  if(s.lastJumpResult)break;
 }
 const final=t.snapshot();assert(seenJump&&longest>=3,{seenJump,longest});
 assert(final.lastJumpResult?.passed,final.lastJumpResult);
 assert(!final.failed);assert.equal(b.assetSha256,base.assetSha256);
 console.log(JSON.stringify({passed:true,steps:final.steps,jump:final.lastJumpResult,longestFlightSeconds:longest*.01}));
}finally{b.dispose();}
