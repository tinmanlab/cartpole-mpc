import assert from 'node:assert/strict';import fs from 'node:fs';
import{createGameWorld}from'../src/wheelbot_game_world.mjs';import{createGameController}from'../src/wheelbot_game_control.mjs';
const read=n=>JSON.parse(fs.readFileSync('assets/wheelbot/'+n+'.json'));
const b=await createGameWorld(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'),'flat');
try{for(const bundle of [null,{...read('target_jump'),nativeVersion:'mismatch'}]){
 const c=createGameController(b,read('live_profile'),read('pose_profiles'),bundle);
 assert.equal(c.snapshot().jumpAvailable,false);const before=c.snapshot();c.beginCharge();c.releaseCharge();assert.deepEqual(c.snapshot().truth,before.truth);assert.deepEqual(c.snapshot().estimate,before.estimate);assert.equal(c.snapshot().charge.active,false);
 c.setInput({horizontal:1});for(let k=0;k<200;k++)assert(!c.step().failed);assert(c.snapshot().truth[6]>.15);
 assert(c.snapshot().jumpUnavailableReason.length>0);
}console.log('Invalid optional jump disables jumping, not ordinary keyboard balance PASS');}finally{b.dispose();}
