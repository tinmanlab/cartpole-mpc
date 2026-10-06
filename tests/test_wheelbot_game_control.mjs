import assert from 'node:assert/strict';
import fs from 'node:fs';
assert(fs.existsSync('src/wheelbot_game_control.mjs'),'game controller must exist');
const {createGameController}=await import('../src/wheelbot_game_control.mjs');
const {createWheelbotContactBackend}=await import('../src/wheelbot_contact_backend.mjs');
const read=n=>JSON.parse(fs.readFileSync(`assets/wheelbot/${n}.json`));
const b=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));
try {
 const c=createGameController(b,read('live_profile'),read('pose_profiles'),read('target_jump'));
 assert.throws(()=>c.setInput({horizontal:NaN}));
 const results=[];
 for(const horizontal of [1,0,-1,0]){c.setInput({horizontal,vertical:0,tilt:0});for(let k=0;k<200;k++)c.step();const s=c.snapshot();assert(!s.failed);assert(Math.abs(s.truth[6]-.2*horizontal)<.04);results.push({horizontal,x:s.truth[0],vx:s.truth[6]});}
 for(const input of [{vertical:1},{vertical:-1},{tilt:1},{tilt:-1}]){c.setInput(input);for(let k=0;k<200;k++)c.step();assert(!c.snapshot().failed);}
 c.cancelInput();const stable=c.snapshot();c.beginCharge();c.step();c.beginCharge();assert.equal(c.snapshot().charge.seconds,.01);c.cancelInput();assert.equal(c.snapshot().charge.active,false);assert.equal(c.snapshot().steps,stable.steps+1);
 assert.throws(()=>createGameController({...b,assetSha256:'wrong'},read('live_profile'),read('pose_profiles'),read('target_jump')));
 const wrapped=createGameController({...b,assetSha256:'scene',robotAssetSha256:b.assetSha256},read('live_profile'),read('pose_profiles'),read('target_jump'));assert.equal(wrapped.snapshot().steps,0);
 wrapped.setInput({horizontal:1});for(let k=0;k<200;k++)wrapped.step();wrapped.beginCharge();for(let k=0;k<10;k++)wrapped.step();const release=wrapped.releaseCharge();assert.equal(release.phase,'jump');const cancelled=wrapped.cancelInput();assert.equal(cancelled.phase,'jump');assert.deepEqual(cancelled.truth,release.truth);assert.deepEqual(cancelled.estimate,release.estimate);for(let k=0;k<400;k++)wrapped.step();assert(!wrapped.snapshot().failed);assert(Math.abs(wrapped.snapshot().truth[6])<.04,'release brakes after landing');
 const s=c.snapshot();assert.equal(s.truth.length,12);assert.equal(s.estimate.length,11);s.truth[0]=999;assert.notEqual(c.snapshot().truth[0],999);
 fs.writeFileSync('evidence/wheelbot_game_control.json',JSON.stringify({engine:b.diagnostics().version,results},null,2)+'\n');console.log(results);
}finally{b.dispose();}
