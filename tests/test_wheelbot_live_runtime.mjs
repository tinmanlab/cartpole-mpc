import fs from 'node:fs';
import assert from 'node:assert/strict';
import {createWheelbotBackend} from '../src/wheelbot_backend.mjs';
import {createWheelbotTrial} from '../src/wheelbot_control.mjs';
const xml=fs.readFileSync('assets/wheelbot/live_model.xml','utf8');
const p=JSON.parse(fs.readFileSync('assets/wheelbot/live_profile.json'));
const b=await createWheelbotBackend(xml);
try{
 const trial=createWheelbotTrial(b,p,{seed:17,mode:'lqr_kf'});
 assert.equal(typeof trial.setGoal,'function','Live setpoint must not reset plant or KF');
 for(let i=0;i<200;i++)assert(!trial.step().failed);
 const before=trial.snapshot();trial.setGoal(.03);const after=trial.snapshot();
 assert.deepEqual(before.truth,after.truth);assert.deepEqual(before.estimate,after.estimate);assert.equal(before.steps,after.steps);assert.equal(after.goal,.03);
 assert.throws(()=>trial.setGoal(NaN));assert.deepEqual(trial.snapshot(),after);
 for(let i=0;i<600;i++)assert(!trial.step().failed);
 assert(Math.abs(trial.snapshot().truth[0]-.03)<.02);
 assert(Math.abs(p.qref[4])>.9&&Math.abs(p.qref[4])<1.3);
 const html=fs.readFileSync('wheelbot.html','utf8');
 assert(html.includes('id="robot-model"')&&html.includes('id="live-left"'),'Visible live model and movement controls missing');
 assert(html.indexOf('id="view"')<html.indexOf('id="physical-editor"'),'Primary robot viewport must precede advanced settings');
 console.log('New-model KF control, state-preserving setpoint, bent posture and viewport-first layout PASS');
}finally{b.dispose();}
