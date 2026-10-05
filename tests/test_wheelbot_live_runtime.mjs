import fs from 'node:fs';
import assert from 'node:assert/strict';
import {createWheelbotBackend} from '../src/wheelbot_backend.mjs';
import {createWheelbotTrial} from '../src/wheelbot_control.mjs';
import {pipelineLabels,createRuntimeMeter} from '../src/wheelbot_live_view.mjs';
for(const [state,expected] of [
 [{liveActive:true,mode:'lqr_kf'},['Six noisy channels: five poses + wheel encoder rate','KF state estimate','LQR feedback','MuJoCo full contact']],
 [{liveActive:true,mode:'passive'},['Six noisy channels: five poses + wheel encoder rate','KF state estimate','Passive · no feedback','MuJoCo full contact']],
 [{mode:'mpc_kf'},['Five noisy position channels','KF state estimate','Local MPC feedback','MuJoCo wheel-contact benchmark']],
 [{configuredModel:true,mode:'lqr_kf'},['Five noisy position channels','KF state estimate','LQR feedback','MuJoCo full contact']],
 [{mode:'lqr_kf'},['Five noisy position channels','KF state estimate','LQR feedback','MuJoCo wheel-contact benchmark']],
 [{tracking:true,feedback:true},['Exact simulator state','No observer','Scheduled feedback','MuJoCo full contact']],
 [{tracking:true,feedback:false},['Exact simulator state','No observer','Planned torques · no feedback','MuJoCo full contact']],
 [{mode:'contact-diagnostic',scenario:'torque',configuredModel:true},['Simulator state display','No observer','Manual torques · no feedback','MuJoCo full contact']],
 [{mode:'contact-diagnostic',scenario:'standing',configuredModel:true},['Simulator state display','No observer','Passive · no feedback','MuJoCo full contact']],
 [{poseOnly:true},['Pose preview · no sensor sampling','No observer','No control','MuJoCo wheel-contact benchmark']],
 [{mode:'tvlqr_kf'},['Five noisy position channels','KF state estimate','Scheduled TVLQR feedback','MuJoCo wheel-contact benchmark']],
 [{},['Simulator state display','No observer','No control','MuJoCo wheel-contact benchmark']],
]) assert.deepEqual(pipelineLabels(state),expected);
let now=0;const meter=createRuntimeMeter(()=>now);meter.start(0);now=1000;meter.sampleControl(.4,1);
assert.equal(meter.snapshot().realTimeFactor,1);meter.pause();const paused=meter.snapshot();now=9000;
assert.deepEqual(meter.snapshot(),paused);assert.equal(paused.active,false);assert.equal(paused.realTimeFactor,null);
meter.start(1);now=10000;meter.sampleControl(.5,1.5);assert.equal(meter.snapshot().realTimeFactor,.5);
meter.reset();assert.equal(meter.snapshot().simulatedSeconds,0);assert.equal(meter.snapshot().wallSeconds,0);
console.log('Mode-specific pipeline and pause-excluding measured runtime PASS');
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
 const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);assert.equal(new Set(ids).size,ids.length,'Duplicate page element IDs');
 assert(html.includes('id="robot-model"')&&html.includes('id="live-left"'),'Visible live model and movement controls missing');
 assert(html.indexOf('id="view"')<html.indexOf('id="physical-editor"'),'Primary robot viewport must precede advanced settings');
 console.log('New-model KF control, state-preserving setpoint, bent posture and viewport-first layout PASS');
}finally{b.dispose();}
