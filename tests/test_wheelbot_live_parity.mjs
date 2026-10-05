import fs from 'node:fs';
import assert from 'node:assert/strict';
import {createWheelbotBackend} from '../src/wheelbot_backend.mjs';
import {createWheelbotTrial} from '../src/wheelbot_control.mjs';
const source=JSON.parse(fs.readFileSync('test-results/live-model-reference.json','utf8'));
const summary=JSON.parse(fs.readFileSync('evidence/wheelbot_live_model.json','utf8'));
const profile=JSON.parse(fs.readFileSync('assets/wheelbot/live_profile.json','utf8'));
const backend=await createWheelbotBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));
assert.equal(source.modelSha256,backend.assetSha256);assert.equal(summary.passed,20);assert.equal(source.trials.length,20);
let stateError=0,torqueError=0,estimateInputs=0,commands=0;const times=[];
try{
 for(const test of source.trials){
  const trial=createWheelbotTrial(backend,profile,{seed:test.seed,initialState:test.initial,mode:'lqr_kf',goal:0});
  for(const expected of test.trace){
   const snapshot=trial.snapshot();trial.setGoal(expected.goal);assert.deepEqual(trial.snapshot().truth,snapshot.truth);assert.deepEqual(trial.snapshot().estimate,snapshot.estimate);
   const started=performance.now(),actual=trial.step();times.push(performance.now()-started);
   assert(!actual.failed);assert.equal(actual.last.measurement.length,6);estimateInputs+=6;
   stateError=Math.max(stateError,...actual.truth.map((v,i)=>Math.abs(v-expected.after[i])));
   torqueError=Math.max(torqueError,...actual.last.u.map((v,i)=>Math.abs(v-expected.u[i])));commands++;
  }
 }
 assert.equal(commands,20000);assert(stateError<1e-7);assert(torqueError<1e-7);
 const sorted=times.slice().sort((a,b)=>a-b);
 const result={passed:true,actualMuJoCoWasm:true,trials:source.trials.length,commands,physicsSteps:commands*5,modelSha256:backend.assetSha256,stateError,torqueError,measurementChannels:6,measurementScope:'Five noisy pose measurements plus explicitly simulated wheel encoder rate; no true-velocity controller shortcut',timing:{samples:times.length,medianMs:sorted[Math.floor(sorted.length*.5)],p95Ms:sorted[Math.floor(sorted.length*.95)],maxMs:sorted.at(-1),scope:'Node control+plant execution; not browser rendering or hardware worst-case time'}};
 fs.writeFileSync('test-results/live-wasm-parity.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result));
}finally{backend.dispose();}
