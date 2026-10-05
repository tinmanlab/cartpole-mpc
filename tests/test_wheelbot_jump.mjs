// Actual WASM motion, not animation. Independent native checks consume raw outputs.
import fs from 'node:fs';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import {createWheelbotBackend} from '../src/wheelbot_backend.mjs';
import {createJumpTrial,jumpMetrics} from '../src/wheelbot_jump.mjs';
const protocol=JSON.parse(fs.readFileSync('tests/fixtures/wheelbot_jump.json'));
const path=process.argv[2]??'assets/wheelbot/jump_profile.json';
const p=JSON.parse(fs.readFileSync(path));
const xml=fs.readFileSync('assets/wheelbot/wheelbot.xml','utf8');
const b=await createWheelbotBackend(xml),rows=[],raw=[];
try{
 assert.deepEqual(p.target,protocol.target);assert.deepEqual(p.envelope,protocol.envelope);
 assert.equal(p.steps*p.controlDt,protocol.durationSeconds);assert.deepEqual(p.limitsNm,protocol.actuatorLimitsNm);
 assert.equal(p.protocolSha256,crypto.createHash('sha256').update(fs.readFileSync('tests/fixtures/wheelbot_jump.json')).digest('hex'));
 assert.equal(b.assetSha256,p.assetSha256);
 const state=p.ref[0],zero=b.step(state,[0,0,0],0);
 b.step(state,[0,0,0],1);assert.deepEqual(b.step(state,[0,0,0],0),zero,'External force must expire when command is zero');
 const rejectedAt=b.diagnostics().steps;
 for(const [u,force]of [[[NaN,0,0],0],[[0,0],0],[[0,0,0],NaN],[[0,0,0],Infinity]])assert.throws(()=>b.step(state,u,force));
 assert.equal(b.diagnostics().steps,rejectedAt,'Invalid commands must not advance physics');
 const before=b.diagnostics().steps;const initial=b.jumpTelemetry(p.ref[0]);assert.equal(b.diagnostics().steps,before,'telemetry must not integrate');
 for(const [mode,seeds]of [['nominal_torque',[0]],['tvlqr_kf',protocol.assessmentSeeds]])for(const seed of seeds){
  const trial=createJumpTrial(b,p,{mode,seed});let current=trial.snapshot();
  while(!current.done)current=trial.step();
  const result=jumpMetrics(p,trial.history,initial);
  assert.equal(result.steps,trial.history.length);assert(result.externalForceIsZero);
  rows.push({mode,seed,...result});raw.push({mode,seed,initial:trial.history.length?p.ref[0]:null,initialTelemetry:initial,result,trace:trial.history});
 }
 const summary={schema:'wheelbot-jump-validation/v1',assetSha256:b.assetSha256,profileSha256:crypto.createHash('sha256').update(fs.readFileSync(path)).digest('hex'),protocolSha256:crypto.createHash('sha256').update(fs.readFileSync('tests/fixtures/wheelbot_jump.json')).digest('hex'),rows,scope:'Nominal torque replay is a physical feedforward demonstration, not state-feedback robustness. TVLQR/KF trial is separately evaluated with5noisypositions; failed cases stay failures.'};
 const prefix=process.argv[3]??'test-results/jump_wasm';
 fs.writeFileSync(prefix+'_summary.json',JSON.stringify(summary,null,2)+'\n');fs.writeFileSync(prefix+'_raw.json',JSON.stringify({profilePath:path,...summary,raw})+'\n');console.log(JSON.stringify(rows,null,2));
 assert(rows.filter(r=>r.mode==='tvlqr_kf').every(r=>r.passed&&r.steps===400),'All assessment seeds must pass');
}finally{b.dispose();}
