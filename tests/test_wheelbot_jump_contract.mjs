// Pure profile/input/phase validation; physical success is separately measured.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
assert(fs.existsSync('src/wheelbot_jump.mjs'),'Missing causal motor-driven jump tracker');
const {validateJumpProfile}=await import('../src/wheelbot_jump.mjs');
const profile=JSON.parse(fs.readFileSync('assets/wheelbot/jump_profile.json'));
const xml=fs.readFileSync('assets/wheelbot/wheelbot.xml');
const hash=crypto.createHash('sha256').update(xml).digest('hex');
const backend={assetSha256:hash,limits:[16,16,1.7]};
assert.equal(validateJumpProfile(backend,profile),true);
for(const corrupt of [p=>p.assetSha256='0'.repeat(64),p=>p.u[0][0]=17,p=>p.K[0][0][0]=NaN,p=>p.measurementIndices=[0,1,2,3,6],p=>p.externalForce=[0,1],p=>p.ref.pop(),p=>p.controlDt=.002]){
 const changed=structuredClone(profile);corrupt(changed);assert.throws(()=>validateJumpProfile(backend,changed));
}
console.log('Jump contract PASS: same asset, actual torques,5measurement channels, zero external force, reference dimensions');
const {createJumpTrial}=await import('../src/wheelbot_jump.mjs');
for(const corrupt of [p=>p.target.comApexAboveInitialM=.00001,p=>p.target.positionErrorM=100,p=>p.target.settleSeconds=10,p=>p.envelope.minimumNonadjacentSeparationM=-1]){
 const changed=structuredClone(profile);corrupt(changed);assert.throws(()=>validateJumpProfile(backend,changed),'reject weakened physical acceptance');
}
let calls=0;
const fake={...backend,step(x,u,force){calls++;assert.equal(force,0);return x.slice();},jumpTelemetry(){return {com:[0,0,.4],wheelContacts:1,wheelClearanceM:0,minimumBodyFloorClearanceM:.1,minimumNonadjacentDistanceM:.1};}};
for(const options of [{mode:'true_state'},{seed:-1},{initialState:[0]}])assert.throws(()=>createJumpTrial(fake,profile,options));
assert.equal(calls,0);
const trial=createJumpTrial(fake,profile);const start=trial.snapshot();
assert.deepEqual(start.estimate.slice(5),Array(6).fill(0));assert.equal(start.last,null);assert.equal(start.steps,0);
for(const force of [1,-1,NaN,Infinity,null,'0',{}])assert.throws(()=>trial.step(force));
assert.equal(calls,0);assert.deepEqual(trial.snapshot(),start);
const first=trial.step(0),saved=structuredClone(first);
first.last.u[0]=999;first.estimateIndices[0]=999;first.truth[0]=999;
assert.deepEqual(trial.snapshot(),saved,'returned sample must not mutate trial');
trial.history[0].last.geometry.com[2]=999;trial.history.length=0;
assert.deepEqual(trial.history,[saved],'history must be isolated from callers');
while(!trial.snapshot().done)trial.step();
const count=calls;assert.equal(count,400);
for(let i=0;i<2;i++)assert.throws(()=>trial.step());
assert.equal(calls,count);
console.log('Jump boundary PASS: immutable observations, clean constructor, boost rejection, repeated completion');
