import assert from 'node:assert/strict';
import fs from 'node:fs';
import load from '../vendor/mujoco/mujoco.js';
import {createTargetJump,admitOptionalTargetJump} from '../src/wheelbot_target_jump.mjs';
const read=p=>JSON.parse(fs.readFileSync(p));
const version=read('package.json').dependencies['@mujoco/mujoco'];
const mj=await load();
assert.equal(mj.mj_versionString(),version);
assert.equal(read('vendor/manifest.json').packages['@mujoco/mujoco'],version);
assert(fs.readFileSync('requirements-native.txt','utf8').split('\n').includes('mujoco=='+version));
const base=read('assets/wheelbot/live_profile.json'),bundle=read('assets/wheelbot/target_jump.json');
const backend={assetSha256:base.assetSha256,limits:base.limitsNm,diagnostics:()=>({version})};
assert.throws(()=>createTargetJump(backend,base,{...bundle,nativeVersion:'mismatch'},{diagnostic:true}),/version/i);
console.log('Actual WASM, manifest, pins and jump version disagreement PASS',version);

for(const bad of [null,{...bundle,releaseValidated:false},{...bundle,nativeVersion:'mismatch'}]){const admission=admitOptionalTargetJump(backend,base,bad,bundle.baselineSha256);assert.equal(admission.planner,null);assert.equal(typeof admission.reason,'string');}
if(process.argv.includes('--require-latest'))assert.equal(version,'3.15.0','Latest engine upgrade is incomplete');

const {createWheelbotContactBackend}=await import('../src/wheelbot_contact_backend.mjs');
const {createWheelbotActions}=await import('../src/wheelbot_actions.mjs');
const plant=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));
try{const trial=createWheelbotActions(plant,base,read('assets/wheelbot/pose_profiles.json'));for(let k=0;k<30;k++)trial.step();const before=trial.snapshot();await trial.requestPath([{x:.1,z:.1}],{mode:'wheel',action:'jump',jumpUnavailableReason:'version mismatch'});const after=trial.snapshot();assert.deepEqual(after.truth,before.truth);assert.deepEqual(after.estimate,before.estimate);assert.equal(after.steps,before.steps);assert.match(after.actionStatus,/version mismatch/);assert(!trial.step().failed);console.log('Optional invalid jump preserves plant/observer and balance PASS');}finally{plant.dispose();}
