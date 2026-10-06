import fs from 'node:fs';
import assert from 'node:assert/strict';
import * as jump from '../src/wheelbot_action_jump.mjs';
import {pipelineLabels} from '../src/wheelbot_live_view.mjs';
const read=p=>JSON.parse(fs.readFileSync(p,'utf8'));
const base=read('assets/wheelbot/live_profile.json'),profile=read('assets/wheelbot/action_jump.json');
const backend={assetSha256:base.assetSha256,limits:base.limitsNm};
assert.equal(typeof jump.admitOptionalJump,'function','Optional jump data must not disable ordinary pose control');
assert.equal(jump.admitOptionalJump(backend,base,profile).profile,profile);
for(const bad of [null,{...profile,steps:1},{...profile,assetSha256:'wrong'}]){
 const admitted=jump.admitOptionalJump(backend,base,bad);
 assert.equal(admitted.profile,null);assert.equal(typeof admitted.reason,'string');assert(admitted.reason.length>0);
}
assert.deepEqual(pipelineLabels({liveActive:true,actions:true,failed:true,mode:'lqr_kf'}),['Simulator state display','Observer invalid after fall','Motors disengaged','MuJoCo full contact']);
console.log('Optional jump admission and invalidated fallen observer labels PASS');
