// Fetched optional designs must match the actual baseline, not just the model XML.
import fs from 'node:fs';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import * as control from '../src/wheelbot_control.mjs';
assert.equal(typeof control.validateWheelbotResponseProfile,'function','Missing baseline-bound response validation');
const text=fs.readFileSync('assets/wheelbot/profile.json','utf8');
const base=JSON.parse(text),response=JSON.parse(fs.readFileSync('assets/wheelbot/response_profile.json','utf8'));
const hash=crypto.createHash('sha256').update(text).digest('hex');
const backend={assetSha256:base.assetSha256,limits:base.limitsNm};
const validate=p=>control.validateWheelbotResponseProfile(backend,base,p,hash);
assert.equal(validate(response),true);
const cases=[
 p=>{p.responseDesign.sourceProfileSha256='0'.repeat(64);},
 p=>{p.R[0][0]*=2;},
 p=>{p.L[0][0]+=.01;},
 p=>{p.measurementSigma[0]*=2;},
 p=>{p.Q[1][1]*=2;},
 p=>{p.Q[0][0]*=2;},
 p=>{p.responseDesign.factor=0;},
 p=>{p.responseDesign.factor=NaN;},
 p=>{delete p.responseDesign;},
 p=>{p.P=[[1]];},
 p=>{p.unknownModelChange=true;}
];
for(const mutate of cases){const p=structuredClone(response);mutate(p);assert.throws(()=>validate(p),/response|baseline|design|profile|cost|changed|dimension/i);}
assert.throws(()=>control.validateWheelbotResponseProfile(backend,base,response,'not-a-sha'),/baseline|hash/i);
console.log('Response profile binding PASS: raw baseline SHA, single Qx change, unchanged observer/model/noise and finite declared factor');
