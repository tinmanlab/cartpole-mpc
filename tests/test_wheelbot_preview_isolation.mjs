import fs from 'node:fs';
import assert from 'node:assert/strict';
import {createWheelbotContactBackend} from '../src/wheelbot_contact_backend.mjs';
import {createWheelbotActions} from '../src/wheelbot_actions.mjs';
const p=JSON.parse(fs.readFileSync('assets/wheelbot/live_profile.json')),a=JSON.parse(fs.readFileSync('assets/wheelbot/pose_profiles.json'));
const b=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));
const t=createWheelbotActions(b,p,a);
const capture=()=>({diagnostics:b.diagnostics(),last:b.lastActionStep(),truth:t.snapshot().truth,estimate:t.snapshot().estimate});
try{
 let before=capture();await t.requestPath([{x:.05,z:p.qref[1]}],{yieldTask:async()=>{}});assert.deepEqual(capture(),before,'Cold preview must not mutate active backend');
 for(let k=0;k<100;k++)t.step();
 let admitted=0;for(const points of [[{x:.05,z:.45}],Array.from({length:10},(_,i)=>({x:i%2?1:-1,z:.45}))]){before=capture();await t.requestPath(points,{yieldTask:async()=>{}});assert.deepEqual(capture(),before);admitted+=Number(t.snapshot().path.available);}assert.equal(admitted,1,'Exercise both acceptance and rejection');
 before=capture();await t.requestPath([{x:.1,z:.45}],{yieldTask:async()=>{t.cancel();}});assert.deepEqual(capture(),before);
 const twinBackend=b.fork();try{const twin=createWheelbotActions(twinBackend,p,a);for(let k=0;k<100;k++)twin.step();const actual=t.step(),expected=twin.step();assert.deepEqual(actual.truth,expected.truth);assert.deepEqual(actual.estimate,expected.estimate);assert.deepEqual(actual.last.measurement,expected.last.measurement);}finally{twinBackend.dispose();}
 console.log('Cold, accepted/rejected and cancelled preview isolation PASS');
}finally{b.dispose();}
