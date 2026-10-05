import assert from 'node:assert/strict';
import fs from 'node:fs';
assert(fs.existsSync('src/wheelbot_disturbance.mjs'),'Missing explicit bounded force task');
const F=await import('../src/wheelbot_disturbance.mjs');
const p=JSON.parse(fs.readFileSync('tests/fixtures/wheelbot_force_envelope.json'));
F.validateForceProtocol(p);
assert.throws(()=>F.validateForceProtocol({...p,assessmentSeeds:[401]}),/disjoint|seed/i);
const make=(amp,pass=true)=>p.controllers.flatMap(mode=>p.screenSeeds.flatMap(seed=>p.directions.map(direction=>({amplitudeN:amp,mode,seed,direction,completed:true,taskPassed:pass,normalPassed:pass,contactLoss:0}))));
const rows=[...make(0),...p.amplitudesN.flatMap(a=>make(a,a<=4))];
const selection=F.selectNormalForce(p,rows);
assert.equal(selection.normalAmplitudeN,3.2);assert.equal(selection.screenUpperN,4);
assert.equal(selection.assessmentUsedForSelection,false);assert(Object.isFrozen(selection));
assert.equal(F.selectNormalForce(p,[...make(0),...p.amplitudesN.flatMap(a=>make(a,a<=2||a===40))]).screenUpperN,2,'cannot jump over a failed amplitude');
assert.equal(F.selectNormalForce(p,[...make(0,false),...rows.filter(x=>x.amplitudeN>0)]).normalAmplitudeN,null);
assert.throws(()=>F.selectNormalForce(p,rows.slice(1)),/coverage|duplicate|screen/i);
assert.throws(()=>F.selectNormalForce(p,[...rows,rows[0]]),/coverage|duplicate|screen/i);
const test=[...p.controllers.flatMap(mode=>p.assessmentSeeds.flatMap(seed=>p.directions.map(direction=>({mode,seed,direction,amplitudeN:3.2,completed:true,normalPassed:true,taskPassed:true,contactLoss:0}))))];
assert(F.admitNormalForce(p,selection,test).accepted);
assert.equal(F.admitNormalForce(p,selection,test.map((r,i)=>i? r:{...r,normalPassed:false})).accepted,false);
assert.equal(F.admitNormalForce(p,selection,test.slice(1)).accepted,false);
assert.throws(()=>F.admitNormalForce(p,{...selection,normalAmplitudeN:2},test),/frozen|selection/i);
console.log('Force protocol, paired all-pass prefix, margin and locked assessment contracts PASS');

// A stored result is checked separately from execution; the CI regenerates it first.
const receipt=JSON.parse(fs.readFileSync('evidence/wheelbot_force_envelope.json'));
const ids={assetSha256:receipt.assetSha256,profileSha256:receipt.profileSha256,protocolSha256:receipt.protocolSha256};
assert.equal(F.validateForceReceipt(p,receipt,ids).amplitudeN,receipt.normal.amplitudeN);
for(const mutate of [r=>{r.profileSha256='0'.repeat(64);},r=>{r.normal.amplitudeN=40;},r=>{r.normal.durationSeconds=.1;},r=>{r.assessment[0].normalPassed=false;}]){
 const bad=structuredClone(receipt);mutate(bad);assert.throws(()=>F.validateForceReceipt(p,bad,ids),/identity|admitted/);
}
console.log('Actual force receipt binding and failed-assessment rejection PASS');
