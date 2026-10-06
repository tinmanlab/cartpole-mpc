import fs from 'node:fs';
import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import {createWheelbotContactBackend} from '../src/wheelbot_contact_backend.mjs';
import {validateActionJump,actionJumpCommand,actionJumpObserver} from '../src/wheelbot_action_jump.mjs';
const read=p=>JSON.parse(fs.readFileSync(p));
const p=read('assets/wheelbot/action_jump.json'),base=read('assets/wheelbot/live_profile.json'),raw=read('test-results/action-jump-reference.json');
const b=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));
let maxState=0,maxTorque=0,steps=0;
try{
 assert(validateActionJump(b,base,p));const bad=structuredClone(p);bad.assetSha256='0'.repeat(64);assert.throws(()=>validateActionJump(b,base,bad));
 assert.throws(()=>actionJumpCommand(p,base,600,Array(11).fill(0),0));assert.throws(()=>actionJumpCommand(p,base,0,Array(11).fill(NaN),0));
 assert.equal(raw.raw.length,6);assert.equal(raw.report.passed,6);
 for(const test of raw.raw){
  let rng=test.case.seed>>>0;const uniform=()=>{rng=(Math.imul(1664525,rng)+1013904223)>>>0;return(rng+.5)/4294967296;};
  const noise=()=>Math.sqrt(-2*Math.log(uniform()))*Math.cos(2*Math.PI*uniform());
  const measure=x=>p.measurementIndices.map((j,i)=>x[j]+p.measurementSigma[i]*noise());
  let x=test.initial.slice(),estimate=[...measure(x).slice(0,5),0,0,0,0,0,0];const first=b.diagnostics().steps;
  for(let k=0;k<600;k++){
   const action=actionJumpCommand(p,base,k,estimate,test.case.xOffset);x=b.stepWrench(x,action.u,[0,0,0]);estimate=actionJumpObserver(p,base,k,estimate,action.u,measure(x),test.case.xOffset);
   maxState=Math.max(maxState,...x.map((v,j)=>Math.abs(v-test.trace[k].after[j])));maxTorque=Math.max(maxTorque,...action.u.map((v,j)=>Math.abs(v-test.trace[k].u[j])));steps++;
  }
  assert.equal(b.diagnostics().steps-first,3000);
 }
 assert(maxState<1e-6&&maxTorque<1e-6,{maxState,maxTorque});
 const report={passed:true,trials:6,controlSteps:steps,physicsSteps:steps*5,maxState,maxTorque,profileSha256:crypto.createHash('sha256').update(fs.readFileSync('assets/wheelbot/action_jump.json')).digest('hex'),modelSha256:b.assetSha256,scope:'Actual same-model WASM sensor-based jump and landing parity; not arbitrary entry/fall/hardware validation.'};
 fs.writeFileSync('test-results/action-jump-wasm.json',JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report));
}finally{b.dispose();}
