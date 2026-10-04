// Test explicit data separation and likelihood scoring, not an enforced performance win.
import fs from 'node:fs';import assert from 'node:assert/strict';import{createRequire}from'node:module';
import{createMujocoBackend}from'../src/mujoco_backend.mjs';
const require=createRequire(import.meta.url),L=require('../src/engine'),C=require('../src/calibration_lab');
assert.equal(typeof C.innovationScore,'function','Missing normalized innovation likelihood');
assert.equal(typeof C.selectProcessScale,'function','Missing train/validation-only process selection');
const simple=C.innovationScore([1,2],[[2,.3],[.3,4]]);
assert(Number.isFinite(simple.nll)&&simple.nis>0&&Number.isFinite(simple.logdet));
assert(C.innovationScore([0,0],[[1000,0],[0,1000]]).nll>C.innovationScore([0,0],[[1,0],[0,1]]).nll,'logdet must penalize arbitrary covariance inflation');
for(const S of [[[1,2],[2,1]],[[0,0],[0,1]],[[NaN,0],[0,1]]])assert.throws(()=>C.innovationScore([0,0],S));
const m=JSON.parse(fs.readFileSync('tests/fixtures/calibration_lab.json')),study=JSON.parse(fs.readFileSync('tests/fixtures/process_selection.json'));
const b=await createMujocoBackend();L.setPhysicsBackend(b);
try{
 L.LabPlant.prototype.measurementVariance=()=>{throw Error('Simulator covariance oracle forbidden');};
 const cap=C.collectStationary(m),fit=C.estimateMeasurementNoise(cap.measurements,m.calibrationScreen);
 const arm={id:'selected',label:'test',calibrationScale:1,processScale:.01};
 const r=C.createRun(m,study.training[0],arm,fit);for(let k=0;k<80;k++)r.step();const result=r.result();
 assert.deepEqual(result.Qe,m.observer.Q.map(v=>v*.01));
 assert.throws(()=>C.createRun(m,study.training[0],{...arm,processScale:0},fit));
 const record=C.processRecord(result);
 assert.deepEqual(Object.keys(record).sort(),['commands','controlDt','id','information','measurements','seed']);assert.equal(record.measurements.length,record.commands.length+1);
 assert.equal(record.commands.length,80);Object.defineProperty(record,'truth',{get(){throw Error('Forbidden truth access');}});
 const score=C.scoreProcessRecord(m,record,fit,.1,10);
 assert.equal(score.samples,70);assert(score.rows.every(row=>Math.abs(row.nll-.5*(row.nis+row.logdet+2*Math.log(2*Math.PI)))<1e-10));
 const options={scales:[.01,.1,1],trainingShortlist:2,includeBaselineInValidation:true,burnInSteps:10};
 Object.defineProperty(options,'test',{get(){throw Error('Test data must not be accessed by selector');}});
 const training=[{...record,id:'train-fixture',seed:6503}],validation=[{...record,id:'validation-fixture',seed:6607}];
 const sel=await C.selectProcessScale(m,options,training,validation,fit);
 assert(Object.isFrozen(sel));assert(sel.validation.some(v=>v.scale===1));assert.equal(sel.testUsedForSelection,false);
 assert(sel.training.every(v=>v.recordIds.join()==='train-fixture'));assert(sel.validation.every(v=>v.recordIds.join()==='validation-fixture'));
 await assert.rejects(()=>C.selectProcessScale(m,options,training,training,fit),/disjoint|overlap/i);
 const invalid={...record,measurements:record.measurements.slice(1)};assert.throws(()=>C.scoreProcessRecord(m,invalid,fit,1,10),/record|length/i);
 fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/process_contracts.json',JSON.stringify({simple,score,selection:sel,passed:true},null,2));
 console.log('Process selection contracts PASS: logdet, SPD, fixed Q shape, data-only record, timing, training/validation split, immutable selection');
}finally{b.dispose();}
