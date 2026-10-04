// Bounded fixed experiment, not a new tuner or hardware calibration system.
import fs from 'node:fs';import crypto from 'node:crypto';import {createRequire} from 'node:module';
import {createMujocoBackend} from '../src/mujoco_backend.mjs';
const require=createRequire(import.meta.url),L=require('../src/engine.js'),C=require('../src/calibration_lab.js');
const manifestPath='tests/fixtures/calibration_lab.json',m=JSON.parse(fs.readFileSync(manifestPath,'utf8'));
const sha=p=>crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
if(sha(manifestPath)!=='6300111aae39c0858702d430de7ec04cb03d926be923858bf6d0e85412188698')throw Error('Frozen lesson manifest changed');
const files=[manifestPath,'src/calibration_lab.js','src/engine.js','src/plant.js','src/mujoco_backend.mjs','assets/cartpole.xml','scripts/run_calibration_lesson.mjs'];
const hashes=Object.fromEntries(files.map(p=>[p,sha(p)]));
const b=await createMujocoBackend();L.setPhysicsBackend(b);
try{
 const capture=C.collectStationary(m),fit=C.estimateMeasurementNoise(capture.measurements,m.calibrationScreen),cases=[],full=[];
 if(!fit.usable)throw Error('The fixed acquisition failed its declared calibration screens');
 for(const test of m.cases){
  const runs=m.arms.map(arm=>{const r=C.createRun(m,test,arm,fit);while(!r.done)r.step();return r.result();});
  const replay=C.replayObserver(m,runs[1],m.arms,fit);
  const paired=[1,2].map(i=>({baseline:runs[0].armId,candidate:runs[i].armId,...C.comparePair(runs[0],runs[i])}));
  // Keep the full dynamic record reproducible locally; do not duplicate thousands
  // of per-frame covariance matrices in the committed summary.
  const summaries=runs.map(r=>{const {trace,...summary}=r;return summary;});
  cases.push({id:test.id,group:test.group,runs:summaries,sameDataReplay:replay,paired});full.push({id:test.id,runs});
  console.log(JSON.stringify({id:test.id,arms:runs.map(r=>({arm:r.armId,outcome:r.outcome,steps:r.appliedSteps,track:r.positionTrackingRmse_m,estP:r.estimationRmseByState[0],force:r.rmsForce_N,saturation:r.saturatedSamples}))}));
 }
 const groups=['primary','boundary'].map(group=>{
  const included=cases.filter(c=>c.group===group),allCompleted=included.filter(c=>c.runs.every(r=>r.outcome==='completed'));
  const arms=m.arms.map(arm=>{
   const rr=included.map(c=>c.runs.find(r=>r.armId===arm.id)),both=allCompleted.map(c=>c.runs.find(r=>r.armId===arm.id)),rms=(key)=>both.length?Math.sqrt(both.reduce((s,r)=>s+r[key]**2,0)/both.length):null;
   return {id:arm.id,completed:rr.filter(r=>r.outcome==='completed').length,taskPassed:rr.filter(r=>r.taskPassed).length,requested:rr.length,
    totalSaturatedSamples:rr.reduce((s,r)=>s+r.saturatedSamples,0),executedSamples:rr.reduce((s,r)=>s+r.appliedSteps,0),
    allArmsCompletedTrackingRmse_m:rms('positionTrackingRmse_m'),allArmsCompletedForceRms_N:rms('rmsForce_N'),allArmsCompletedSlewRms_N:rms('rmsCommandSlew_N')};
  });return {group,caseCount:included.length,allArmsCompletedCases:allCompleted.length,excludedFromEqualDurationAggregate:included.length-allCompleted.length,arms};
 });
 const valid=files.every(p=>hashes[p]===sha(p))&&cases.every(c=>c.runs.every(r=>r.outcome!=='execution-error'))&&cases.every(c=>c.sameDataReplay.find(r=>r.armId==='measured').matchedArmMaximumDifference<1e-10);
 const report={schema:'cartpole-calibration-lesson-evidence/v1',manifestSha256:sha(manifestPath),sourceSha256:hashes,physics:b.diagnostics(),capture,fit,groups,cases,experimentValid:valid,
   defaultChanged:false,hardware:'NOT_EVALUATED',claims:['R is estimated from a separate stationary measurement record, not simulator truth variance.','Only R_e differs among arms; Q_e/controller gains/limits/seed/initial state are fixed.','Inflated R is a deliberate wrong-model intervention, not a tuned or calibrated optimum.','No test outcome selects a new global parameter or automatically promotes an arm.','Mean NIS and lag-one are diagnostics, not formal Gaussian/white-noise consistency certification.','Boundary cases retain correlated noise or command delay with the same measurement amplitudes. Covariance magnitude alone cannot identify those mechanisms.']};
 fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/calibration_lesson_full.json',JSON.stringify({capture,fit,cases:full},null,2)+'\n');
 fs.writeFileSync('evidence/calibration_lesson.json',JSON.stringify(report,null,2)+'\n');
 console.log(JSON.stringify({fit,groups,experimentValid:valid},null,2));if(!valid)process.exitCode=1;
}finally{b.dispose();}
