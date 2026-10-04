// The experiment must use the existing plant/filter/controller, not a mock trace.
import fs from 'node:fs';import assert from 'node:assert/strict';import {createRequire} from 'node:module';
import {createMujocoBackend} from '../src/mujoco_backend.mjs';
const require=createRequire(import.meta.url),L=require('../src/engine.js');
let C;try{C=require('../src/calibration_lab.js');}catch(e){if(e.code!=='MODULE_NOT_FOUND')throw e;}
assert(C?.estimateMeasurementNoise,'Missing sensor-data-only calibration lesson');
const m=JSON.parse(fs.readFileSync('tests/fixtures/calibration_lab.json','utf8'));
const b=await createMujocoBackend();L.setPhysicsBackend(b);
try{
 const measurementVariance=L.LabPlant.prototype.measurementVariance;
 L.LabPlant.prototype.measurementVariance=()=>{throw Error('Injected noise oracle must not enter calibration/observer');};
 const capture=C.collectStationary(m),fit=C.estimateMeasurementNoise(capture.measurements,m.calibrationScreen);
 assert(fit.usable);assert.equal(fit.samples,512);assert.equal(fit.biasIdentified,false);assert.equal(fit.source,'stationary-measurements');
 const shifted=capture.measurements.map(y=>[y[0]+10,y[1]-.2]);const fit2=C.estimateMeasurementNoise(shifted,m.calibrationScreen);
 fit.Rdiag.forEach((v,i)=>assert(Math.abs(v-fit2.Rdiag[i])<1e-12,'variance must not use known truth/mean offset'));
 for(const invalid of [[],[[1,2]],Array.from({length:40},()=>[0,0]),[...capture.measurements.slice(0,40),[NaN,0]]])assert.throws(()=>C.estimateMeasurementNoise(invalid,m.calibrationScreen));
 const drift=Array.from({length:512},(_,i)=>[i*.01,i*.001]);assert.equal(C.estimateMeasurementNoise(drift,m.calibrationScreen).usable,false);
 assert.throws(()=>C.createRun(m,m.cases[0],{...m.arms[1],calibrationScale:-1},fit),/positive|covariance/i);
 assert.throws(()=>C.createRun(m,m.cases[0],m.arms[1],{...fit,usable:false}),/calibration/i);
 const runs=m.arms.map(arm=>{
  const r=C.createRun(m,m.cases[0],arm,fit);for(let i=0;i<80;i++)r.step();const out=r.result();
  assert.equal(out.trace.length,80);assert.deepEqual(out.Rdiag,C.armCovariance(arm,fit));assert.equal(out.parameterSource,arm.id==='nominal_assumption'?'explicit-prior-assumption':'stationary-measurements');
  assert(out.trace[0].innovation===null,'no invented initial innovation');
  for(const row of out.trace){
   assert.equal(row.commandTime,row.measurementTime);assert(row.nextTime>row.commandTime);
   assert(Math.abs(row.requestedForce-row.oracleForce-row.estimationForceContributions.reduce((s,v)=>s+v,0))<1e-9);
   assert(Math.abs(row.command)<=10+1e-12);assert(row.truth.every(Number.isFinite));assert(row.estimate.every(Number.isFinite));
  }
  return out;
 });
 const replay=C.replayObserver(m,runs[1],m.arms,fit);
 assert.equal(replay.length,3);assert(replay.every(r=>r.samples===runs[1].trace.length));assert(replay.every(r=>r.controllerApplied===false));
 assert.deepEqual(runs[0].K,runs[1].K);assert.deepEqual(runs[0].Qe,runs[1].Qe);
 L.LabPlant.prototype.measurementVariance=measurementVariance;
 fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/calibration_contracts.json',JSON.stringify({capture,fit,runs,replay,passed:true},null,2)+'\n');
 console.log('Calibration causal contracts PASS: no noise oracle, independent covariance input, honest bias/correlation scope, force decomposition and same-data replay');
}finally{b.dispose();}
