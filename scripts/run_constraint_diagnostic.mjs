// F2: a small fixed diagnostic using existing controllers, observers and real WASM.
// Exact-state plans are explicitly offline diagnostics, never sensor-only evidence.
import fs from 'node:fs';import crypto from 'node:crypto';import {createRequire} from 'node:module';
import {createMujocoBackend} from '../src/mujoco_backend.mjs';
const require=createRequire(import.meta.url),L=require('../src/engine'),D=require('../src/design_study');
const protocolPath='tests/fixtures/constraint_diagnostic.json',protocol=JSON.parse(fs.readFileSync(protocolPath));
const m=JSON.parse(fs.readFileSync(protocol.baseTask));D.validateManifest(m);
if(protocol.steps!==m.steps||protocol.cases.length!==5)throw Error('Unexpected frozen diagnostic scope');
const files=[protocolPath,protocol.baseTask,'scripts/run_constraint_diagnostic.mjs','src/design_study.js','src/calibration_lab.js','src/engine.js','src/qp.js','src/plant.js','src/mujoco_backend.mjs','assets/cartpole.xml'];
const sha=p=>crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const sourceSha256=Object.fromEntries(files.map(p=>[p,sha(p)]));
const b=await createMujocoBackend();L.setPhysicsBackend(b);
const errors=(x,g)=>[x[0]-g,x[1],x[2],x[3]];
const world=(x,g)=>[x[0]+g,x[1],x[2],x[3]];
function plan(m,test,pair,baseline,fit,spec){
 const d=D.design(m,pair,baseline,fit),{controller}=D.buildControllers(m,pair,d,spec);controller.reset();
 if(pair.controller==='hard_mpc'){
  try{
   controller.act(test.initialState,test.goal);const X=controller.lastPrediction.map(x=>x.slice()),U=controller.lastControls.slice();
   return {accepted:true,U,X,J:controller.lastCost,kktResidual:controller.lastKktResidual,primalResidual:controller.lastPrimalResidual,
    railActive:X.some(x=>Math.abs(x[0])>=m.task.railLimit_m-1e-6),maxWorldPosition:Math.max(...X.map(x=>Math.abs(x[0])))};
  }catch(e){if(!String(e.message).startsWith('QP '))throw e;return {accepted:false,reason:String(e.message),railActive:false};}
 }
 // This is only a rollout of clipped LQR on the existing nominal linear model.
 let x=test.initialState.slice();const X=[x.slice()],U=[];
 for(let k=0;k<d.horizon;k++){
  const u=controller.act(x,test.goal);U.push(u);const next=L.mv(d.A,errors(x,test.goal)).map((v,i)=>v+d.B[i][0]*u);
  x=world(next,test.goal);X.push(x.slice());
 }
 return {U,X,J:L.linearQuadraticCost(X.map(x=>errors(x,test.goal)),U,L.diag(d.Qc),d.Rc,d.Pf),maxWorldPosition:Math.max(...X.map(x=>Math.abs(x[0]))),scope:'nominal closed-loop clipped LQR prediction, not a physical rollout'};
}
try{
 L.LabPlant.prototype.measurementVariance=()=>{throw Error('Injected covariance oracle forbidden');};
 const fit=D.calibrate(m).fit,baseline=D.candidates(m).find(c=>c.baseline),spec=new L.LabPlant().spec;
 const probes=[],runs=[];
 for(const test of protocol.cases){
  if(test.initialState.length!==4||!test.initialState.every(Number.isFinite)||Math.abs(test.initialState[0])>m.task.railLimit_m)throw Error('Invalid initial state');
  const lpair=m.pairs.find(p=>p.id==='lqr-kf'),mpair=m.pairs.find(p=>p.id==='mpc-kf');
  probes.push({caseId:test.id,initialState:test.initialState.slice(),goal:test.goal,design:D.design(m,mpair,baseline,fit),information:'exact-state model-only diagnostic, not fed into the sensor-only runs',lqr:plan(m,test,lpair,baseline,fit,spec),mpc:plan(m,test,mpair,baseline,fit,spec)});
  for(const pair of m.pairs){
   const run=D.createRun(m,test,pair,baseline,fit,{record:true});const initial=run.snapshot();
   // reset uses sensor p/theta and zeros for velocities. Do not draw a second sensor
   // sample (which would change RNG history), and do not initialize from true velocity.
   const initialMeasurement=[initial.estimate[0],initial.estimate[2]];
   while(!run.done)run.step();const result=run.result();
   const residual=[0,0,0,0];
   for(const row of result.trace){
    const predicted=world(L.mv(result.design.A,errors(row.truth,row.goal)).map((v,i)=>v+result.design.B[i][0]*row.command),row.goal);
    row.nextTruth.forEach((v,i)=>{const error=i===2?L.wrap(v-predicted[i]):v-predicted[i];residual[i]=Math.max(residual[i],Math.abs(error));});
   }
   runs.push({...result,initialEstimate:initial.estimate,initialMeasurement,
    maxOneStepPredictionResidual:residual,predictionResidualUnits:['m','m/s','rad','rad/s'],
    residualScope:'true-state evaluation of nominal commanded-input prediction; combines linearization, model and input-realization mismatch, not observer error',
    actualRailViolated:result.trace.some(q=>Math.abs(q.nextTruth[0])>m.task.railLimit_m),
    initialEstimatedOutsideRail:Math.abs(initial.estimate[0])>m.task.railLimit_m});
  }
 }
 const valid=files.every(p=>sha(p)===sourceSha256[p])&&runs.every(r=>r.outcome!=='execution-error');
 const summary={schema:'cartpole-constraint-diagnostic-summary/v1',protocolSha256:sha(protocolPath),
  counts:{cases:protocol.cases.length,sensorBasedRuns:runs.length,completed:runs.filter(r=>r.outcome==='completed').length,taskPassed:runs.filter(r=>r.taskPassed).length,
   exactStatePlansAccepted:probes.filter(p=>p.mpc.accepted).length,exactStatePlansRailActive:probes.filter(p=>p.mpc.accepted&&p.mpc.railActive).length,
   sensorMpcRailActiveSamples:runs.filter(r=>r.pairId.startsWith('mpc')).reduce((s,r)=>s+r.predictedRailActiveSamples,0)},
  probes:probes.map(p=>({caseId:p.caseId,lqrMaxNominalPosition:p.lqr.maxWorldPosition,mpcAccepted:p.mpc.accepted,mpcRailActive:p.mpc.railActive,mpcMaxNominalPosition:p.mpc.maxWorldPosition??null,reason:p.mpc.reason??null})),
  runs:runs.map(r=>({caseId:r.caseId,pairId:r.pairId,outcome:r.outcome,reason:r.reason,steps:r.appliedSteps,taskPassed:r.taskPassed,score:r.fullScore,trackingRmse:r.positionTrackingRmse,
   predictedRailActiveSamples:r.predictedRailActiveSamples,actualRailViolated:r.actualRailViolated,initialEstimatedOutsideRail:r.initialEstimatedOutsideRail,initialEstimate:r.initialEstimate,
   maxOneStepPredictionResidual:r.maxOneStepPredictionResidual})),
  defaultsChanged:false,claimBoundary:protocol.claimBoundary};
 const report={schema:'cartpole-constraint-diagnostic/v1',sourceSha256,protocolSha256:sha(protocolPath),physics:b.diagnostics(),fit,probes,runs,summary,experimentValid:valid,defaultsChanged:false};
 fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/constraint_diagnostic_full.json',JSON.stringify(report)+'\n');
 console.log('CONSTRAINT_DIAGNOSTIC_COUNTS '+JSON.stringify(summary.counts));
 if(!valid)throw Error('Diagnostic implementation/source identity failed; this is not a controller task failure');
}finally{b.dispose();}
