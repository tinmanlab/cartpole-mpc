// Independent verification probe. No production source/defaults are changed.
import fs from 'node:fs';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {createMujocoBackend} from '../src/mujoco_backend.mjs';
const L=createRequire(import.meta.url)('../src/engine.js');
const sha=p=>crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const numericalFiles=['src/engine.js','src/plant.js','src/qp.js','src/mujoco_backend.mjs','src/bam_params.js','assets/cartpole.xml','vendor/manifest.json','package-lock.json'];
const sourceHashes=()=>Object.fromEntries(numericalFiles.map(p=>[p,sha(p)]));
const backend=await createMujocoBackend();L.setPhysicsBackend(backend);
try{
 const manifestPath='tests/fixtures/terminal_validation.json';
 const manifest=JSON.parse(fs.readFileSync(manifestPath,'utf8'));
 assert.equal(sha(manifestPath),'23594cd723f706ddc881bc84717b5d28cc17370508502a11debcd7662b847742');
  const ref=JSON.parse(fs.readFileSync('evidence/terminal_reference.json','utf8'));
  assert.equal(ref.passed,true);assert.equal(ref.fixtureSha256,sha('test-results/terminal_fixture.json'));
  const before=sourceHashes(),startAt=new Date().toISOString(),pairs=[];
  const clock=()=>performance.now();
  const mean=x=>x.length?x.reduce((s,v)=>s+v,0)/x.length:null;
  function dist(values){if(!values.length)return null;const a=values.slice().sort((x,y)=>x-y);return {samples:a.length,p50:a[Math.ceil(.5*a.length)-1],p95:a[Math.ceil(.95*a.length)-1],p99:a[Math.ceil(.99*a.length)-1],max:a.at(-1)};}
  function run(test,terminal){
   const plant=new L.LabPlant({scenario:test.scenario,seed:test.seed});plant.reset(test.initialState);
   const ctl=L.makeController('hard_mpc',plant.spec,null,{terminalCost:terminal}),obs=L.makeObserver('ekf',plant.spec,{R:plant.measurementVariance()});
   if(terminal==='dare'){
    assert.equal(ctl.terminalInfo.kind,'dare');assert(ctl.terminalInfo.converged);assert(ctl.terminalInfo.normalizedResidual<=manifest.numericalTolerances.normalizedDareResidual);
    const p=ref.cases[0].scipyP;assert(Math.max(...ctl.Qf.flatMap((row,i)=>row.map((v,j)=>Math.abs(v-p[i][j]))))<=manifest.numericalTolerances.matrixMaxAbsoluteError);
   }
   const y0=plant.sensor();obs.reset(y0,plant.s);ctl.reset();let estimate=obs.x.slice(),goal=test.goal;
   const squared=[],angles=[],inputs=[],slews=[],solveTimes=[],computeTimes=[],tail=[],lastSamples=[];
   let outcome='completed',reason=null,maxPos=0,maxAngle=0,maxForce=0,maxKkt=0,maxPrimal=0,stale=0,used=0,lastU=0;
   for(let k=0;k<test.steps;k++){
    if(test.goalChange&&k===test.goalChange.step)goal=test.goalChange.value;
    const start=clock();
    try{
     const solveStart=clock(),u=ctl.act(estimate,goal);solveTimes.push(clock()-solveStart);
     assert(Number.isFinite(u)&&Math.abs(u)<=10+1e-8,'Invalid input accepted');
     assert(ctl.lastConverged&&ctl.lastKktResidual<=manifest.numericalTolerances.kktResidual&&ctl.lastPrimalResidual<=manifest.numericalTolerances.primalResidual,'Numerical acceptance gate violated');
     maxKkt=Math.max(maxKkt,ctl.lastKktResidual);maxPrimal=Math.max(maxPrimal,ctl.lastPrimalResidual);
     if(k===test.push.step)plant.applyPush(test.push.force,test.push.duration);
     const out=plant.step(u),y=plant.sensor();estimate=obs.step(plant.sensorMeta.fresh?y:null,u,out.state);
     assert(estimate.every(Number.isFinite),'Nonfinite estimate');
     plant.sensorMeta.fresh?used++:stale++;squared.push((out.state[0]-goal)**2);angles.push(out.state[2]**2);inputs.push(u*u);slews.push((u-lastU)**2);lastU=u;
     maxPos=Math.max(maxPos,Math.abs(out.state[0]));maxAngle=Math.max(maxAngle,Math.abs(out.state[2]));maxForce=Math.max(maxForce,Math.abs(u));
     tail.push(Math.abs(out.state[0]-goal));if(tail.length>100)tail.shift();
     lastSamples.push({step:k+1,truth:out.state.slice(),estimate:estimate.slice(),goal,u,applied:out.appliedCommand,fresh:plant.sensorMeta.fresh});if(lastSamples.length>5)lastSamples.shift();
     computeTimes.push(clock()-start);
     if(out.failed){outcome='envelope-failure';reason=Math.abs(out.state[0])>2.4?'rail':'angle';break;}
    }catch(e){computeTimes.push(clock()-start);outcome=String(e.message).startsWith('QP rejected:')?'solver-rejected':(['QP acceptance residual exceeded','QP returned invalid solution'].includes(String(e.message))?'numerical-plan-rejected':'execution-error');reason=String(e.message);break;}
   }
   const appliedSteps=squared.length;
   return {terminal,terminalInfo:ctl.terminalInfo,outcome,reason,appliedSteps,requestedSteps:test.steps,trackingRmse_m:appliedSteps?Math.sqrt(mean(squared)):null,angleRms_rad:appliedSteps?Math.sqrt(mean(angles)):null,rmsForce_N:appliedSteps?Math.sqrt(mean(inputs)):null,rmsDeltaForce_N:appliedSteps?Math.sqrt(mean(slews)):null,finalWindowMeanAbsoluteError_m:mean(tail),finalState:plant.s.slice(),finalEstimate:estimate,finalGoal:goal,maxPosition_m:maxPos,maxAngle_rad:maxAngle,maxForce_N:maxForce,maxKktResidual:maxKkt,maxPrimalResidual:maxPrimal,staleSamples:stale,measurementUpdates:used,solveTimeMs:dist(solveTimes),synchronousStepMs:dist(computeTimes),measurementCovarianceSource:'injected-noise-oracle',lastSamples,squared};
  }
  for(const c of manifest.cases){
   const old=run(c,'original'),next=run(c,'dare'),n=Math.min(old.appliedSteps,next.appliedSteps);
   const prefix={steps:n,originalMse:n?mean(old.squared.slice(0,n)):null,dareMse:n?mean(next.squared.slice(0,n)):null};
   delete old.squared;delete next.squared;
   const p={id:c.id,set:c.set,scenario:c.scenario,seed:c.seed,original:old,dare:next,commonPrefix:prefix};pairs.push(p);
   fs.writeFileSync('test-results/terminal-progress.json',JSON.stringify({stage:'validation',completedPairs:pairs.length,totalPairs:manifest.cases.length,last:{id:c.id,original:old.outcome,dare:next.outcome}},null,2));
   console.log(JSON.stringify({id:c.id,old:old.outcome,candidate:next.outcome,steps:[old.appliedSteps,next.appliedSteps]}));
  }
  const summary={};for(const set of ['primary','boundary']){
   const pp=pairs.filter(p=>p.set===set),both=pp.filter(p=>p.original.outcome==='completed'&&p.dare.outcome==='completed');
   const only=pp.filter(p=>p.original.outcome==='completed'&&p.dare.outcome!=='completed');
   const fixes=pp.filter(p=>p.original.outcome!=='completed'&&p.dare.outcome==='completed');
   const counts=key=>Object.fromEntries(['completed','envelope-failure','solver-rejected','numerical-plan-rejected','execution-error'].map(o=>[o,pp.filter(p=>p[key].outcome===o).length]));
   const orig=both.reduce((s,p)=>s+p.original.trackingRmse_m**2*p.original.appliedSteps,0),cand=both.reduce((s,p)=>s+p.dare.trackingRmse_m**2*p.dare.appliedSteps,0);
   summary[set]={pairs:pp.length,original:counts('original'),dare:counts('dare'),candidateOnlyFailureIds:only.map(p=>p.id),resolvedBaselineFailureIds:fixes.map(p=>p.id),bothCompletedPairs:both.length,excludedFromFullDurationComparison:pp.length-both.length,bothCompletedTrackingMseRatio:both.length&&orig>0?cand/orig:null};
  }
  const noErrors=pairs.every(p=>p.original.outcome!=='execution-error'&&p.dare.outcome!=='execution-error');
  const sourcesStable=JSON.stringify(before)===JSON.stringify(sourceHashes());
  const primaryEligible=noErrors&&sourcesStable&&summary.primary.candidateOnlyFailureIds.length===0&&summary.primary.bothCompletedPairs>=manifest.admission.minimumBothCompletedPairs&&summary.primary.bothCompletedTrackingMseRatio<=manifest.admission.bothCompletedTrackingMseRatioMax;
  const blanketEligible=primaryEligible&&summary.boundary.candidateOnlyFailureIds.length===0;
  const report={schema:'cartpole-terminal-prospective-validation/v3',implementation:'Public constructor terminalCost option; no direct test-only Qf assignment',classificationNote:'Existing QP acceptance rejections count as numerical-plan-rejected, never successful trials; frozen manifest and tolerances are unchanged.',baseCommit:manifest.baseCommit,startAt,completedAt:new Date().toISOString(),manifestSha256:sha(manifestPath),probeSha256:sha('scripts/validate_terminal.mjs'),referenceSha256:sha('evidence/terminal_reference.json'),sourceSha256:before,sourcesStable,physics:backend.diagnostics(),summary,pairs,validExperiment:noErrors&&sourcesStable,primaryEmpiricalEligibility:primaryEligible,predeclaredDefaultScreenPassed:blanketEligible,browserOptionImplemented:true,browserDefaultChanged:false,hardwareAdmission:'NOT_EVALUATED',scope:'Prospective fixed simulation validation; zero parameter tuning; not rare-failure certification. Failed candidates and unequal-duration runs remain explicit.'};
  fs.writeFileSync('evidence/terminal_validation.json',JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify({validExperiment:report.validExperiment,summary,primaryEmpiricalEligibility:primaryEligible,predeclaredDefaultScreenPassed:blanketEligible},null,2));
  if(!report.validExperiment)process.exitCode=1;
}finally{backend.dispose();}
