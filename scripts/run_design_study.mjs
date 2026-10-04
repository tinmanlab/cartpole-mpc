// Frozen task-aware design study: fair candidate budget, no test-led reselection.
import fs from 'node:fs';import crypto from 'node:crypto';import{createRequire}from'node:module';
import{createMujocoBackend}from'../src/mujoco_backend.mjs';
const require=createRequire(import.meta.url),L=require('../src/engine'),D=require('../src/design_study');
const sha=p=>crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const manifestPath='tests/fixtures/design_study.json';
if(sha(manifestPath)!=='1cc2de2d8d72fa151a5b5e5aa9ef2659c5f9783d7aff6ea2d56ea791b5e79445')throw Error('Frozen common-task manifest changed');
const m=JSON.parse(fs.readFileSync(manifestPath));
const files=[manifestPath,'src/design_study.js','src/calibration_lab.js','src/engine.js','src/plant.js','src/qp.js','src/mujoco_backend.mjs','assets/cartpole.xml','scripts/run_design_study.mjs'];
const before=Object.fromEntries(files.map(p=>[p,sha(p)])),events=[],started=new Date().toISOString();
fs.mkdirSync('test-results',{recursive:true});const b=await createMujocoBackend();L.setPhysicsBackend(b);
try{
 L.LabPlant.prototype.measurementVariance=()=>{throw Error('Injected covariance oracle is forbidden in common-task design');};
 const full=await D.runStudy(m,{onProgress:e=>{events.push(e);fs.writeFileSync('test-results/design-progress.json',JSON.stringify(e));console.log(JSON.stringify(e));}});
 const lock=events.findIndex(e=>e.stage==='selection-locked'),test=events.findIndex(e=>e.stage==='locked-test');
 const valid=files.every(p=>sha(p)===before[p])&&lock>=0&&test>lock&&full.auditRuns.every(r=>r.outcome!=='execution-error')&&full.selection.trainingEvaluations===108;
 const report={schema:'cartpole-task-design-study/v1',baseCommit:'1c9056726761cb5374754e77448caca1f4d1ab31',started,completed:new Date().toISOString(),manifestSha256:sha(manifestPath),sourceSha256:before,physics:b.diagnostics(),
 fit:full.capture.fit,selection:full.selection,summary:full.summary,admission:full.admission,selectionLockedBeforeTest:test>lock,events,
 tests:full.tests.map(t=>({...t,baseline:D.withoutSeries(t.baseline),candidate:D.withoutSeries(t.candidate)})),
 trialCounts:{training:full.selection.trainingEvaluations,validation:full.selection.validationEvaluations,test:full.tests.length*2,total:full.auditRuns.length},experimentValid:valid,defaultsChanged:false,hardwareAdmission:'NOT_EVALUATED',
 scope:'Four existing pairs and a fixed 9-candidate grid each. Same physical task, sensor assumptions and score; validation recommends before test. Identical trial budget is not a universal fair benchmark or global optimum. Construction/runtime timing is reported but not used to select under variable host load.'};
 fs.writeFileSync('test-results/design_study_full.json',JSON.stringify(full)+'\n');
 fs.writeFileSync('evidence/design_study.json',JSON.stringify(report,null,2)+'\n');
 console.log(JSON.stringify({trialCounts:report.trialCounts,pairs:full.selection.pairs.map(p=>({id:p.pairId,candidate:p.candidate,eligible:p.eligible,validationScore:p.validationScore})),recommendation:full.selection.recommendedPairId,admission:full.admission,summary:full.summary,valid},null,2));
 if(!valid)process.exitCode=1;
}finally{b.dispose();}
