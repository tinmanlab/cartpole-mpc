// One fixed scalar selection study. No test-driven retuning or default promotion.
import fs from 'node:fs';import crypto from 'node:crypto';import{createRequire}from'node:module';
import{createMujocoBackend}from'../src/mujoco_backend.mjs';
const require=createRequire(import.meta.url),L=require('../src/engine'),C=require('../src/calibration_lab');
const sha=p=>crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const path='tests/fixtures/process_selection.json';if(sha(path)!=='96607c7b583b9d1253900a72b0c48e84d0a7e0ed155723072206bbe2135df04a')throw Error('Frozen process-selection manifest changed');
const m=JSON.parse(fs.readFileSync('tests/fixtures/calibration_lab.json')),study=JSON.parse(fs.readFileSync(path));
const sources=[path,'tests/fixtures/calibration_lab.json','src/calibration_lab.js','src/engine.js','src/plant.js','src/mujoco_backend.mjs','assets/cartpole.xml','scripts/run_process_selection.mjs'];
const hashes=Object.fromEntries(sources.map(p=>[p,sha(p)]));const events=[];
const b=await createMujocoBackend();L.setPhysicsBackend(b);
try{
 const full=await C.runProcessStudy(m,study,{onProgress:event=>{events.push(event);console.log(JSON.stringify(event));}});
 const {evaluations,...selection}=full.selection;
 const locked=events.findIndex(e=>e.stage==='selection-locked'),firstTest=events.findIndex(e=>e.stage==='locked-test');
 const valid=locked>=0&&firstTest>locked&&sources.every(p=>hashes[p]===sha(p))&&full.pairs.every(p=>p.sameDataReplay.find(r=>r.armId==='q_baseline').matchedArmMaximumDifference<1e-10);
 const summaryRun=({trace,...r})=>r;
 const report={schema:'cartpole-process-selection/v1',manifestSha256:sha(path),sourceSha256:hashes,physics:b.diagnostics(),fit:full.fit,selection,
  recordMetadata:{training:full.trainingRecords.map(r=>({id:r.id,seed:r.seed,transitions:r.commands.length})),validation:full.validationRecords.map(r=>({id:r.id,seed:r.seed,transitions:r.commands.length})),fields:['id','seed','controlDt','measurements','commands','information']},
  selectionLockedBeforeTest:firstTest>locked,events,groups:full.groups,
  pairs:full.pairs.map(p=>({...p,baseline:summaryRun(p.baseline),candidate:summaryRun(p.candidate)})),
  experimentValid:valid,defaultPromoted:false,hardware:'NOT_EVALUATED',scope:full.scope,
  scoreScope:'Mean prefit Gaussian NLL in fixed m/rad coordinates; approximate EKF likelihood. R/Q shape/P0 remain fixed. Lower NLL does not prove closed-loop improvement.',
  splitScope:'Training produces a shortlist; separate validation selects once; new fixed test seeds evaluated only after the selection lock. Known results become regression data on reuse.'};
 fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/process_selection_full.json',JSON.stringify(full,null,2)+'\n');
 fs.writeFileSync('evidence/process_selection.json',JSON.stringify(report,null,2)+'\n');
 console.log(JSON.stringify({scale:selection.scale,boundary:selection.atGridBoundary,training:selection.training,validation:selection.validation,groups:full.groups,valid},null,2));
 if(!valid)process.exitCode=1;
}finally{b.dispose();}
