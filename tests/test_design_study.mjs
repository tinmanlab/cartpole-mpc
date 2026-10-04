// One task, fair bounded candidate budget, explicit information/selection boundaries.
import fs from 'node:fs';import assert from 'node:assert/strict';import{createRequire}from'node:module';
import{createMujocoBackend}from'../src/mujoco_backend.mjs';
assert(fs.existsSync('src/design_study.js'),'Missing task-aware four-pair design implementation');
const require=createRequire(import.meta.url),L=require('../src/engine'),D=require('../src/design_study');
const m=JSON.parse(fs.readFileSync('tests/fixtures/design_study.json'));
D.validateManifest(m);assert.equal(D.candidates(m).length,9);
assert.throws(()=>D.validateManifest({...m,test:[m.training[0]]}),/disjoint|group|test/i);
const parts=D.stageCost(m,{truth:[.25,0,.1,0],goal:0,u:5,previousU:3});
assert.deepEqual(parts,{position:1,angle:.2,force:.05,deltaForce:.02,total:1.27});
assert.throws(()=>D.stageCost(m,{truth:[NaN,0,0,0],goal:0,u:0,previousU:0}),/finite|state/);
const rankBase={candidate:{id:'base',baseline:true},hardFailures:0,taskFailures:0,fullScore:1};
const rankFail={candidate:{id:'failed'},hardFailures:1,taskFailures:1,fullScore:0};assert(D.rankCandidates(rankBase,rankFail)<0);
assert(D.rankCandidates({...rankBase,candidate:{id:'x',baseline:false}},rankBase)>0,'exact tie must prefer declared baseline');
const b=await createMujocoBackend();L.setPhysicsBackend(b);
try{
 L.LabPlant.prototype.measurementVariance=()=>{throw Error('Injected noise oracle is forbidden');};
 const acquisition=D.calibrate(m),fit=acquisition.fit;assert(fit.usable&&fit.source==='stationary-measurements');
 const options=D.candidates(m),baseline=options.find(c=>c.baseline);let fixture=[];
 for(const pair of m.pairs){
  const design=D.design(m,pair,baseline,fit);assert(design.terminalInfo.converged&&design.terminalInfo.normalizedResidual<=1e-9);
  assert.deepEqual(design.Qc,m.theoryDesign.stateScales.map(v=>1/v**2));assert.equal(design.Rc,1/9);
  assert.deepEqual(design.Qe,m.theoryDesign.QeBase);assert.deepEqual(design.P0,m.theoryDesign.P0diag);
  const run=D.createRun(m,m.training[0],pair,baseline,fit,{record:true});
  while(!run.done&&run.snapshot().steps<20)run.step();const r=run.result();assert.equal(r.appliedSteps,20);assert.equal(r.fullScore,null,'partial replay is not full task success');
  assert.equal(r.trace[0].measurementTime,0);assert.equal(r.trace[0].nextTime,.02);
  assert.deepEqual(r.trace[0].estimate,[r.trace[0].measurement[0],0,r.trace[0].measurement[1],0]);assert.deepEqual(r.trace[0].P,L.diag(m.theoryDesign.P0diag));
  assert(r.trace.every(q=>Math.abs(q.command)<=10+1e-8));assert.equal(r.information.controller,'observer-output only');
  fixture.push({pair,design,first:r.trace[0],trajectory:r.auditSeries});
 }
 const Qc=fixture[0].design.Qc,Rc=fixture[0].design.Rc;for(const f of fixture){assert.deepEqual(f.design.Qc,Qc);assert.equal(f.design.Rc,Rc);assert.deepEqual(f.design.Re,fixture[0].design.Re);}
 const fitClone=JSON.parse(JSON.stringify(fit));Object.defineProperty(fitClone,'truth',{get(){throw Error('Forbidden truth access');}});D.design(m,m.pairs[0],baseline,fitClone);
 assert.throws(()=>D.design(m,{...m.pairs[0],observer:'truth'},baseline,fit),/pair|unsupported/i);
 assert.throws(()=>D.design(m,m.pairs[0],{...baseline,processMultiplier:0},fit),/candidate|positive/i);
 // The selector API accepts train and validation only; it must not touch the test getter.
 const optionsOnly={...m};delete optionsOnly.test;Object.defineProperty(optionsOnly,'test',{get(){throw Error('Selector read test before lock');}});
 const fake=(pair,candidate,cases)=>cases.map(c=>({caseId:c.id,requestedSteps:600,appliedSteps:600,outcome:'completed',taskPassed:true,fullScore:candidate.baseline?1:2}));
 const selected=await D.select(optionsOnly,fit,{evaluate:fake});assert(Object.isFrozen(selected));assert.equal(selected.testUsedForSelection,false);
 assert(selected.pairs.every(p=>p.candidate.baseline));assert.equal(selected.recommendedPairId,'lqr-kf','policy breaks exact cross-pair tie, not a universal optimum');
 assert.equal(selected.trainingEvaluations,4*9*3);assert(selected.pairs.every(p=>p.validation.length>=2&&p.validation.length<=3));
 await assert.rejects(()=>D.select({...optionsOnly,validation:optionsOnly.training},fit,{evaluate:fake}),/disjoint|overlap/i);
 const badRun=D.createRun(m,{...m.training[0],initialState:[2.5,0,0,0]},m.pairs[2],baseline,fit);badRun.step();const bad=badRun.result();
 assert.equal(bad.outcome,'solver-rejected');assert.equal(bad.appliedSteps,0);assert.equal(bad.fullScore,null);
 const rejected=D.adjudicate(m,selected,[{pairId:'lqr-kf',group:'primary',baseline:{outcome:'completed',taskPassed:true,fullScore:1},candidate:{outcome:'completed',taskPassed:false,fullScore:.1}}]);
 assert.equal(rejected.accepted,false);assert.equal(rejected.recommendedPairId,'lqr-kf','test may reject but never reselect');
 assert.throws(()=>D.application(m,selected,rejected,m.test[0],fit),/rejected|admitted/i);
 const allPrimary=m.test.filter(t=>t.group==='primary').map(t=>({caseId:t.id,pairId:selected.recommendedPairId,group:'primary',baseline:{outcome:'completed',taskPassed:true,fullScore:1},candidate:{outcome:'completed',taskPassed:true,fullScore:1}}));
 const admitted=D.adjudicate(m,selected,allPrimary);assert(admitted.accepted);
 assert(D.application(m,selected,admitted,m.test[0],fit).startPaused);
 const changed={...m,theoryDesign:{...m.theoryDesign,forceScale_N:4}};
 assert.throws(()=>D.application(changed,selected,admitted,m.test[0],fit),/settings|signature|changed/i);
 assert.throws(()=>D.application(m,selected,admitted,m.test[0],{...fit,Rdiag:fit.Rdiag.map(v=>2*v)}),/calibration|covariance|changed/i);
 const poisoned=JSON.parse(JSON.stringify(selected));assert.throws(()=>D.application(m,poisoned,{accepted:true,recommendedPairId:'other'},m.test[0],fit),/selection|admission|identity|case/i);
 fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/design_contract_fixture.json',JSON.stringify({fit,fixture,selection:selected},null,2)+'\n');
 console.log('Task design contracts PASS: four pairs, fixed score, measured R, Q/R/P0 identity, no test selection, partial/failure rejection and explicit application gate');
}finally{b.dispose();}
