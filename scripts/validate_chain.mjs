// Frozen N-link feasibility screen; validity, completion and task achievement are separate.
import fs from 'node:fs';import crypto from 'node:crypto';
import {createChainBackend} from '../src/chain_backend.mjs';
import {createChainTrial,relativeAngles} from '../src/chain_control.mjs';
const sha=p=>crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const manifestPath='tests/fixtures/chain_validation.json';const manifest=JSON.parse(fs.readFileSync(manifestPath,'utf8'));
const prestabilized=process.argv.includes('--prestabilized');
const controllers=prestabilized?['mpc_pre']:manifest.controllers;
const nonlinearObserver=process.argv.includes('--ekf');
const observers=nonlinearObserver?['ekf']:manifest.observers;
const outputPath=nonlinearObserver?'evidence/chain_ekf_validation.json':prestabilized?'evidence/chain_preconditioned_validation.json':'evidence/chain_validation.json';
const profiles=JSON.parse(fs.readFileSync('assets/chains/profiles.json','utf8')).profiles;
const sourceFiles=['scripts/build_chain_assets.py','scripts/design_chain_profiles.py','src/chain_backend.mjs','src/chain_control.mjs','src/chain_preconditioned.mjs','src/chain_observer.mjs','src/qp.js','scripts/validate_chain.mjs','assets/chains/profiles.json',manifestPath];
const before=Object.fromEntries(sourceFiles.map(p=>[p,sha(p)])),rows=[],started=new Date().toISOString();
for(const profile of profiles){
 const b=await createChainBackend(fs.readFileSync(profile.asset,'utf8'),profile.poles);
 try{
  for(const test of manifest.cases)for(const controller of controllers)for(const observer of observers){
   const abs=Array.from({length:profile.poles},(_,i)=>test.absoluteAngle*(i%2?-.5:1));
   const initial=[test.cart,...relativeAngles(abs),...Array(profile.dof).fill(0)];
   let trial,outcome='completed',reason=null,states=[],errors=[],commands=[],times=[],computeTimes=[],tail=[],last=null;
   const stateSE=Array(profile.nx).fill(0);let noiseToInputSE=0,constructionMs=null;
   try{
    const initTime=performance.now();
    trial=createChainTrial(b,profile,{controller,observer,goal:test.goal,seed:test.seed,initialState:initial,noise:manifest.encoderSigma});constructionMs=performance.now()-initTime;
    for(let k=0;k<manifest.steps;k++){
     const ext=test.push&&k>=test.push.step&&k<test.push.step+test.push.duration?test.push.force:0;
     last=trial.step(ext);const err=last.truth[0]-test.goal,angle=Math.max(...last.absoluteAngles.map(Math.abs));
     const error=last.estimate.map((v,i)=>{let d=v-last.truth[i];if(i>0&&i<profile.dof)d=Math.atan2(Math.sin(d),Math.cos(d));stateSE[i]+=d*d;return d;});noiseToInputSE+=profile.K[0].reduce((s,v,i)=>s+v*error[i],0)**2;
     states.push({p:last.truth[0],angle});errors.push(err*err);commands.push(last.last.u);times.push(last.last.solveMs);computeTimes.push(last.last.computeMs);
     tail.push({tracking:Math.abs(err),angle});if(tail.length>100)tail.shift();
     if(last.last.failed){outcome='plant-envelope-failure';break;}
    }
   }catch(e){reason=String(e.message);outcome=reason.startsWith('Design rejected')?'design-rejected':reason.startsWith('QP ')?'solver-rejected':'execution-error';}
   const rms=values=>values.length?Math.sqrt(values.reduce((s,v)=>s+v,0)/values.length):null;
   const sort=times.slice().sort((a,b)=>a-b);const computeSort=computeTimes.slice().sort((a,b)=>a-b);
   const taskPassed=outcome==='completed'&&tail.length===100&&Math.max(...tail.map(v=>v.tracking))<=manifest.finalPositionTolerance&&Math.max(...tail.map(v=>v.angle))<=manifest.finalAngleTolerance;
   const row={poles:profile.poles,case:test.name,controller,observer,outcome,reason,appliedSteps:states.length,taskPassed,
    constructionMs,estimationRmseByState:states.length?stateSE.map(v=>Math.sqrt(v/states.length)):null,estimationMetricUnits:'[m, rad x N, m/s, rad/s x N]',lqrSensitivityErrorRms_N:states.length?Math.sqrt(noiseToInputSE/states.length):null,
    rmsForce:commands.length?Math.sqrt(commands.reduce((s,v)=>s+v*v,0)/commands.length):null,saturatedCommandSamples:commands.filter(v=>Math.abs(v)>=profile.forceLimit-1e-8).length,
    synchronousComputeP95Ms:computeSort.length?computeSort[Math.ceil(.95*computeSort.length)-1]:null,positionRmse:rms(errors),maxPosition:states.length?Math.max(...states.map(x=>Math.abs(x.p))):null,maxAbsoluteLinkAngle:states.length?Math.max(...states.map(x=>x.angle)):null,
    maxForce:commands.length?Math.max(...commands.map(Math.abs)):null,solveP95Ms:sort.length?sort[Math.ceil(.95*sort.length)-1]:null,finalState:last?.truth??null,finalEstimate:last?.estimate??null,
    tailMaximumPositionError:tail.length?Math.max(...tail.map(t=>t.tracking)):null,tailMaximumLinkAngle:tail.length?Math.max(...tail.map(t=>t.angle)):null};
   rows.push(row);console.log(JSON.stringify({n:profile.poles,case:test.name,controller,observer,outcome,taskPassed,steps:states.length}));
  }
 }finally{b.dispose();}
}
const summary=profiles.map(p=>({poles:p.poles,nx:p.nx,designAvailable:p.designAvailable,
 completed:rows.filter(r=>r.poles===p.poles&&r.outcome==='completed').length,taskPassed:rows.filter(r=>r.poles===p.poles&&r.taskPassed).length,
 trials:rows.filter(r=>r.poles===p.poles).length,rejections:rows.filter(r=>r.poles===p.poles&&r.outcome.endsWith('rejected')).length}));
const stable=sourceFiles.every(p=>sha(p)===before[p]);
const report={schema:'cartpole-chain-validation/v2',coordinateForm:prestabilized?'u=-K e+v, same original cost/constraints':'direct-input baseline',started,completed:new Date().toISOString(),manifestSha256:sha(manifestPath),sourceSha256:before,sourcesStable:stable,summary,rows,
 experimentValid:stable&&rows.every(r=>r.outcome!=='execution-error'),scope:'local balancing/limited tracking; fixed per-link mass and length make total mass/height increase with N; not swing-up, global reachability or a humanoid contact claim',hardware:'NOT_EVALUATED'};
fs.writeFileSync(outputPath,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(summary,null,2));if(!report.experimentValid)process.exitCode=1;
