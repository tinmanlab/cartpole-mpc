// Frozen N-link feasibility screen; validity, completion and task achievement are separate.
import fs from 'node:fs';import crypto from 'node:crypto';
import {createChainBackend} from '../src/chain_backend.mjs';
import {createChainTrial,relativeAngles} from '../src/chain_control.mjs';
const sha=p=>crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const manifestPath='tests/fixtures/chain_validation.json';const manifest=JSON.parse(fs.readFileSync(manifestPath,'utf8'));
const profiles=JSON.parse(fs.readFileSync('assets/chains/profiles.json','utf8')).profiles;
const sourceFiles=['scripts/build_chain_assets.py','scripts/design_chain_profiles.py','src/chain_backend.mjs','src/chain_control.mjs','src/qp.js','scripts/validate_chain.mjs','assets/chains/profiles.json',manifestPath];
const before=Object.fromEntries(sourceFiles.map(p=>[p,sha(p)])),rows=[],started=new Date().toISOString();
for(const profile of profiles){
 const b=await createChainBackend(fs.readFileSync(profile.asset,'utf8'),profile.poles);
 try{
  for(const test of manifest.cases)for(const controller of manifest.controllers)for(const observer of manifest.observers){
   const abs=Array.from({length:profile.poles},(_,i)=>test.absoluteAngle*(i%2?-.5:1));
   const initial=[test.cart,...relativeAngles(abs),...Array(profile.dof).fill(0)];
   let trial,outcome='completed',reason=null,states=[],errors=[],commands=[],times=[],tail=[],last=null;
   try{
    trial=createChainTrial(b,profile,{controller,observer,goal:test.goal,seed:test.seed,initialState:initial,noise:manifest.encoderSigma});
    for(let k=0;k<manifest.steps;k++){
     const ext=test.push&&k>=test.push.step&&k<test.push.step+test.push.duration?test.push.force:0;
     last=trial.step(ext);const err=last.truth[0]-test.goal,angle=Math.max(...last.absoluteAngles.map(Math.abs));
     states.push({p:last.truth[0],angle});errors.push(err*err);commands.push(last.last.u);times.push(last.last.solveMs);
     tail.push({tracking:Math.abs(err),angle});if(tail.length>100)tail.shift();
     if(last.last.failed){outcome='plant-envelope-failure';break;}
    }
   }catch(e){reason=String(e.message);outcome=reason.startsWith('Design rejected')?'design-rejected':reason.startsWith('QP ')?'solver-rejected':'execution-error';}
   const rms=values=>values.length?Math.sqrt(values.reduce((s,v)=>s+v,0)/values.length):null;
   const sort=times.slice().sort((a,b)=>a-b);
   const taskPassed=outcome==='completed'&&tail.length===100&&Math.max(...tail.map(v=>v.tracking))<=manifest.finalPositionTolerance&&Math.max(...tail.map(v=>v.angle))<=manifest.finalAngleTolerance;
   const row={poles:profile.poles,case:test.name,controller,observer,outcome,reason,appliedSteps:states.length,taskPassed,
    positionRmse:rms(errors),maxPosition:states.length?Math.max(...states.map(x=>Math.abs(x.p))):null,maxAbsoluteLinkAngle:states.length?Math.max(...states.map(x=>x.angle)):null,
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
const report={schema:'cartpole-chain-validation/v1',started,completed:new Date().toISOString(),manifestSha256:sha(manifestPath),sourceSha256:before,sourcesStable:stable,summary,rows,
 experimentValid:stable&&rows.every(r=>r.outcome!=='execution-error'),scope:'local balancing/limited tracking; fixed per-link mass and length make total mass/height increase with N; not swing-up, global reachability or a humanoid contact claim',hardware:'NOT_EVALUATED'};
fs.writeFileSync('evidence/chain_validation.json',JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(summary,null,2));if(!report.experimentValid)process.exitCode=1;
