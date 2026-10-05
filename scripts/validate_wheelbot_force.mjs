// User-authorized adjustment of the external challenge, not controller retuning.
import fs from 'node:fs';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import {createWheelbotBackend} from '../src/wheelbot_backend.mjs';
import {createForceTrial,validateForceProtocol,selectNormalForce,admitNormalForce} from '../src/wheelbot_disturbance.mjs';
const sha=f=>crypto.createHash('sha256').update(fs.readFileSync(f)).digest('hex');
const path='tests/fixtures/wheelbot_force_envelope.json',protocol=JSON.parse(fs.readFileSync(path));
validateForceProtocol(protocol);
const sources=[path,protocol.profile,protocol.asset,protocol.historicalCases,'src/wheelbot_control.mjs','src/wheelbot_backend.mjs','src/wheelbot_mpc.mjs','src/wheelbot_disturbance.mjs'];
const sourceSha256=Object.fromEntries(sources.map(f=>[f,sha(f)]));
const p=JSON.parse(fs.readFileSync(protocol.profile)),backend=await createWheelbotBackend(fs.readFileSync(protocol.asset,'utf8'));
const raw=[],screen=[],assessment=[],stress=[];let evaluationCount=0;
function execute(amplitudeN,direction,mode,seed,phase){
 const trial=createForceTrial(backend,p,protocol,{amplitudeN,direction,mode,seed});
 while(!trial.snapshot().done)trial.step();
 const result=trial.result();evaluationCount++;
 raw.push({phase,...result,trace:trial.history.map(s=>({steps:s.steps,truth:s.truth,estimate:s.estimate,u:s.last.u,measurement:s.last.measurement,externalX:s.last.externalX,contact:s.last.contact,failed:s.failed}))});
 return result;
}
try{
 for(const amp of [0,...protocol.amplitudesN])for(const mode of protocol.controllers)for(const seed of protocol.screenSeeds)for(const direction of protocol.directions)screen.push(execute(amp,direction,mode,seed,'screen'));
 const selection=selectNormalForce(protocol,screen);
 assert.equal(evaluationCount,120); // Includes both directions for zero force too.
 const lockAtEvaluation=evaluationCount;
 fs.writeFileSync('test-results/wheelbot_force_selection_locked.json',JSON.stringify({sourceSha256,selection,lockAtEvaluation},null,2)+'\n');
 if(selection.normalAmplitudeN!==null)for(const mode of protocol.controllers)for(const seed of protocol.assessmentSeeds)for(const direction of protocol.directions)assessment.push(execute(selection.normalAmplitudeN,direction,mode,seed,'normal-assessment'));
 for(const mode of protocol.controllers)for(const seed of protocol.assessmentSeeds)for(const direction of protocol.directions)stress.push(execute(protocol.stressN,direction,mode,seed,'stress-assessment'));
 const admission=admitNormalForce(protocol,selection,assessment);
 assert(sources.every(f=>sha(f)===sourceSha256[f]));
 const mass=p.modelMetadata.totalMassKg,g=9.81,amp=selection.normalAmplitudeN;
 const report={schema:'wheelbot-force-envelope/v1',sourceSha256,protocolSha256:sha(path),profileSha256:sha(protocol.profile),assetSha256:backend.assetSha256,
  massKg:mass,gravityMS2:g,weightN:mass*g,selection,admission,normal:amp===null?null:{amplitudeN:amp,durationSeconds:.2,impulseNs:amp*.2,forceToWeight:amp/(mass*g),fullImpulseVelocityScaleMS:amp*.2/mass,point:protocol.forcePoint},
  stress:{amplitudeN:40,durationSeconds:.2,impulseNs:8,forceToWeight:40/(mass*g),fullImpulseVelocityScaleMS:8/mass},
  lockAtEvaluation,evaluationCount,assessmentAfterLock:true,screen,assessment,stressAssessment:stress,taskCriteriaUnchanged:true,originalCasesUnchanged:true,scope:protocol.claimBoundary};
 fs.writeFileSync('test-results/wheelbot_force_raw.json',JSON.stringify({sourceSha256,rows:raw})+'\n');
 fs.writeFileSync('evidence/wheelbot_force_envelope.json',JSON.stringify(report,null,2)+'\n');
 console.log(JSON.stringify({screen:selection.groups,normal:report.normal,admission,stressCompleted:stress.filter(r=>r.completed).length,stressPassed:stress.filter(r=>r.taskPassed).length,evaluations:evaluationCount},null,2));
}finally{backend.dispose();}
