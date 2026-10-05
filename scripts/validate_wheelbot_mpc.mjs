import fs from 'node:fs';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import {createWheelbotBackend,CONTROLLED_INDICES} from '../src/wheelbot_backend.mjs';
import {createWheelbotTrial,lqrTorque} from '../src/wheelbot_control.mjs';
const p=JSON.parse(fs.readFileSync('assets/wheelbot/profile.json')),manifest=JSON.parse(fs.readFileSync('tests/fixtures/wheelbot_cases.json'));
const b=await createWheelbotBackend(fs.readFileSync('assets/wheelbot/wheelbot.xml','utf8')),rows=[],raw=[];
try{
 const x=[...p.qref,0,0,0,0,0,0],shift=x.slice();shift[0]+=.03;const original=b.step(x,p.uref),translated=b.step(shift,p.uref);translated[0]-=.03;
 const translationError=Math.max(...original.map((v,i)=>Math.abs(v-translated[i])));assert(translationError<1e-8);
 const profiles=[['baseline',p]];if(process.argv.includes('--response'))profiles.push(['response',JSON.parse(fs.readFileSync('assets/wheelbot/response_profile.json'))]);
 const seeds=process.argv.includes('--new-seeds')?[101,211,307]:manifest.seeds;
 for(const [profileLabel,profile] of profiles)for(const mode of ['lqr_kf','mpc_kf'])for(const c of manifest.cases)for(const seed of seeds){
  const initial=x.slice();initial[2]+=c.pitchOffset;const t=createWheelbotTrial(b,profile,{initialState:initial,mode,seed,goal:c.goal});let s=t.snapshot(),trace=[],rejection=null;
  const start=performance.now();while(s.steps<manifest.steps&&!s.failed){const push=c.push,force=push&&s.steps>=push.start&&s.steps<push.end?push.force:0;try{const comparison=lqrTorque(profile,s.estimate,c.goal,b.limits);s=t.step(force);const {E,U,...last}=s.last;trace.push({...s,last:{...last,sameEstimateLqrU:comparison.u}});}catch(e){if(!String(e.message).startsWith('QP rejected:'))throw e;rejection=e.message;break;}}
  const elapsedMs=performance.now()-start,tail=trace.slice(-manifest.tailSteps),completed=s.steps===manifest.steps&&!s.failed;
  const taskPassed=completed&&tail.every(t=>Math.abs(t.truth[0]-p.qref[0]-c.goal)<manifest.positionTolerance&&Math.abs(t.truth[2]-p.qref[2])<manifest.pitchTolerance);
  const rms=f=>trace.length?Math.sqrt(trace.reduce((sum,t,i)=>sum+f(t,i)**2,0)/trace.length):null;
  raw.push({profile:profileLabel,mode,name:c.name,seed,initial,trace,rejection,elapsedMs});
  rows.push({profile:profileLabel,mode,name:c.name,seed,completed,taskPassed,steps:s.steps,qpRejected:!!rejection,rejection,physicalEnvelopeFailure:s.failed,contactLoss:trace.filter(t=>!t.last.contact.wheelContacts).length,saturations:trace.filter(t=>t.last.saturated).length,
   estimationRmseByState:CONTROLLED_INDICES.map((i,j)=>rms(t=>t.truth[i]-t.estimate[j])),units:['m','m','rad','rad','rad','m/s','m/s','rad/s','rad/s','rad/s','rad/s'],
   tailPositionMax:tail.length?Math.max(...tail.map(t=>Math.abs(t.truth[0]-p.qref[0]-c.goal))):null,tailPitchMax:tail.length?Math.max(...tail.map(t=>Math.abs(t.truth[2]-p.qref[2]))):null,torqueRmsNm:[0,1,2].map(j=>rms(t=>t.last.u[j])),slewRmsNmPerSecond:[0,1,2].map(j=>rms((t,i)=>(t.last.u[j]-(i?trace[i-1].last.u[j]:p.uref[j]))/.01)),maxSlip:trace.length?Math.max(...trace.map(t=>Math.abs(t.last.contact.slip))):null,elapsedMs,appliedTorqueAtLimitSamples:trace.filter(t=>t.last.saturated).length,predictedConstraintActiveSamples:mode==='mpc_kf'?trace.filter(t=>t.last.forecastConstraintActive).length:null,maxSameEstimateLqrDifferenceNm:trace.length?Math.max(...trace.flatMap(t=>t.last.u.map((v,j)=>Math.abs(v-t.last.sameEstimateLqrU[j])))):null,solverMaxMs:mode==='mpc_kf'&&trace.length?Math.max(...trace.map(t=>t.last.solveMs)):null,final:s.truth});
 }
 if(!process.argv.includes('--new-seeds')){
  const baseline=JSON.parse(fs.readFileSync('evidence/wheelbot_validation.json')).rows;
  for(const r of rows.filter(r=>r.mode==='lqr_kf')){const old=baseline.find(v=>v.name===r.name&&v.seed===r.seed);assert.equal(r.completed,old.completed);assert.equal(r.taskPassed,old.taskPassed);assert.deepEqual(r.final,old.final);}
 }
 const hashes=Object.fromEntries(profiles.map(([label])=>[label,crypto.createHash('sha256').update(fs.readFileSync('assets/wheelbot/'+(label==='baseline'?'profile':'response_profile')+'.json')).digest('hex')]));
 const receipt={profileSha256:hashes,schema:'wheelbot-mpc-regression/v1',translationError,seeds,scope:'Same physical cases; independent noise seeds only when new-seeds selected. Host timing, no real-time promise.',rows};
 const suffix=process.argv.includes('--response')?'response_validation':process.argv.includes('--new-seeds')?'new_seeds':'regression';
 if(process.argv.includes('--response'))fs.writeFileSync('test-results/wheelbot_mpc_response_traces.json',JSON.stringify({profileSha256:hashes,rows:raw}));
 fs.writeFileSync('evidence/wheelbot_mpc_'+suffix+'.json',JSON.stringify(receipt,null,2)+'\n');
 console.log(JSON.stringify(rows.reduce((a,r)=>{const key=r.profile+'/'+r.mode;const z=a[key]??={completed:0,taskPassed:0,qpRejected:0,physicalEnvelopeFailure:0};for(const k of Object.keys(z))z[k]+=Number(r[k]);return a;},{})));
}finally{b.dispose();}
