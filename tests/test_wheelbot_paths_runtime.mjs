import fs from 'node:fs';
import assert from 'node:assert/strict';
import {createWheelbotContactBackend} from '../src/wheelbot_contact_backend.mjs';
const variant=process.env.WHEELBOT_VARIANT_DIR;
const {createWheelbotActions}=await import(variant?new URL('../'+variant+'/wheelbot_actions.mjs',import.meta.url):new URL('../src/wheelbot_actions.mjs',import.meta.url));
import {worldTransform} from '../src/wheelbot_live_view.mjs';
for(const [w,h] of [[390,310],[780,620],[1400,650]]){const t=worldTransform(w,h),p={x:.2,z:.4},back=t.toWorld(t.toPixel(p));assert(Math.abs(back.x-p.x)<1e-12&&Math.abs(back.z-p.z)<1e-12);}
const read=n=>JSON.parse(fs.readFileSync((variant??'assets/wheelbot')+'/'+n+'.json'));
const p=read('live_profile'),atlas=read('pose_profiles'),b=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));
fs.mkdirSync('test-results',{recursive:true});
const rows=[],parity=[],control=[],plant=[],loop=[];
const protocol=JSON.parse(fs.readFileSync('tests/fixtures/wheelbot_paths_protocol.json'));
try{
 for(let i=0;i<8;i++){
  const source=i<4?p:atlas.profiles[12],trial=createWheelbotActions(b,p,atlas,{seed:[1,7,42,2026][i%4],initial:i<4?null:{truth:[...source.qref,0,0,0,0,0,0],estimate:[...source.qref.slice(0,5),0,0,0,0,0,0],appliedTarget:{x:0,z:source.qref[1],pitch:source.qref[2]}}});
  for(let k=0;k<100;k++)trial.step();
  const z=source.qref[1],direction=i%2?-1:1;
  const points=i===6?[{x:.04,z:z+.01},{x:.08,z:z-.015},{x:.1,z}]:i===7?[{x:.08,z},{x:-.05,z}]:[{x:i%4<2?direction*.1:0,z:z+(i%4===2?-.03:i%4===3?.03:0)}];
  const before=trial.snapshot();await trial.requestPath(points,{yieldTask:async()=>{}});let s=trial.snapshot();
  assert.deepEqual(s.truth,before.truth);assert.deepEqual(s.estimate,before.estimate);
  let pen=0,exc=0,peakPitch=0,peakRate=0,sat=0,at15=null,endpointTimeS=null,peakPathDistanceM=0;
  for(let k=0;k<600;k++){const x=trial.snapshot().truth;const loopStart=performance.now();s=trial.step();loop.push(performance.now()-loopStart);control.push(s.last.controlMs);plant.push(s.last.plantMs);if(i===0&&k<50)parity.push({before:x,u:s.last.u,after:s.truth});pen=Math.max(pen,s.last.physical.maximumPenetrationM);exc=Math.max(exc,s.last.physical.jointLimitExcursionRad);peakPitch=Math.max(peakPitch,Math.abs(s.truth[2]-source.qref[2]));peakRate=Math.max(peakRate,...s.truth.slice(6).map(Math.abs));sat+=s.last.requested.some((v,j)=>Math.abs(v)>b.limits[j]);const accepted=s.path?.accepted;if(accepted?.length){let distance=Infinity;const nodes=[{x:before.truth[0],z:before.truth[1]},...accepted];for(let j=1;j<nodes.length;j++){const a=nodes[j-1],c=nodes[j],dx=c.x-a.x,dz=c.z-a.z,f=Math.max(0,Math.min(1,((s.truth[0]-a.x)*dx+(s.truth[1]-a.z)*dz)/(dx*dx+dz*dz||1)));distance=Math.min(distance,Math.hypot(s.truth[0]-a.x-f*dx,s.truth[1]-a.z-f*dz));}peakPathDistanceM=Math.max(peakPathDistanceM,distance);if(endpointTimeS===null&&Math.abs(s.truth[0]-s.target.x)<=.02&&Math.abs(s.truth[1]-s.target.z)<=.005&&s.truth.slice(6).every(v=>Math.abs(v)<=.3))endpointTimeS=(k+1)*.01;}if(k===149)at15=s.truth.slice();if(s.failed)break;}
  const goal=points.at(-1),err=x=>[Math.abs(x[0]-goal.x),Math.abs(x[1]-goal.z),Math.abs(x[2]-source.qref[2])];
  const accepted15Errors=s.path?.available&&at15?[Math.abs(at15[0]-s.target.x),Math.abs(at15[1]-s.target.z),Math.abs(at15[2]-s.target.pitch)]:null;
  const feasibleStep=points.length===1&&s.path?.available&&Math.hypot(s.target.x-goal.x,s.target.z-goal.z)<1e-8;
  rows.push({feasibleStep,criterion:feasibleStep?'1.5 s original feasible step':points.length>1?'declared path duration plus 1.5 s settling':'accepted projected endpoint; original error retained',declaredCompletionS:(s.path?.durationS??0)+(s.path?.settlingS??1.5),i,task:i>=6?'multi-waypoint path':i===3?'projected height':'single target',endpointTimeS,peakPathDistanceM,accepted15Errors,accepted15Pass:!!accepted15Errors&&accepted15Errors.every((e,j)=>e<=[.02,.005,.03][j])&&at15.slice(6).every(v=>Math.abs(v)<=.3),requested:points,admission:s.path,at15Errors:at15?err(at15):null,goal15Pass:!!at15&&err(at15).every((e,j)=>e<=[.02,.005,.03][j])&&at15.slice(6).every(v=>Math.abs(v)<=.3),finalErrors:err(s.truth),acceptedFinalErrors:s.path?.available?[Math.abs(s.truth[0]-s.target.x),Math.abs(s.truth[1]-s.target.z),Math.abs(s.truth[2]-s.target.pitch)]:null,requestedProjectionDeviation:s.path?.accepted?Math.hypot(s.path.accepted.at(-1).x-goal.x,s.path.accepted.at(-1).z-goal.z):null,failed:s.failed,pen,exc,peakPitch,peakRate,saturatedCommands:sat,finalRate:Math.max(...s.truth.slice(6).map(Math.abs))});
 }
 const stats=a=>({p95:a.slice().sort((a,b)=>a-b)[Math.floor(a.length*.95)],max:Math.max(...a)});
 fs.writeFileSync('test-results/'+(variant?'q80-paired':'wheelbot-paths-runtime')+'.json',JSON.stringify({protocol,modelHash:b.assetSha256,rows,timing:{pureControlMs:stats(control),plantMs:stats(plant),loopMs:stats(loop),browserFPS:null,browserRTF:null}},null,2));
 fs.writeFileSync('test-results/wheelbot-box-parity-input.json',JSON.stringify({modelHash:b.assetSha256,parity}));
 console.log(JSON.stringify({admitted:rows.filter(r=>r.admission.available).length,goal15Pass:rows.filter(r=>r.goal15Pass).length,physicalFailures:rows.filter(r=>r.failed).length,rows:rows.map(r=>({i:r.i,accepted:r.admission.available,duration:r.admission.durationS,reason:r.admission.reason,errors:r.at15Errors}))}));
 if(process.argv.includes('--require-feasible-fast'))assert(rows.filter(r=>r.feasibleStep).every(r=>r.goal15Pass),'Feasible 1.5 s step goal failed; no release claim');
 if(process.argv.includes('--require-fast'))assert(rows.every(r=>r.goal15Pass),'Frozen 1.5 s transient contract failed; evidence retained');
 const t=createWheelbotActions(b,p,atlas);for(const point of [{x:5,z:2},{x:0,z:-1}]){await t.requestPath([point],{yieldTask:async()=>{}});assert(t.snapshot().path.requested);assert(t.snapshot().path.reason);}
 const preserved=t.snapshot();t.cancel();assert.deepEqual(t.snapshot().truth,preserved.truth);assert.deepEqual(t.snapshot().estimate,preserved.estimate);
 const moving=createWheelbotActions(b,p,atlas);for(let k=0;k<100;k++)moving.step();
 await moving.requestPath([{x:.08,z:.45}],{yieldTask:async()=>{}});for(let k=0;k<35;k++)moving.step();
 const mid=moving.snapshot();await moving.requestPath([{x:-.05,z:.46}],{yieldTask:async()=>{}});assert.deepEqual(moving.snapshot().truth,mid.truth);assert.deepEqual(moving.snapshot().estimate,mid.estimate);
 for(let k=0;k<500;k++)assert(!moving.step().failed,'Sudden replacement reverse must remain physical');
 const foot=createWheelbotActions(b,p,atlas);for(let k=0;k<100;k++)foot.step();await foot.requestPath([{x:.05,z:.2}],{mode:'wheel',yieldTask:async()=>{}});assert(foot.snapshot().path.available,JSON.stringify(foot.snapshot().path));assert(Math.abs(foot.snapshot().path.accepted[0].z-.05)<.001);for(let k=0;k<500;k++)assert(!foot.step().failed);
 assert(Math.abs(b.geometry(foot.snapshot().truth).wheel[0]-.05)<.02);
 const old=t.snapshot();await t.requestPath([{x:.1,z:.7}],{action:'jump'});assert.deepEqual(t.snapshot().truth,old.truth);assert.equal(t.snapshot().path.available,false);
 await assert.rejects(t.requestPath([{x:NaN,z:0}]));
 assert(rows.every(r=>!r.failed&&r.pen<=.005&&r.exc<=.02));
}finally{b.dispose();}
