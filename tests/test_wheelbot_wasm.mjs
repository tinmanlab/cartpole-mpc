import fs from 'node:fs';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {createWheelbotBackend,CONTROLLED_INDICES} from '../src/wheelbot_backend.mjs';
import {validateWheelbotProfile,createWheelbotTrial,observerUpdate,lqrTorque} from '../src/wheelbot_control.mjs';
const p=JSON.parse(fs.readFileSync('assets/wheelbot/profile.json','utf8'));
const xml=fs.readFileSync('assets/wheelbot/wheelbot.xml','utf8'),b=await createWheelbotBackend(xml);
const error=(a,b)=>Math.max(...a.flat(Infinity).map((v,i)=>Math.abs(v-b.flat(Infinity)[i])));
try{
 assert.equal(b.nx,12);assert.equal(b.nu,3);validateWheelbotProfile(b,p);
 assert.throws(()=>validateWheelbotProfile(b,{...p,assetSha256:'stale'}),/hash mismatch/);
 assert.throws(()=>validateWheelbotProfile(b,{...p,K:[[0]]}),/dimensions/);
 assert.throws(()=>createWheelbotTrial(b,p,{mode:'mpc'}),/NOT_YET_SUPPORTED/);
 const x=[...p.qref,0,0,0,0,0,0],g=b.geometry(x);
 assert(Math.abs(g.hip[1])+Math.abs(g.knee[1])+Math.abs(g.wheel[1])<1e-12);
 assert(Math.abs(Math.hypot(...g.hip.map((v,i)=>v-g.knee[i]))-.25)<1e-10);
 assert(Math.abs(Math.hypot(...g.knee.map((v,i)=>v-g.wheel[i]))-.25)<1e-10);
 assert(Math.abs(Math.abs(g.wheelRotation[5])-1)<1e-10,'cylinder local z axis must be world y');
 const shifted=x.slice();shifted[5]+=1.234;
 assert(error(CONTROLLED_INDICES.map(i=>b.step(x,p.uref)[i]),CONTROLLED_INDICES.map(i=>b.step(shifted,p.uref)[i]))<1e-8,'wheel phase symmetry');
 assert(error(b.step(x,[0,0,0]),x)>1e-5,'passive gravity is active');
 const largeEstimate=CONTROLLED_INDICES.map(i=>i<6?p.qref[i]:0);largeEstimate[2]+=10;const clipped=lqrTorque(p,largeEstimate,0,b.limits);assert(clipped.saturated&&clipped.u.every((v,i)=>Math.abs(v)<=b.limits[i]));
 assert(error(b.step(x,[100,-100,100]),b.step(x,b.limits.map((v,i)=>i===1?-v:v)))<1e-12,'saturation');
 const airborne=x.slice();airborne[1]+=.2;assert.equal(b.contact(airborne).count,0);
 const wheelReaction=b.step(airborne,[0,0,.1]),passiveAir=b.step(airborne,[0,0,0]);assert(Math.abs(wheelReaction[8]-passiveAir[8])>1e-5,'airborne wheel torque reacts on chassis');
 assert.equal(b.contact(x).wheelContacts>0,true,'wheel contact detected');
 let motorEffects=[];for(let j=0;j<3;j++){const u=p.uref.slice();u[j]+=.1;const delta=error(b.step(x,u),b.step(x,p.uref));assert(delta>1e-5);motorEffects.push(delta);}
 const linA=Array.from({length:11},()=>Array(11).fill(0)),linB=Array.from({length:11},()=>Array(3).fill(0)),eps=1e-6;
 for(let j=0;j<11;j++){const a=x.slice(),c=x.slice();a[CONTROLLED_INDICES[j]]+=eps;c[CONTROLLED_INDICES[j]]-=eps;const plus=b.step(a,p.uref),minus=b.step(c,p.uref);for(let i=0;i<11;i++)linA[i][j]=(plus[CONTROLLED_INDICES[i]]-minus[CONTROLLED_INDICES[i]])/(2*eps);}
 for(let j=0;j<3;j++){const a=p.uref.slice(),c=p.uref.slice();a[j]+=eps;c[j]-=eps;const plus=b.step(x,a),minus=b.step(x,c);for(let i=0;i<11;i++)linB[i][j]=(plus[CONTROLLED_INDICES[i]]-minus[CONTROLLED_INDICES[i]])/(2*eps);}
 const derivativeErrorA=error(linA,p.A),derivativeErrorB=error(linB,p.B);
 assert(derivativeErrorA<.005&&derivativeErrorB<.005,JSON.stringify({derivativeErrorA,derivativeErrorB}));
 // The historical solver explicitly condenses only B[:,0]. It cannot accept MIMO.
 const QP=createRequire(import.meta.url)('../src/qp.js');
 const scalar=QP.solve([[.9]],[[1]],[[1]],1,[[1]],[.1],3,1);
 const mimo=QP.solve([[.9]],[[1,7,9]],[[1]],1,[[1]],[.1],3,1);
 assert.deepEqual(scalar,mimo,'document existing single-input boundary; never route wheelbot through it');
 const replay=[];
 for(const height of [0,.08]){
  let z=x.slice();z[1]+=height;z[2]+=.003;
  const initial=z.slice(),inputs=[],states=[];
  for(let k=0;k<80;k++){const u=p.uref.map((v,i)=>v+.02*Math.sin(.17*k+i));const external=k>=20&&k<24?.2:0;inputs.push({u,external});z=b.step(z,u,external);states.push(z);}
  replay.push({initial,inputs,states});
 }
 const estimate=CONTROLLED_INDICES.map((i,j)=>(i<6?p.qref[i]:0)+.001*(j-4));
 const u=p.uref.map((v,i)=>v+.01*(i-1)),y=p.qref.slice(0,5).map((v,i)=>v+.001*i);
 const algebra={estimate,u,y,goal:.03,observer:observerUpdate(p,estimate,u,y),control:lqrTorque(p,estimate,.03,b.limits)};
 const withVelocity=x.slice();withVelocity[6]=.4;
 assert.deepEqual(createWheelbotTrial(b,p,{seed:7,initialState:x}).snapshot().estimate,createWheelbotTrial(b,p,{seed:7,initialState:withVelocity}).snapshot().estimate,'no true velocity initialization');
 const raw=[];
 const manifest=JSON.parse(fs.readFileSync('tests/fixtures/wheelbot_cases.json','utf8')),rows=[];
 for(const c of manifest.cases)for(const seed of manifest.seeds){
  const initial=x.slice();initial[2]+=c.pitchOffset;const trial=createWheelbotTrial(b,p,{initialState:initial,goal:c.goal,seed});let s=trial.snapshot(),tail=[],saturations=0,contactLoss=0,maxSlip=0;const trace=[];
  while(s.steps<manifest.steps&&!s.failed){const push=c.push,force=push&&s.steps>=push.start&&s.steps<push.end?push.force:0;s=trial.step(force);trace.push(s);tail.push(s.truth.slice());if(tail.length>manifest.tailSteps)tail.shift();saturations+=Number(s.last.saturated);contactLoss+=Number(!s.last.contact.wheelContacts);maxSlip=Math.max(maxSlip,Math.abs(s.last.contact.slip));}
  const completed=s.steps===manifest.steps&&!s.failed,taskPassed=completed&&tail.every(t=>Math.abs(t[0]-p.qref[0]-c.goal)<manifest.positionTolerance&&Math.abs(t[2]-p.qref[2])<manifest.pitchTolerance);
  const movement=[2,3,4].map(i=>Math.max(...trace.map(t=>t.truth[i]))-Math.min(...trace.map(t=>t.truth[i])));
  const estimationRmseByState=CONTROLLED_INDICES.map((i,j)=>Math.sqrt(trace.reduce((sum,t)=>sum+(t.truth[i]-t.estimate[j])**2,0)/trace.length));
  const measurementRmseByChannel=Array.from({length:5},(_,i)=>Math.sqrt(trace.reduce((sum,t)=>sum+(t.last.measurement[i]-t.truth[i])**2,0)/trace.length));
  rows.push({name:c.name,seed,completed,taskPassed,steps:s.steps,saturations,contactLoss,maxSlip,movementPitchHipKnee:movement,estimationRmseByState,estimationUnits:['m','m','rad','rad','rad','m/s','m/s','rad/s','rad/s','rad/s','rad/s'],measurementRmseByChannel,measurementUnits:['m','m','rad','rad','rad'],final:s.truth});
  raw.push({name:c.name,seed,initial,trace});
 }
 fs.mkdirSync('test-results',{recursive:true});
 fs.writeFileSync('test-results/wheelbot_wasm_reference.json',JSON.stringify({assetSha256:b.assetSha256,linA,linB,replay,algebra,raw}));
 fs.writeFileSync('evidence/wheelbot_validation.json',JSON.stringify({schema:'wheelbot-wasm/v1',profileSha256:crypto.createHash('sha256').update(fs.readFileSync('assets/wheelbot/profile.json')).digest('hex'),manifestSha256:crypto.createHash('sha256').update(fs.readFileSync('tests/fixtures/wheelbot_cases.json')).digest('hex'),physicalTorqueScope:'hip/knee/wheel relative-joint motors; externalX is a separately reported disturbance, never wheel actuation',controlledIndices:CONTROLLED_INDICES,physics:b.diagnostics(),motorEffects,derivativeErrorA,derivativeErrorB,rows},null,2)+'\n');
 console.log(JSON.stringify({motorEffects,derivativeErrorA,derivativeErrorB,trials:rows.length,completed:rows.filter(r=>r.completed).length,taskPassed:rows.filter(r=>r.taskPassed).length},null,2));
}finally{b.dispose();}
