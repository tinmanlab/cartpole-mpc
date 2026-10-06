import fs from 'node:fs';
import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import {createWheelbotContactBackend} from '../src/wheelbot_contact_backend.mjs';
import {createWheelbotActions} from '../src/wheelbot_actions.mjs';
const read=p=>JSON.parse(fs.readFileSync(p));
const base=read('assets/wheelbot/live_profile.json'),bundle=read('assets/wheelbot/pose_profiles.json');
const cases=[];
for(const z of [.36,.3925,.425,.4575,.49])for(const pitch of [-Math.PI/18,0,Math.PI/18])cases.push({target:{x:cases.length%2?-1:1,z,pitch},seed:17+cases.length});
for(const x of [-1,0,1])cases.push({target:{x,z:.46,pitch:.08},seed:42});
for(const kind of ['push','gust','twist'])for(const direction of [-1,1])cases.push({target:{x:direction,z:.425,pitch:direction*.08},seed:7,disturbance:{kind,direction,strength:'medium'}});
// Freeze the protocol before any evaluation; failures stay in the evidence.
const protocol={durationS:20,finalWindowS:2,disturbanceAtS:12,limits:{xErrorM:.03,zErrorM:.01,pitchErrorRad:.04,jointExcursionRad:.02,standingPenetrationM:.005,finalBodyVelocity:.3,finalWheelVelocity:1},cases};
const fixture={schema:'wheelbot-actions-reference/v1',modelSha256:bundle.assetSha256,protocol,transitions:[]};
const b=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));
const results=[],controlTimes=[],plantTimes=[];
try{
 const t=createWheelbotActions(b,base,bundle);const before=t.snapshot();
 for(const fn of [()=>t.setTarget({x:NaN}),()=>t.setTarget({z:Infinity}),()=>t.disturb({kind:'fake',direction:1,strength:'light'}),()=>t.step(NaN),()=>t.step(5),()=>t.jump(),()=>t.recover()]){assert.throws(fn);assert.deepEqual(t.snapshot(),before);}
 t.setTarget({x:1});assert.deepEqual(t.snapshot().truth,before.truth);assert.deepEqual(t.snapshot().estimate,before.estimate);assert.equal(t.snapshot().steps,0);
 t.disturb({kind:'twist',direction:1,strength:'light'});const startSteps=b.diagnostics().steps;
 for(let i=0;i<11;i++)t.step();assert.deepEqual(t.snapshot().last.externalWrench,[0,0,0]);assert.equal(b.diagnostics().steps-startSteps,55);
 const clamped=t.setTarget({x:2,z:1,pitch:2});assert.equal(clamped.target.x,1);assert.equal(clamped.target.z,.49);assert.equal(clamped.target.pitch,Math.PI/18);assert.match(clamped.actionStatus,/clamped/);
 assert.equal(b.modelInfo().rootUnactuated,true);assert.deepEqual(b.modelInfo().torqueLimitsNm,[16,16,1.7]);
 for(const [index,c] of cases.entries()){
  const trial=createWheelbotActions(b,base,bundle,{seed:c.seed});trial.setTarget(c.target);
  const forceHistory=[];
  let peakPen=0,peakExc=0,peakTorque=[0,0,0],tail=[0,0,0],bodyVelocity=0,wheelVelocity=0,positiveGround=false;
  for(let k=0;k<2000;k++){
   if(k===1200&&c.disturbance)trial.disturb(c.disturbance);
   const before=trial.snapshot().truth,s=trial.step(),a=b.actionTelemetry(s.truth,s.last.u);
   if(s.last.externalWrench.some(v=>v!==0))forceHistory.push({timeS:s.steps*.01,wrench:s.last.externalWrench});
   peakPen=Math.max(peakPen,a.maximumPenetrationM);peakExc=Math.max(peakExc,a.jointLimitExcursionRad);
   a.actualTorqueNm.forEach((v,i)=>peakTorque[i]=Math.max(peakTorque[i],Math.abs(v)));
   positiveGround ||= a.pairs.some(p=>(p.geom1==='floor'||p.geom2==='floor')&&p.normalForceN>0);
   if(k>=1800){['x','z','pitch'].forEach((key,i)=>tail[i]=Math.max(tail[i],Math.abs(s.truth[i]-s.target[key])));bodyVelocity=Math.max(bodyVelocity,...s.truth.slice(6,11).map(Math.abs));wheelVelocity=Math.max(wheelVelocity,Math.abs(s.truth[11]));}
   controlTimes.push(s.last.controlMs);plantTimes.push(s.last.plantMs);
   if(k%200===0)fixture.transitions.push({case:index,step:k,before,u:s.last.u,wrench:s.last.externalWrench,after:s.truth});
  }
  const s=trial.snapshot();const pass=!s.failed&&tail[0]<=.03&&tail[1]<=.01&&tail[2]<=.04&&peakPen<=.005&&peakExc<=.02&&peakTorque.every((v,i)=>v<=[16,16,1.7][i]+1e-10)&&bodyVelocity<=.3&&wheelVelocity<=1&&positiveGround;
  results.push({index,...c,pass,failed:s.failed,phase:s.phase,tailErrors:tail,peakPen,peakExc,peakTorque,bodyVelocity,wheelVelocity,positiveGround,forceHistory,final:s.truth});
  console.log(JSON.stringify(results.at(-1)));
 }
 // Deliberate over-domain physical state: passive integration must continue.
 const original=b.stepWrench;let inject=true;
 const fallBackend={...b,stepWrench(x,u,w){if(inject){x=x.slice();x[2]=1;inject=false;}return original(x,u,w);}};
 const fall=createWheelbotActions(fallBackend,base,bundle);fall.step();assert(fall.snapshot().failed);
 const falling=fall.snapshot();assert.throws(()=>fall.setTarget({x:0}));assert.deepEqual(fall.snapshot(),falling);
 for(let i=0;i<500;i++)fall.step();assert.equal(fall.snapshot().steps,501);assert.notDeepEqual(fall.snapshot().truth,falling.truth);assert.deepEqual(fall.snapshot().last.u,[0,0,0]);
 const stress=[];
 for(const kind of ['push','gust','twist'])for(const direction of [-1,1]){
  const trial=createWheelbotActions(b,base,bundle,{seed:7});trial.setTarget({x:direction,z:.36,pitch:direction*Math.PI/18});
  let peakPen=0;const forceHistory=[];
  for(let k=0;k<2000;k++){
   if(k===1200)trial.disturb({kind,direction,strength:'strong'});
   const s=trial.step();peakPen=Math.max(peakPen,b.actionTelemetry(s.truth,s.last.u).maximumPenetrationM);
   if(s.last.externalWrench.some(v=>v!==0))forceHistory.push({timeS:s.steps*.01,wrench:s.last.externalWrench});
  }
  const s=trial.snapshot();stress.push({kind,direction,strength:'strong',failed:s.failed,phase:s.phase,final:s.truth,peakPen,forceHistory});
 }
 const p95=a=>a.sort((x,y)=>x-y)[Math.floor(a.length*.95)];
 const evidence={schema:'wheelbot-actions-evidence/v1',modelSha256:bundle.assetSha256,profileSha256:crypto.createHash('sha256').update(fs.readFileSync('assets/wheelbot/pose_profiles.json')).digest('hex'),versions:{...bundle.versions,node:process.version,wasm:b.diagnostics().version},protocol,results,strongDisturbanceStress:stress,passCount:results.filter(r=>r.pass).length,caseCount:results.length,timing:{controlP95Ms:p95(controlTimes),plantP95Ms:p95(plantTimes),scope:'Measured Node/WASM wall time; excludes diagnostics and rendering; not WCET'},passiveFall:{initial:falling.truth,final:fall.snapshot().truth,phase:fall.snapshot().phase},capabilities:{jump:false,recover:false}};
 fs.mkdirSync('evidence',{recursive:true});fs.writeFileSync('evidence/wheelbot_actions.json',JSON.stringify(evidence,null,2)+'\n');fs.writeFileSync('tests/fixtures/wheelbot_actions.json',JSON.stringify(fixture,null,2)+'\n');
 console.log(JSON.stringify({pass:evidence.passCount,total:evidence.caseCount,timing:evidence.timing}));
 assert.equal(evidence.passCount,evidence.caseCount,'Physical acceptance failures retained in evidence');
}finally{b.dispose();}
