import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
import {createRequire} from 'node:module';
import {createMujocoBackend} from '../src/mujoco_backend.mjs';
const require=createRequire(import.meta.url),L=require('../src/engine'),Plant=require('../src/plant');
const spec=new L.LabPlant().spec;
const samples=[[0,0,0,0],[.2,.5,.4,-.7],[-.3,-.4,-.6,.8],[0,.8,1.2,1.4]];
const parameters=[{mc:1,mp:.1,l:.5},{mc:1.25,mp:.08,l:.575},{mc:.8,mp:.14,l:.42}];
const reference=[];
for(const p of parameters)for(const x of samples)for(const u of [-7,0,5]){
 const s={...spec,...p},r=Plant.integrate(x,s,{...p,friction:0},u,0,.005);
 reference.push({spec:s,x,u,acc:[r.drive.acc,r.drive.alpha],step:L.nonlinearStep(x,u,s)});
}
const backend=await createMujocoBackend();
assert.equal(backend.version,JSON.parse(fs.readFileSync('package.json')).dependencies['@mujoco/mujoco']);assert.equal(backend.assetSha256,crypto.createHash('sha256').update(fs.readFileSync('assets/cartpole.xml')).digest('hex'));
let maxAccelerationError=0,maxStepError=0,maxGeometryError=0;
for(const r of reference){
 const a=backend.acceleration(r.x,r.u,r.spec),x=backend.transition(r.x,r.u,r.spec);
 maxAccelerationError=Math.max(maxAccelerationError,...a.map((v,i)=>Math.abs(v-r.acc[i])));
 maxStepError=Math.max(maxStepError,...x.map((v,i)=>Math.abs(v-r.step[i])));
 const g=backend.geometry(r.x,r.spec),length=Math.hypot(...g.tip.map((v,i)=>v-g.pivot[i]));
 maxGeometryError=Math.max(maxGeometryError,Math.abs(length-2*r.spec.l),Math.abs(g.cart[0]-r.x[0]),Math.abs(g.tip[0]-g.pivot[0]-2*r.spec.l*Math.sin(r.x[2])),Math.abs(g.poleInertia[1]-r.spec.mp*r.spec.l*r.spec.l/3));
}
assert(maxAccelerationError<1e-9);assert(maxStepError<1e-10);assert(maxGeometryError<1e-12);
L.setPhysicsBackend(backend);
const replay={spec,x0:[0,0,.03,0],controls:Array.from({length:80},(_,k)=>.5*Math.sin(.17*k)),states:[]};let x=replay.x0.slice();
for(const u of replay.controls){x=backend.transition(x,u,spec);replay.states.push(x);}
const rows=[];
for(const controller of ['lqr','linear_mpc','hard_mpc','full_nmpc'])for(const observer of ['kf','ekf','ukf','mhe']){
 const r=L.runEpisode({controller,observer,steps:220,pushAt:100,pushForce:2,seed:43});
 assert.equal(r.physics.backend,'mujoco-wasm');assert.equal(r.failed,false,controller+'/'+observer+' nominal failure');
 rows.push({controller,observer,failed:r.failed,steps:r.steps,rmseState:r.rmseState,p95SolveMs:r.p95SolveMs});
}
// The new default hard-rail controller is tested outside nominal conditions too.
// A rejected QP is not hidden as successful control or replaced with a guessed input.
const hardStress=[];
for(const scenario of ['mixed','latency','sim2real','glitch'])for(const seed of [77,301,302,303]){
  try{
    const r=L.runEpisode({controller:'hard_mpc',observer:'ekf',scenario,seed,steps:240,pushAt:96,pushForce:3,goal:.5});
    hardStress.push({scenario,seed,outcome:r.failed?'envelope-failure':'completed',steps:r.steps,maxPosition:r.maxPosition,maxAngle:r.maxAngle});
  }catch(error){
    if(!String(error.message).startsWith('QP rejected:'))throw error;
    hardStress.push({scenario,seed,outcome:'solver-rejected',reason:error.message});
  }
}
const parameterReplays=parameters.map((p,i)=>{
  const params={...p,friction:[0,.15,.04][i]},x0=[.1,.2,.05,-.1];let x=x0.slice();
  const inputs=Array.from({length:40},(_,k)=>({u:.5*Math.sin(.17*k),external:k>=12&&k<16?1.5:0}));
  const states=inputs.map(({u,external})=>{x=backend.advance(x,u,external,spec,params);return x;});
  return {params,x0,inputs,states};
});
const drop=L.runEpisode({controller:'linear_mpc',observer:'ekf',scenario:'dropout',steps:100,pushAt:999});
assert(drop.trace.some(q=>q.sensorFresh===false));assert(drop.trace.filter(q=>!q.sensorFresh).every(q=>q.measurementUsed===false&&q.S===null));
assert.throws(()=>backend.transition([NaN,0,0,0],0,spec));assert.throws(()=>backend.transition([0,0,0,0],NaN,spec));
const receipt={schema:'cartpole-mujoco-wasm/v1',physics:backend.diagnostics(),accelerationCases:reference.length,maxAccelerationError,maxStepError,maxGeometryError,rows,replay,parameterReplays,hardStress,
 assetScope:'locally authored uniform-rod MJCF, SI, no contacts/rolling-wheel claim',hardwareVerified:false};
fs.writeFileSync('evidence/mujoco_wasm.json',JSON.stringify(receipt,null,2)+'\n');
console.log(JSON.stringify({physics:receipt.physics,accelerationCases:reference.length,maxAccelerationError,maxStepError,maxGeometryError,closedLoopRows:rows.length,allNominalPassed:rows.every(r=>!r.failed)},null,2));
backend.dispose();
