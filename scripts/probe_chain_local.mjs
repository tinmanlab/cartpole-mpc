// Diagnostic only: distinguish an extremely small noiseless local basin from
// the fixed task/noisy-estimation envelope. Does not promote any default.
import fs from 'node:fs';
import {createChainBackend} from '../src/chain_backend.mjs';
import {createChainTrial,relativeAngles} from '../src/chain_control.mjs';
const profiles=JSON.parse(fs.readFileSync('assets/chains/profiles.json','utf8')).profiles,rows=[];
for(const n of [4,5,6]){
 const p=profiles.find(p=>p.poles===n),b=await createChainBackend(fs.readFileSync(p.asset,'utf8'),n);
 try{for(const observer of ['oracle','steady_kf']){
  const x=[0,...relativeAngles(Array.from({length:n},(_,i)=>1e-7*(i%2?-.5:1))),...Array(n+1).fill(0)];
  const t=createChainTrial(b,p,{controller:'lqr',observer,initialState:x,goal:0,noise:observer==='oracle'?0:2e-4});
  let last,maxAngle=0,maxForce=0,outcome='completed';for(let k=0;k<2000;k++){last=t.step();maxAngle=Math.max(maxAngle,...last.absoluteAngles.map(Math.abs));maxForce=Math.max(maxForce,Math.abs(last.last.u));if(last.last.failed){outcome='envelope-failure';break;}}
  rows.push({poles:n,observer,initialAbsoluteAngleAmplitude:1e-7,measurementSigma:observer==='oracle'?0:2e-4,outcome,steps:last.steps,maxAngle,maxForce,finalState:last.truth});
 }}finally{b.dispose();}
}
const report={schema:'cartpole-chain-micro-local-diagnostic/v1',rows,scope:'Post-baseline diagnostic, not held-out task success. Vanishingly small noiseless oracle perturbations do not establish a useful noisy sensor recovery domain.'};
fs.writeFileSync('evidence/chain_local_diagnostic.json',JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(rows,null,2));
