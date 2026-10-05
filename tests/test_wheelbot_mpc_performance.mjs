// Optional saved baseline enables paired host timing; no timing-dependent pass gate.
import fs from 'node:fs';
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
import {createWheelbotMPC} from '../src/wheelbot_mpc.mjs';
const baseline=process.argv[2];
const old=baseline?(await import('data:text/javascript;base64,'+Buffer.from(fs.readFileSync(baseline,'utf8').replace("'../vendor/quadprog/quadprog.js'",JSON.stringify(pathToFileURL(process.cwd()+'/vendor/quadprog/quadprog.js').href))).toString('base64'))).createWheelbotMPC:createWheelbotMPC;
const stats=values=>{const v=values.slice().sort((a,b)=>a-b);return {n:v.length,p50:v[Math.floor(v.length*.5)],p95:v[Math.floor(v.length*.95)],p99:v[Math.floor(v.length*.99)],max:v.at(-1),over10ms:v.filter(x=>x>10).length};};
const receipt={scope:'paired deterministic host observations, not WCET',baseline:baseline??'self',tolerances:{trajectory:1e-8,relativeObjective:1e-10},rows:[],rejectionDiscrepancies:0,maxTrajectoryDifference:0};
const traces=baseline&&fs.existsSync('test-results/wheelbot_mpc_response_traces.json')?JSON.parse(fs.readFileSync('test-results/wheelbot_mpc_response_traces.json')).rows:[];
for(const name of ['baseline','response']){
 const p=JSON.parse(fs.readFileSync(`assets/wheelbot/${name==='baseline'?'profile':'response_profile'}.json`)),ref=p.controlledIndices.map(i=>i<6?p.qref[i]:0);
 for(const N of [1,20]){
  const factories=[old,createWheelbotMPC],construct=[[],[]],times=[[],[]];
  for(let i=0;i<24;i++)for(const side of i%2?[1,0]:[0,1]){const t=performance.now();factories[side](p,p.limitsNm,{N});if(i>=4)construct[side].push(performance.now()-t);}
  const controllers=factories.map(f=>f(p,p.limitsNm,{N}));
  const inputs=[.0001,.01,.3,2].map(scale=>({estimate:ref.map((v,i)=>v+scale*Math.sin(i+1)),goal:0}));
  const physical=traces.filter(r=>r.profile===name&&r.mode==='mpc_kf').flatMap(r=>r.trace.filter((_,i)=>i%40===0).map(s=>({estimate:s.estimate,goal:s.goal}))).slice(0,80);
  inputs.push(...physical);
  for(let i=0;i<Math.max(100,inputs.length*3);i++){
   const input=inputs[i%inputs.length],results=[];
   for(const side of i%2?[1,0]:[0,1]){const t=performance.now();try{results[side]=controllers[side].solve(input.estimate,input.goal);}catch(e){results[side]={error:e.message};}if(i>=20)times[side].push(performance.now()-t);}
   assert.equal(Boolean(results[0].error),Boolean(results[1].error),'rejection classification');
   if(results[0].error)continue;
   for(const key of ['E','U']){const a=results[0][key].flat(),b=results[1][key].flat(),delta=Math.max(...a.map((v,j)=>Math.abs(v-b[j])));receipt.maxTrajectoryDifference=Math.max(receipt.maxTrajectoryDifference,delta);assert(delta<=1e-8,key);}
   assert(Math.abs(results[0].J-results[1].J)<=1e-10*Math.max(1,Math.abs(results[0].J)));
   for(const key of ['forecastActiveCountsByMotor','forecastConstraintActive','saturated'])assert.deepEqual(results[0][key],results[1][key]);
  }
  receipt.rows.push({profile:name,N,physicalEstimates:physical.length,construction:{old:stats(construct[0]),new:stats(construct[1])},solve:{old:stats(times[0]),new:stats(times[1])}});
 }
}
if(baseline)fs.writeFileSync('evidence/wheelbot_mpc_performance.json',JSON.stringify(receipt,null,2)+'\n');
console.log(JSON.stringify(receipt,null,2));
