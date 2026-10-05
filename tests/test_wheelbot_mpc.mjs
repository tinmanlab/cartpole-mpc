import fs from 'node:fs';
import assert from 'node:assert/strict';
import {createWheelbotMPC} from '../src/wheelbot_mpc.mjs';
const p=JSON.parse(fs.readFileSync('assets/wheelbot/profile.json'));
const ref=p.controlledIndices.map(i=>i<6?p.qref[i]:0),limits=p.limitsNm;
const cases=[];
for(const N of [1,20]){
 const controller=createWheelbotMPC(p,limits,{N});
 for(const scale of [.0001,.3]){
  const e0=ref.map((_,i)=>scale*Math.sin(i+1)),estimate=ref.map((v,i)=>v+e0[i]);
  const result=controller.solve(estimate,0);
  assert.equal(result.u.length,3);assert(result.U.every(u=>u.every((v,i)=>Math.abs(v)<=limits[i]+1e-8)));
  if(scale<.001){const expected=p.uref.map((v,i)=>v-p.K[i].reduce((s,k,j)=>s+k*e0[j],0));assert(Math.max(...result.u.map((v,i)=>Math.abs(v-expected[i])))<1e-7);}
  cases.push({name:scale<.001?'inactive':'constructed_active',N,p,limits,e0,result});
 }
 assert.throws(()=>controller.solve([1],0),/Invalid/);
 assert.throws(()=>controller.solve(ref,NaN),/Invalid/);
}
const off=structuredClone(p);off.R[0][1]=off.R[1][0]=.025;
const e0=ref.map((_,i)=>.01*Math.sin(i+1));
const result=createWheelbotMPC(off,limits,{N:20}).solve(ref.map((v,i)=>v+e0[i]),0);
cases.push({name:'off_diagonal_R',N:20,p:off,limits,e0,result});
assert.throws(()=>createWheelbotMPC({...p,B:p.B.map(r=>[r[0]])},limits),/Invalid/);
fs.writeFileSync('test-results/wheelbot_mpc_reference.json',JSON.stringify({cases}));
console.log('wheelbot MPC: equivalence, MIMO bounds, off-diagonal R, invalid input passed');
for(const c of cases.filter(c=>c.name==='constructed_active'))assert(c.result.u.every((v,i)=>Math.abs(Math.abs(v)-limits[i])<1e-8),'all three bounds active');
for(const motor of [1,2]){
 const changed=structuredClone(p);changed.B=changed.B.map(r=>r.map((v,i)=>i===motor?0:v));
 const z=createWheelbotMPC(changed,limits).solve(ref.map((v,i)=>v+e0[i]));
 assert(Math.abs(z.u[motor]-createWheelbotMPC(p,limits).solve(ref.map((v,i)=>v+e0[i])).u[motor])>1e-5,'all B columns matter');
}
const {createWheelbotTrial}=await import('../src/wheelbot_control.mjs');let advances=0;
const backend={assetSha256:p.assetSha256,limits,step(){advances++;throw Error('unexpected plant advance');}};
assert.throws(()=>createWheelbotTrial({...backend,assetSha256:'wrong'},p,{mode:'mpc_kf'}),/mismatch/);
const trial=createWheelbotTrial(backend,p,{mode:'mpc_kf'}),saved=globalThis.Quadprog.solveQP;
try{globalThis.Quadprog.solveQP=()=>({solution:[0,NaN],Lagrangian:[]});assert.throws(()=>trial.step(),/invalid solver command/);assert.equal(advances,0);assert.equal(trial.snapshot().steps,0);}finally{globalThis.Quadprog.solveQP=saved;}

assert(createWheelbotMPC(p,limits).solve(ref).u.every((v,i)=>Math.abs(v-p.uref[i])<1e-10),'nonzero trim feedforward');
assert.throws(()=>createWheelbotMPC(p,limits).solve(ref.map(()=>1e308)),/rejected/);
// Every malformed solver return must fail before the physical plant advances.
for(const bad of [null,undefined,{}, {solution:null,Lagrangian:null},{solution:3,Lagrangian:[]},{solution:[0,NaN],Lagrangian:[]},{message:'injected failure'}]){
 const before=trial.snapshot();
 try{globalThis.Quadprog.solveQP=()=>bad;assert.throws(()=>trial.step(),/QP rejected/);assert.deepEqual(trial.snapshot(),before);assert.equal(advances,0);}finally{globalThis.Quadprog.solveQP=saved;}
}
for(const bad of [null,undefined,{},[NaN]])assert.throws(()=>createWheelbotMPC(bad,limits),/Invalid MPC/);
for(const bad of [null,{},[NaN],ref.map(()=>NaN)])assert.throws(()=>createWheelbotMPC(p,limits).solve(bad),/Invalid MPC/);
assert.throws(()=>createWheelbotTrial(backend,p,{mode:'nmpc'}),/NOT_YET_SUPPORTED/);
const extreme=ref.map((v,i)=>v+2*Math.sin(i+1));
let rejection;
try{createWheelbotMPC(p,limits).solve(extreme);}catch(e){rejection=e.message;}
assert.match(rejection??'',/QP rejected/);
fs.writeFileSync('evidence/wheelbot_mpc_large_state_rejection.json',JSON.stringify({scope:'offline constructed state, not physical reachability',e0:extreme.map((v,i)=>v-ref[i]),accepted:false,rejection},null,2)+'\n');
const response=JSON.parse(fs.readFileSync('assets/wheelbot/response_profile.json'));
for(const N of [1,20])for(const scale of [.0001,.3]){
 const e0=ref.map((_,i)=>scale*Math.sin(i+1)),result=createWheelbotMPC(response,limits,{N}).solve(ref.map((v,i)=>v+e0[i]));
 cases.push({name:'response_'+(scale<.001?'inactive':'constructed_active'),N,p:response,limits,e0,result});
}
fs.writeFileSync('test-results/wheelbot_mpc_reference.json',JSON.stringify({cases}));
console.log('response profile, null/NaN/shape solver rejection, no-advance and large-state rejection passed');
const activeComparison=cases.filter(c=>c.name.includes('constructed_active')).map(c=>{
 const requested=c.p.uref.map((v,i)=>v-c.p.K[i].reduce((s,k,j)=>s+k*c.e0[j],0));
 const clipped=requested.map((v,i)=>Math.max(-limits[i],Math.min(limits[i],v)));
 return {name:c.name,N:c.N,mpc:c.result.u,lqrRequested:requested,lqrClipped:clipped,maxActionDifferenceNm:Math.max(...clipped.map((v,i)=>Math.abs(v-c.result.u[i])))};
});
assert(activeComparison.some(c=>c.maxActionDifferenceNm>1e-3));
fs.writeFileSync('evidence/wheelbot_mpc_active_comparison.json',JSON.stringify({scope:'offline constructed states, not physically reachable claims',rows:activeComparison},null,2)+'\n');

for(const c of cases){assert.deepEqual(c.result.forecastActiveCountsByMotor,[0,1,2].map(j=>c.result.U.filter(u=>Math.abs(u[j])>=c.limits[j]-1e-8).length));assert.equal(c.result.forecastConstraintActive,c.result.forecastActiveCountsByMotor.some(n=>n>0));}
