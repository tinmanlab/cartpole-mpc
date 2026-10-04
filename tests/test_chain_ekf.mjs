// New dimension-aware observer reproduces the existing EKF equations using the actual WASM map.
import fs from 'node:fs';import assert from 'node:assert/strict';
import {ChainEKF} from '../src/chain_observer.mjs';
import {createChainBackend} from '../src/chain_backend.mjs';
const ps=JSON.parse(fs.readFileSync('assets/chains/profiles.json','utf8')).profiles,rows=[];
for(const p of ps.filter(p=>[1,3,6].includes(p.poles))){
 const b=await createChainBackend(fs.readFileSync(p.asset,'utf8'),p.poles);
 try{
  const y=Array(p.dof).fill(0);y[1]=.02;const o=new ChainEKF(b,p,y),before=o.snapshot();
  assert.equal(before.x.length,p.nx);assert.equal(before.P.length,p.nx);
  assert.throws(()=>o.step([0],0),/measurement/);assert.deepEqual(o.snapshot(),before);
  const prediction=b.step(before.x,.15),F=b.linearize(before.x,.15).A;
  const nextY=prediction.slice(0,p.dof).map((v,i)=>v+(i%2?.0001:-.0001));const result=o.step(nextY,.15);
  assert(result.x.every(Number.isFinite));assert(result.last.measurementUsed);assert(result.last.nis>=0);
  rows.push({profile:p,before,prediction,F,measurement:nextY,after:result});
  const no=o.step(null,0);assert.equal(no.last.measurementUsed,false);assert.equal(no.last.nis,null);assert.equal(no.last.S,null);
  assert(no.x.every(Number.isFinite));assert(o.step(nextY,0).last.measurementUsed);
 }finally{b.dispose();}
}
fs.writeFileSync('test-results/chain_ekf_fixture.json',JSON.stringify({rows},null,2)+'\n');
console.log('N-dimensional EKF: same-time update, invalid measurement atomic rejection, missingness and native fixture PASS');
