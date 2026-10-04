import fs from 'node:fs';import assert from 'node:assert/strict';
import {createChainBackend} from '../src/chain_backend.mjs';
import {createChainTrial,relativeAngles,absoluteAngles} from '../src/chain_control.mjs';
const p=JSON.parse(fs.readFileSync('assets/chains/profiles.json','utf8')).profiles[1];
const b=await createChainBackend(fs.readFileSync(p.asset,'utf8'),2);
try{
 const x=[.1,...relativeAngles([.01,-.02]),0,0,0];assert.deepEqual(absoluteAngles(x,2).map(v=>+v.toFixed(8)),[.01,-.02]);
 for(const opts of [{controller:'ppo'},{observer:'mhe'},{initialState:[0,0,0,0]}])assert.throws(()=>createChainTrial(b,p,opts));
 assert.throws(()=>createChainTrial(b,{...p,assetSha256:'invalid'}),/identity/);
 assert.throws(()=>createChainTrial(b,{...p,designAvailable:false,reason:'test rejection'}),/Design rejected/);
 const t=createChainTrial(b,p,{initialState:x});const before=t.snapshot();
 assert.throws(()=>t.step(NaN),/force/);assert.deepEqual(t.snapshot().truth,before.truth);assert.equal(t.snapshot().steps,0);
 const oracle=createChainTrial(b,p,{observer:'oracle',initialState:x});const o=oracle.step();assert.deepEqual(o.estimate,o.truth);
 const result=t.step();assert.equal(result.steps,1);assert.equal(result.estimate.length,6);assert.equal(result.observer,'steady_kf');
 assert(result.last.kktResidual<=1e-7&&result.last.primalResidual<=1e-8);
 console.log('N-chain dimension, identity, oracle boundary, angle map and no-invalid-force contracts PASS');
}finally{b.dispose();}
