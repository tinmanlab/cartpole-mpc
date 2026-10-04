// Compare actual arbitrary-dimension WASM models, coordinate maps and linearization.
import fs from 'node:fs';
import assert from 'node:assert/strict';
import {createChainBackend} from '../src/chain_backend.mjs';
import {createMujocoBackend} from '../src/mujoco_backend.mjs';
const profiles=JSON.parse(fs.readFileSync('assets/chains/profiles.json','utf8')).profiles;
const rows=[];
for(const p of profiles){
 const b=await createChainBackend(fs.readFileSync(p.asset,'utf8'),p.poles);
 try{
  assert.equal(b.nx,2*(p.poles+1));assert.equal(b.nu,1);assert.equal(b.assetSha256,p.assetSha256);
  const x=Array(b.nx).fill(0);x[0]=.1;for(let i=1;i<=p.poles;i++)x[i]=.005*(i%2?1:-1);
  const lin=b.linearize(),err=(A,B)=>Math.max(...A.flatMap((r,i)=>r.map((v,j)=>Math.abs(v-B[i][j]))));
  const errorA=err(lin.A,p.A),errorB=err(lin.B,p.B);assert(errorA<1e-7&&errorB<1e-7,'native vs WASM derivative');
  const ge=b.geometry(x);assert.equal(ge.links.length,p.poles);
  let angle=0,point=[x[0],0,.18];let geometryError=0;
  for(let i=0;i<p.poles;i++){angle+=x[i+1];point=[point[0]+Math.sin(angle),0,point[2]+Math.cos(angle)];for(let j=0;j<3;j++)geometryError=Math.max(geometryError,Math.abs(point[j]-ge.links[i].tip[j]));}
  assert(geometryError<1e-10);
  assert.throws(()=>b.step([0,0],0),/dimension/);assert.throws(()=>b.step(x,NaN),/force/);
  const controls=Array.from({length:50},(_,k)=>.02*Math.sin(.2*k));let z=x.slice();const X=controls.map(u=>z=b.step(z,u,0));
  rows.push({poles:p.poles,asset:p.asset,assetSha256:p.assetSha256,x0:x,controls,states:X,errorA,errorB,geometryError});
  if(p.poles===1){const old=await createMujocoBackend();const spec={actuator:'ideal',mc:1,mp:.1,l:.5,gravity:9.8};const legacy=old.transition([x[0],x[2],x[1],x[3]],.2,spec);const next=b.step(x,.2);assert(Math.max(...next.map((v,i)=>Math.abs(v-legacy[[0,2,1,3][i]])))<1e-12);old.dispose();}
 }finally{b.dispose();}
}
fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/chain_wasm.json',JSON.stringify({rows},null,2)+'\n');
console.log(JSON.stringify(rows.map(({states,controls,x0,...r})=>r),null,2));
