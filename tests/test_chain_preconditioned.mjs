// Same finite-horizon problem, different coordinates; not retuned control or relaxed bounds.
import fs from 'node:fs';import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {createPrestabilizedQP} from '../src/chain_preconditioned.mjs';
const L=createRequire(import.meta.url)('../src/engine.js');
const ps=JSON.parse(fs.readFileSync('assets/chains/profiles.json','utf8')).profiles;
const cases=[],max=(x)=>Math.max(...x.map(Math.abs));
for(const p of ps.filter(p=>p.designAvailable)){
 const s=createPrestabilizedQP(p),x=Array(p.nx).fill(0);x[1]=.003;
 const r=s.solve(x,.1);assert(r.accepted,'A finite feasible test plan must be verified');
 assert(r.kktResidual<=1e-7&&r.primalResidual<=1e-8,'same absolute QP acceptance thresholds');
 assert(r.physicalViolation<=1e-8&&r.dynamicsDefect<=1e-8);
 assert.equal(r.U.length,p.horizon);assert.equal(r.X.length,p.horizon+1);
 const e=x.slice();e[0]-=.1;const J=L.linearQuadraticCost(r.X.map(z=>{const q=z.slice();q[0]-=.1;return q;}),r.U,p.Q,p.R,p.P);
 assert(Math.abs(J-r.J)<=1e-7*Math.max(1,Math.abs(J)),'original objective not preserved');
 const probeV=Array.from({length:p.horizon},(_,i)=>.01*Math.sin(i*.31));
 const distantError=Array(p.nx).fill(0);distantError[0]=-3;
 assert(s.evaluateCoordinates(distantError,Array(p.horizon).fill(0)).costIdentityRelativeError<1e-7,'reference error is not a world rail coordinate');
 const c=s.evaluateCoordinates(e,probeV);
 assert(c.costIdentityRelativeError<=1e-7,'exact transformed/original cost for arbitrary decision vector');
 assert(c.transformError<=1e-8,'u -> v -> u is bijective');
 if(p.poles<=3){const old=L.solveBoxLinearMpc(p.A,p.B,p.Q,p.R,p.P,e,Array(p.horizon).fill(0),10,30,{positionLimit:2.4,goal:.1});assert(Math.abs(old.U[0]-r.U[0])<.001);}
 cases.push({poles:p.poles,x0:x,goal:.1,profile:p,result:r,probeV,coordinateCheck:c,condensed:s.exportProblem(x,.1)});
 assert.throws(()=>s.solve([0,0],0),/state/);assert.throws(()=>s.solve(Array(p.nx).fill(NaN),0),/state/);
 const outside=Array(p.nx).fill(0);outside[0]=2.5;assert.throws(()=>s.solve(outside,0),/rail/);
}
for(const [label,x0,rail] of [['active-force',[0,.3,0,0],2.4],['active-rail',[.3,.08,.1,-.05],.35]]){
 const profile={...ps[0],railLimit:rail},solver=createPrestabilizedQP(profile),result=solver.solve(x0,0);
 assert(Math.max(...result.U.map(Math.abs))>9.999);if(label==='active-rail')assert(Math.max(...result.X.map(x=>Math.abs(x[0])))>.34999);
 cases.push({label,poles:1,x0,goal:0,profile,result,condensed:solver.exportProblem(x0,0)});
}
for(const p of ps.filter(p=>!p.designAvailable))assert.throws(()=>createPrestabilizedQP(p),/design/i);
fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/chain_preconditioned_fixture.json',JSON.stringify({cases},null,2)+'\n');
console.log('Prestabilized QP: full original cost, transformed physical constraints, unchanged residual thresholds, native fixture PASS');
