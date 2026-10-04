// Actual-WASM terminal-cost contracts. No new optimizer or hidden test-only Qf assignment.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
import {createRequire} from 'node:module';
import {createMujocoBackend} from '../src/mujoco_backend.mjs';
const L=createRequire(import.meta.url)('../src/engine.js');
const backend=await createMujocoBackend();L.setPhysicsBackend(backend);
try{
 const spec=new L.LabPlant().spec;
 const legacy=new L.LinearMPCController(spec);
 assert.deepEqual(legacy.Qf,L.diag([8,1.5,110,7]),'Default numerical formulation must not change');
 const cases=[{}, {R:.05,Qdiag:[3,.8,90,5]}, {R:.4,Qdiag:[1,.2,45,2]}, {spec:{...spec,mc:1.2,mp:.12,l:.55}}];
 const rows=cases.map((opts,id)=>{
  const c=new L.LinearMPCController(opts.spec??spec,{...opts,terminalCost:'dare'});
  assert(Math.abs(c.Qf[0][1])>1,'Full Riccati cross terms must not be discarded');
  assert.equal(c.terminalInfo.kind,'dare');assert.equal(c.terminalInfo.source,'nominal-discrete-riccati');
  assert.equal(c.terminalInfo.converged,true);assert(c.terminalInfo.normalizedResidual<=1e-9);
  const before=JSON.stringify(c.Qf),info=JSON.stringify(c.terminalInfo),x=[.3,.1,.08,-.05];
  c.act(x,0);c.reset();c.act(x,0);
  assert.equal(JSON.stringify(c.Qf),before);assert.equal(JSON.stringify(c.terminalInfo),info,'Do not recompute Riccati per tick');
  return {id,spec:opts.spec??spec,A:c.A,B:c.B,Q:c.Q,R:c.R,Qf:c.Qf,N:c.N,limit:c.limit,x,U:c.lastControls,J:c.lastCost,terminalInfo:c.terminalInfo};
 });
 assert.notDeepEqual(rows[0].Qf,rows[1].Qf,'Actual Q/R must affect P');
 assert.throws(()=>new L.LinearMPCController(spec,{terminalCost:'typo'}),/terminal/i);
 assert.throws(()=>new L.LinearMPCController(spec,{terminalCost:'dare',Qfdiag:[1,1,1,1]}),/conflict/i);
 for(const opts of [{R:0},{R:NaN},{Qdiag:[1,-1,1,1]},{Qdiag:[1,1]},{Qdiag:[1,1,1,Infinity]}])
  assert.throws(()=>new L.LinearMPCController(spec,{...opts,terminalCost:'dare'}));
 assert.throws(()=>L.riccatiTerminal(rows[0].A,rows[0].B,rows[0].Q,rows[0].R,{maxIter:1}),/converg/i);
 assert.throws(()=>L.riccatiTerminal(rows[0].A,rows[0].B,L.diag([-1,1,1,1]),.14),/positive/i);
 assert.throws(()=>L.makeController('lqr',spec,null,{terminalCost:'dare'}),/unsupported/i);
 const hard=L.makeController('hard_mpc',spec,null,{terminalCost:'dare'});
 hard.act([.1,0,.02,0],.5);assert(hard.lastPrediction.every(x=>Math.abs(x[0])<=2.4+1e-8));
 assert.throws(()=>hard.act([2.5,0,0,0],.5),/rail/i);
 assert.throws(()=>hard.act([0,0,NaN,0],0),/state/i);
 assert.throws(()=>hard.act([0,0,0,0],Infinity),/state|goal/i);
 const run=L.runEpisode({controller:'hard_mpc',observer:'ekf',controllerOpts:{terminalCost:'dare'},steps:10,pushAt:999});
 assert.equal(run.terminalCost.kind,'dare');assert(run.trace.every(q=>q.terminalCost==='dare'));
 const custom=new L.LinearMPCController(spec,{Qfdiag:[1,2,3,4]});assert.deepEqual(custom.Qf,L.diag([1,2,3,4]));
 assert.equal(custom.terminalInfo.kind,'original');
 const sources=['src/engine.js','src/plant.js','src/qp.js','src/mujoco_backend.mjs','assets/cartpole.xml','tests/test_terminal.mjs'];
 fs.mkdirSync('test-results',{recursive:true});
 fs.writeFileSync('test-results/terminal_fixture.json',JSON.stringify({cases:rows,physics:backend.diagnostics(),sourceSha256:Object.fromEntries(sources.map(p=>[p,crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex')]))},null,2)+'\n');
 console.log('terminal option: actual WASM, full matrix, convergence, rejection, metadata and unchanged-default contracts PASS');
}finally{backend.dispose();}
