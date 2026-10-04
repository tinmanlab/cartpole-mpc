// Explicit continuous/conditional search path; original nine-point API remains strict.
import assert from 'node:assert/strict';import fs from 'node:fs';import{createRequire}from'node:module';
import{createMujocoBackend}from'../src/mujoco_backend.mjs';
const require=createRequire(import.meta.url),L=require('../src/engine'),D=require('../src/design_study');
const task=JSON.parse(fs.readFileSync('tests/fixtures/design_study.json'));const protocol=JSON.parse(fs.readFileSync('tests/fixtures/sequential_tuning.json'));
const b=await createMujocoBackend();L.setPhysicsBackend(b);
try{
 const fit=D.calibrate(task).fit,pair=task.pairs.find(p=>p.id==='mpc-ekf');
 const candidate={id:'search',effortMultiplier:.75,processMultiplier:.8,horizon:20};
 assert.throws(()=>D.createRun(task,task.training[0],pair,candidate,fit),/candidate/i);
 const run=D.createRun(task,task.training[0],pair,candidate,fit,{searchDomain:protocol.domain,record:true});
 run.step();const r=run.result();assert.equal(r.design.horizon,20);assert.equal(r.design.Rc,.75/9);assert.equal(r.design.Qe[0],task.theoryDesign.QeBase[0]*.8);
 assert.equal(r.information.controller,'observer-output only');assert.equal(r.appliedSteps,1);
 assert.throws(()=>D.createRun(task,task.training[0],task.pairs[0],candidate,fit,{searchDomain:protocol.domain}),/horizon|inactive/i);
 for(const c of [{...candidate,effortMultiplier:.49},{...candidate,processMultiplier:NaN},{...candidate,horizon:25}])assert.throws(()=>D.createRun(task,task.training[0],pair,c,fit,{searchDomain:protocol.domain}));
 const old=D.candidates(task).find(c=>c.baseline);
 for(const p of task.pairs){
  const previous=D.createRun(task,task.training[0],p,old,fit,{record:true});
  const newer=D.createRun(task,task.training[0],p,{...old,...(p.controller==='hard_mpc'?{horizon:30}:{})},fit,{record:true,searchDomain:protocol.domain});
  for(let i=0;i<30;i++){previous.step();newer.step();}
  assert.deepEqual(previous.result().auditSeries,newer.result().auditSeries,'baseline changed through optional adapter');
 }
 console.log('Continuous/conditional tuning adapter: strict legacy API, bounded factors, inactive horizon and baseline numerical parity PASS');
}finally{b.dispose();}
