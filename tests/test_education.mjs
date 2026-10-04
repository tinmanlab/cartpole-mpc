// Educational claims must match the executed model, not only static prose.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {createRequire} from 'node:module';
import {createMujocoBackend} from '../src/mujoco_backend.mjs';
const require=createRequire(import.meta.url),L=require('../src/engine');
const {CONTROL_LAB_TOPICS:topics,CONTROL_LAB_TOPIC_ORDER:order}=require('../src/topics');
const backend=await createMujocoBackend();L.setPhysicsBackend(backend);
const results=[];
function check(name,fn){try{fn();results.push({name,passed:true});}catch(e){results.push({name,passed:false,error:e.message});}}
try{
 const spec=new L.LabPlant().spec;
 check('all lessons disclose implementation scope and sources',()=>{
  assert.equal(order.length,28);
  for(const key of order){const t=topics[key];assert(['implemented','analogue','paper','concept'].includes(t.scope),key);assert(t.sources?.length,key);assert(!t.body.includes('\\n'),'literal backslash-n in '+key);}
 });
 check('local approximate optimizer does not claim convergence',()=>{
  for(const id of ['ltv_mpc','state_mpc']){const c=L.makeController(id,spec);c.reset();c.act([.2,.1,.05,0],.5);assert.equal(c.lastConverged,null,id);assert.equal(typeof c.lastUpdateAccepted,'boolean');}
 });
 check('reduced plan does not fabricate a four-state trajectory',()=>{
  const c=new L.CentroidalMPCController(spec);c.reset();c.act([.2,.1,.05,0],.5);
  assert.equal(c.lastPrediction.length,0);assert.equal(c.lastPredictionReduced.length,33);assert.equal(c.predictionSpace,'reduced-com');
 });
 check('per-component estimation error is separate from tracking',()=>{
  const r=L.runEpisode({controller:'lqr',observer:'truth',steps:10,goal:.5,pushAt:999});
  assert.deepEqual(r.estimationRmseByState,[0,0,0,0]);assert(r.positionTrackingRmse>0);
  assert.deepEqual(r.stateUnits,['m','m/s','rad','rad/s']);assert.equal(r.legacyRmseScope,'mixed-units; not a controller score');
 });
 check('scenario teaching matches actual model count and local solver',()=>{
  const c=new L.ScenarioMPCController(spec);assert.equal(c.models.length,5);
  assert(topics.robust_mpc.body.includes('5'));assert.equal(topics.robust_mpc.runtime.controller,'scenario_mpc');
 });
 check('theoretical branches are not renamed local reproductions',()=>{
  assert.equal(topics.inekf.scope,'analogue');assert.equal(topics.coco.scope,'paper');
  assert.equal(topics.focus.scope,'analogue');assert.equal(topics.youm.scope,'paper');
  assert(!topics.ltv_mpc.title.includes('SQP-RTI'));
  assert(topics.ukf.body.includes('H P_minus H'));assert(topics.mhe.body.includes('0.16'));
  assert(topics.ppo.body.includes('모방')&&topics.ppo.body.includes('보장'));
 });
 const beforePath='tests/fixtures/education_controls.json';
 check('diagnostic corrections preserve sampled control outputs',()=>{
  for(const old of JSON.parse(fs.readFileSync(beforePath,'utf8')).rows){const c=L.makeController(old.name,spec);c.reset();let x=[.2,.1,.05,-.1];for(const expected of old.U){const actual=c.act(x,.5);assert(Math.abs(actual-expected)<=1e-12,old.name);x=backend.transition(x,actual,spec);}}
 });
 fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/education_contracts.json',JSON.stringify({results},null,2));
 console.log(JSON.stringify({results},null,2));if(results.some(r=>!r.passed))process.exitCode=1;
}finally{backend.dispose();}
