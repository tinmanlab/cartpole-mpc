import fs from 'node:fs';
import assert from 'node:assert/strict';
import {createTrackingController,loadTrackingBundle,createTrackingTrial,hashText} from '../src/wheelbot_contact_tracking.mjs';
import {mountDesignEditor} from '../src/wheelbot_design_view.mjs';
import {buildConfiguredWheelbotXml} from '../src/wheelbot_configuration.mjs';
import {createWheelbotContactBackend} from '../src/wheelbot_contact_backend.mjs';
const read=p=>fs.readFileSync(p,'utf8'),json=p=>JSON.parse(read(p));
const args={profileText:read('assets/wheelbot/contact_tracking_profile.json'),xml:read('assets/wheelbot/contact_model.xml'),costText:read('assets/wheelbot/contact_profile.json'),protocolText:read('tests/fixtures/wheelbot_contact_tracking.json'),traceSha256:json('assets/wheelbot/contact_tracking_profile.json').identities.trace_sha256};
const p=JSON.parse(args.profileText),bundle=await loadTrackingBundle(args),c=bundle.controller,x=c.initial(1);
for(const values of [[x,-1,1],[x,250,1],[x,.5,1],[x,0,0],[x,0,1,'standing'],[Array(12).fill(NaN),0,1],[x.slice(1),0,1],[x,0,1,'tracking',1]])assert.throws(()=>c.command(...values));
for(const key of Object.keys(p.identities))assert.throws(()=>createTrackingController(p,{...p.identities,[key]:'bad'}));
for(const mutate of [p=>p.steps=249,p=>p.branchDiagnostics.selected_derivatives_validated=1,p=>p.controlledIndices[0]=12,p=>p.limitsNm[0]=160,p=>p.trajectories['1'].K.pop(),p=>p.trajectories['1'].K[0][0][0]=NaN,p=>p.trajectories['1'].u[0][0]=17]){const bad=structuredClone(p);mutate(bad);assert.throws(()=>createTrackingController(bad,bundle.identities));}
await assert.rejects(loadTrackingBundle({...args,xml:args.xml+'\n'}));
await assert.rejects(loadTrackingBundle({...args,costText:args.costText+'\n'}));
await assert.rejects(loadTrackingBundle({...args,protocolText:args.protocolText+'\n'}));
const defaults=json('assets/wheelbot/contact_design.json'),template=read('assets/wheelbot/wheelbot.xml');
assert.equal(buildConfiguredWheelbotXml(template,defaults).xml,args.xml);
const changed=structuredClone(defaults);changed.base.massKg=1.3;
const editedXml=buildConfiguredWheelbotXml(template,changed).xml;
await assert.rejects(loadTrackingBundle({...args,xml:editedXml}));
const edited=await createWheelbotContactBackend(editedXml);
try{const before=edited.diagnostics().steps;assert.throws(()=>createTrackingTrial(edited,bundle));assert.equal(edited.diagnostics().steps,before);}finally{edited.dispose();}
assert.throws(()=>bundle.protocol.variants[0].offset=9);
const backend=await createWheelbotContactBackend(args.xml),native=json('test-results/tracking-native-runtime.json'),results=[];
let maxStateError=0,maxCommandError=0,maxMetricError=0,positiveGroundForce=false;
assert.deepEqual(new Set(native.map(t=>[t.sign,t.variant.name,t.controller].join('/'))),new Set([-1,1].flatMap(sign=>bundle.protocol.variants.flatMap(v=>['feedforward','tvlqr'].map(mode=>[sign,v.name,mode].join('/'))))));
assert.equal(native.length,20);assert.equal(new Set(native.map(t=>[t.sign,t.variant.name,t.controller].join('/'))).size,20);
try{
 const before=backend.diagnostics().steps;
 assert.throws(()=>createTrackingTrial(backend,{...bundle,protocol:{...bundle.protocol,variants:[{name:'invented',index:3,offset:0}]}},{variant:'invented'}));
 assert.throws(()=>createTrackingTrial({...backend,assetSha256:'wrong'},bundle));assert.equal(backend.diagnostics().steps,before);
 assert.throws(()=>createTrackingTrial(backend,bundle,{variant:'arbitrary'}));
 for(const u of [[17,0,0],[0,NaN,0],[0,0,Infinity],[0,0]])assert.throws(()=>backend.physicsStep(x,u));
 assert.throws(()=>backend.physicsStep(Array(12).fill(NaN),[0,0,0]));
 assert.equal(backend.diagnostics().steps,before);
 for(const t of native){
  const trial=createTrackingTrial(backend,bundle,{sign:t.sign,variant:t.variant.name,feedback:t.controller==='tvlqr'}),start=backend.diagnostics().steps;
  for(const force of [1,-1,NaN,Infinity,null]){assert.throws(()=>trial.step(force));assert.equal(backend.diagnostics().steps,start);assert.equal(trial.snapshot().steps,0);}
  assert.deepEqual(trial.snapshot().truth,t.initial_state);const corrupt=trial.snapshot();corrupt.truth.fill(99);for(const force of [1,-1,NaN,Infinity,null]){assert.throws(()=>trial.step(force));assert.equal(backend.diagnostics().steps,start);assert.equal(trial.snapshot().steps,0);}
  assert.deepEqual(trial.snapshot().truth,t.initial_state);
  for(let k=0;k<250;k++){
   const s=trial.step(),r=t.history[k];assert.equal(s.elapsedSeconds,(k+1)*.002);assert.equal(backend.diagnostics().steps-start,k+1);assert(s.last.externalForcesZero);assert(s.last.u.every((v,i)=>Math.abs(v)<=[16,16,1.7][i]));
   maxStateError=Math.max(maxStateError,...s.truth.map((v,i)=>Math.abs(v-r.after[i])));
   maxMetricError=Math.max(maxMetricError,Math.abs(s.last.edgeRiseM-r.edge_rise_m));
   maxCommandError=Math.max(maxCommandError,...s.last.u.map((v,i)=>Math.abs(v-r.tau[i])));
   positiveGroundForce ||= s.last.allContacts.pairs.some(p=>p.normalForceN>1);
  }
  assert.equal(backend.diagnostics().steps-start,250);assert.throws(()=>trial.step());assert.equal(backend.diagnostics().steps-start,250);
  const isolated=trial.history;isolated[0].after.fill(999);assert.notEqual(trial.history[0].after[0],999);
  const s=trial.snapshot();assert.equal(s.trackingResult.tracking_success,t.metrics.tracking_success);
  for(const key of ['final_window_angle_error_rad','final_window_pitch_speed_rad_s','overshoot_rad','edge_rise_m','max_penetration_m','max_joint_excursion_rad'])maxMetricError=Math.max(maxMetricError,Math.abs(s.trackingResult[key]-t.metrics[key==='overshoot_rad'?'rotation_overshoot_rad':key]));
  results.push({sign:t.sign,variant:t.variant.name,controller:t.controller,steps:s.steps,initial:s.initial,end:s.truth,metrics:s.trackingResult});
 }
 assert(maxStateError<1e-7,`state ${maxStateError}`);assert(maxCommandError<1e-7,`command ${maxCommandError}`);assert(maxMetricError<1e-7);assert(positiveGroundForce);
 const sourceHashes={};for(const path of ['src/wheelbot_backend.mjs','src/wheelbot_contact_backend.mjs','src/wheelbot_contact_tracking.mjs','tests/test_wheelbot_contact_tracking_wasm.mjs','test-results/tracking-native-runtime.json'])sourceHashes[path]=await hashText(read(path));
 const report={schema:'wheelbot-contact-tracking-wasm/v1',actualWasm:true,trials:20,physicsSteps:backend.diagnostics().fastPhysicsSteps,dt:.002,maxStateError,maxCommandError,maxMetricError,positiveGroundForce,feedbackPasses:results.filter(r=>r.controller==='tvlqr'&&r.metrics.tracking_success).length,feedforwardPasses:results.filter(r=>r.controller==='feedforward'&&r.metrics.tracking_success).length,profileSha256:bundle.profileSha256,identities:bundle.identities,sourceHashes,results,browserExecuted:false};
 fs.writeFileSync('test-results/wheelbot_contact_tracking_wasm.json',JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({...report,results:undefined,sourceHashes:undefined}));
}finally{backend.dispose();}

// Exercise the editor's real admission/restore path with an in-memory DOM.
const fields=new Map();globalThis.document={getElementById(id){if(!fields.has(id))fields.set(id,{value:id==='physical-tracking-mode'?'tvlqr':'0',disabled:true,classList:{toggle(){}},querySelectorAll:()=>[]});return fields.get(id);}};
let active=false,current=null,loaded=args;
const editor=mountDesignEditor({template,defaults,active:()=>active,readState:()=>current.snapshot(),loadProfile:async()=>{throw Error('No local standing profile in this test');},loadTrackingSources:async()=>loaded,applyToView:(b,t)=>{active=true;current=t;},activateTrial:t=>{current=t;},restoreView:()=>{active=false;}});
await editor.apply(defaults);assert(editor.getState().trackingValid);assert(!document.getElementById('physical-track-left').disabled);
editor.startTracking(-1);assert(current.snapshot().tracking);assert.equal(current.snapshot().steps,0);current.step();assert.equal(current.snapshot().elapsedSeconds,.002);
await editor.apply(changed);assert(!editor.getState().trackingValid);const retained=current.snapshot();assert.throws(()=>editor.startTracking(1));assert.deepEqual(current.snapshot(),retained);
assert(document.getElementById('physical-track-right').disabled);assert(!document.getElementById('physical-fall-right').disabled);
loaded={...args,profileText:JSON.stringify({...p,steps:249})};await editor.apply(defaults);assert(!editor.getState().trackingValid);assert(!document.getElementById('physical-fall-left').disabled);
loaded=args;await editor.apply(defaults);assert(editor.getState().trackingValid);editor.restore();assert(document.getElementById('physical-track-left').disabled);assert(!active);
console.log('Shared editor admission, expired profile isolation, changed XML rejection and restore PASS');
