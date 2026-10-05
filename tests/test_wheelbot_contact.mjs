import fs from 'node:fs';
import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import {buildConfiguredWheelbotXml} from '../src/wheelbot_configuration.mjs';
import {createWheelbotContactBackend} from '../src/wheelbot_contact_backend.mjs';
import {createContactTrial} from '../src/wheelbot_contact_trial.mjs';
const c=JSON.parse(fs.readFileSync('assets/wheelbot/contact_design.json'));const source=fs.readFileSync('assets/wheelbot/wheelbot.xml','utf8');
const variants=[['default',c],['edited',(()=>{const p=structuredClone(c);p.base.massKg=1.3;p.upper.lengthM=.3;p.motor.torqueLimitNm=[5,6,1];return p;})()]];
const raws=[],results=[];
for(const [name,config]of variants){
 const {xml,metadata}=buildConfiguredWheelbotXml(source,config);const backend=await createWheelbotContactBackend(xml);
 try{
  assert.equal(backend.modelInfo().totalMassKg,metadata.totalMassKg);
  assert.deepEqual(backend.limits,config.motor.torqueLimitNm);
  assert.equal(backend.diagnostics().nq,6);assert.equal(backend.diagnostics().nu,3);
  for(const scenario of ['fall-left','fall-right','obstacle','torque']){
   const trial=createContactTrial(backend,metadata,{scenario,torques:[50,-50,10],steps:scenario==='torque'?1:400});const initial=trial.snapshot().truth;
   const before=backend.diagnostics().steps;backend.contactDetails(initial);backend.sceneGeometry(initial);assert.equal(backend.diagnostics().steps,before);
   while(!trial.snapshot().done)trial.step();
   const pairs=new Map();let penetration=0,maxForce=0;
   for(const row of trial.history){
    for(const p of row.last.allContacts.pairs){pairs.set(p.geom1+'|'+p.geom2,(pairs.get(p.geom1+'|'+p.geom2)??0)+1);assert(p.normalForceN>=-1e-8);assert(p.forceOnGeom2WorldN.every(Number.isFinite));maxForce=Math.max(maxForce,p.normalForceN);}
    penetration=Math.max(penetration,row.last.allContacts.maximumPenetrationM);assert(row.last.u.every((v,j)=>Math.abs(v)<=backend.limits[j]));
   }
   if(scenario.startsWith('fall')){
    assert(pairs.has('floor|torso_visual'),'body-ground contact must produce a real reaction');
    assert(maxForce>1,'Recorded load-bearing contact force cannot be zero');
    assert(trial.history.some(s=>Math.abs(s.truth[2])>.6),'fall must not stop at the old local balance limit');
    assert(trial.history.length===400&&!trial.snapshot().failed);
   }
   if(scenario==='obstacle')assert([...pairs.keys()].some(k=>k.includes('obstacle')),'physical obstacle did not contact robot');
   if(scenario==='torque')assert(trial.history[0].last.saturated);
   const summary={variant:name,scenario,steps:trial.history.length,failed:trial.snapshot().failed,final:trial.snapshot().truth,pairs:Object.fromEntries(pairs),maximumPenetrationM:penetration,maximumContactNormalN:maxForce};
   results.push(summary);raws.push({variant:name,scenario,xml,metadata,initial,trace:trial.history});
  }
 }finally{backend.dispose();}
}
fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/wheelbot_contact_raw.json',JSON.stringify({raws})+'\n');
const report={schema:'wheelbot-full-contact-validation/v1',configurationSha256:crypto.createHash('sha256').update(fs.readFileSync('assets/wheelbot/contact_design.json')).digest('hex'),results,claim:'Actual body/link/ground/obstacle dynamics and parameter changes; no self-righting success claim.'};
fs.writeFileSync('test-results/wheelbot_contact_validation.json',JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(results,null,2));
