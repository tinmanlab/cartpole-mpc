import fs from 'node:fs';import assert from 'node:assert/strict';
import {createGameJump}from '../src/wheelbot_game_jump.mjs';
import {createWheelbotContactBackend}from '../src/wheelbot_contact_backend.mjs';
const read=p=>JSON.parse(fs.readFileSync(p));const base=read('assets/wheelbot/live_profile.json'),bundle=read('assets/wheelbot/target_jump.json');
const b=await createWheelbotContactBackend(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'));const rows=[];
try{
 const planner=createGameJump(b,base,bundle),entry=bundle.profiles.find(p=>p.id===1).entry;
 const e=[...entry.slice(0,5),...entry.slice(6)],low=planner.start(e,0).acceptedHeight,high=planner.start(e,1).acceptedHeight;
 assert(Math.abs(planner.start(e,.25).acceptedHeight-(low+.25*(high-low)))<1e-6,'Hold duration must request an intermediate height, not just two tiers');
 for(const seed of [7,42])for(const fraction of [0,.25,.5,.75,1])for(const vx of [-.2,0,.2]){
  let rng=seed;const uniform=()=>{rng=(Math.imul(rng,1664525)+1013904223)>>>0;return(rng+.5)/4294967296;};
  const measure=x=>base.measurementIndices.map((j,i)=>x[j]+base.measurementSigma[i]*Math.sqrt(-2*Math.log(uniform()))*Math.cos(2*Math.PI*uniform()));
  let x=entry.slice();x[6]=vx;x[11]=vx/.05;let estimate=[...x.slice(0,5),...x.slice(6)];const plan=planner.start(estimate,fraction);
  let clearance=0,flight=0,flightS=0,pen=0,exc=0,spin=0;
  for(let k=0;k<400;k++){const u=plan.command(k,estimate).u;x=b.stepWrench(x,u,[0,0,0],true);estimate=plan.observe(k,estimate,u,measure(x));const d=b.lastActionStep();clearance=Math.max(clearance,d.maximumWheelClearanceM);pen=Math.max(pen,d.maximumPenetrationM);exc=Math.max(exc,d.jointLimitExcursionRad);spin=Math.max(spin,d.maximumAirborneWheelRate);for(const s of d.flightSamples){flight=s.air?flight+.002:0;flightS=Math.max(flightS,flight);}assert(pen<=.01&&exc<=.02&&spin<=25&&Math.abs(x[2])<=.6);}
  const r={seed,fraction,vx,targetHeightM:plan.acceptedHeight,actualClearanceM:clearance,heightErrorM:Math.abs(clearance-plan.acceptedHeight),flightS,pen,exc,spin,finalVx:x[6]};
  assert(r.heightErrorM<=.003&&flightS>=.03&&Math.abs(x[6]-vx)<=.04);rows.push(r);
 }
 const report={passed:true,trials:rows.length,engine:b.diagnostics().version,rows,scope:'Continuous interpolation at five tested charge fractions and three planar velocities with noisy measurements. Linear requested height is calibrated locally; not proof for arbitrary states/terrain.'};
 fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/continuous-charge-verified.json',JSON.stringify(report,null,2));console.log(JSON.stringify(report));
}finally{b.dispose();}
