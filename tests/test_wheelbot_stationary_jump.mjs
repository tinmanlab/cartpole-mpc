import assert from 'node:assert/strict';
import fs from 'node:fs';
import {createWheelbotContactBackend} from '../src/wheelbot_contact_backend.mjs';
import {createGameController} from '../src/wheelbot_game_control.mjs';
const read=n=>JSON.parse(fs.readFileSync('assets/wheelbot/'+n+'.json'));
const xml=fs.readFileSync('assets/wheelbot/live_model.xml','utf8');
const fractions=[0,.5,1],rows=[];
for(const fraction of fractions){
 const b=await createWheelbotContactBackend(xml);
 try{
  const c=createGameController(b,read('live_profile'),read('pose_profiles'),read('target_jump'),{seed:7,stationaryJumpBundle:read('stationary_jump')});
  for(let k=0;k<150;k++)assert(!c.step().failed);
  const launch=c.snapshot().truth[0];
  c.beginCharge();for(let k=0;k<Math.round(fraction*100);k++)assert(!c.step().failed);
  const released=c.releaseCharge();assert.equal(released.phase,'jump');assert(released.stationaryJumpAvailable);
  let maxDx=0,maxAir=0,maxClearance=0,done=false;
  for(let k=0;k<700;k++){
   const s=c.step();
   maxDx=Math.max(maxDx,Math.abs(s.truth[0]-launch));
   maxAir=Math.max(maxAir,s.last?.physical?.maximumAirborneWheelRate??0);
   maxClearance=Math.max(maxClearance,s.last?.physical?.maximumWheelClearanceM??0);
   assert(!s.failed,{fraction,k,s,maxDx,maxAir});
   if(s.phase==='ground'&&s.jump?.landed&&s.jump?.success){done=true;break;}
  }
  const final=c.snapshot();
  assert(done,{fraction,final});
  assert(maxDx<.08,{fraction,maxDx});
  assert(Math.abs(final.truth[0]-launch)<.02,{fraction,launch,finalX:final.truth[0]});
  assert(maxAir<25,{fraction,maxAir});
  assert(maxClearance>.02,{fraction,maxClearance});
  rows.push({fraction,acceptedHeight:final.charge.acceptedHeight,maxDx,finalDx:final.truth[0]-launch,maxAir,maxClearance});
 }finally{b.dispose();}
}
assert(rows[0].acceptedHeight<rows[1].acceptedHeight&&rows[1].acceptedHeight<rows[2].acceptedHeight,rows);
console.log(JSON.stringify({passed:true,rows}));
{
 const b=await createWheelbotContactBackend(xml);
 try{
  const bad={...read('stationary_jump'),nativeVersion:'mismatch'};
  const c=createGameController(b,read('live_profile'),read('pose_profiles'),read('target_jump'),{seed:7,stationaryJumpBundle:bad});
  assert.equal(c.snapshot().stationaryJumpAvailable,false);
  c.beginCharge();c.step();const stopped=c.releaseCharge();assert.notEqual(stopped.phase,'jump');assert(stopped.jumpUnavailableReason.length>0);
  c.setInput({horizontal:1});for(let k=0;k<200;k++)assert(!c.step().failed);
  c.beginCharge();for(let k=0;k<10;k++)c.step();const moving=c.releaseCharge();assert.equal(moving.phase,'jump','Moving jump remains available when only stationary data is invalid');
 }finally{b.dispose();}
}
