import assert from 'node:assert/strict';
import fs from 'node:fs';
import {createGameWorld} from '../src/wheelbot_game_world.mjs';
import {createGameController} from '../src/wheelbot_game_control.mjs';
const read=n=>JSON.parse(fs.readFileSync('assets/wheelbot/'+n+'.json'));
const xml=fs.readFileSync('assets/wheelbot/live_model.xml','utf8');
const options={seed:7,stationaryJumpBundle:read('stationary_jump')};

// Vertical box steps are outside the admitted rolling domain of the current
// local LQR/KF controller. Holding D must stop safely before first contact,
// not collide, penetrate, spin up, or trip the physical envelope.
{
 const b=await createGameWorld(xml,'obstacles');
 try{
  const c=createGameController(b,read('live_profile'),read('pose_profiles'),read('target_jump'),options);
  c.setInput({horizontal:1});
  let s;
  for(let k=0;k<2200;k++){s=c.step();assert(!s.failed);}
  assert(s.truth[0]>.45&&s.truth[0]<.70,{x:s.truth[0]});
  assert(Math.abs(s.truth[6])<.03,{vx:s.truth[6]});
  assert(Math.abs(s.target.vx)<.01,{target:s.target});
  assert(s.status.startsWith('Vertical step ahead'),s.status);
  assert((s.last?.physical?.maximumPenetrationM??1)<.001,s.last?.physical);
  c.setInput({horizontal:-1});for(let k=0;k<300;k++)assert(!c.step().failed);
  assert(c.snapshot().truth[0]<s.truth[0]-.05,'Robot must be able to back away from an inadmissible step');
 }finally{b.dispose();}
}

// Smooth known terrain remains an admitted rolling task.
for(const [level,targetX,maxSteps] of [['ramp',1.85,2600],['uneven',1.8,2600]]){
 const b=await createGameWorld(xml,level);
 try{
  const c=createGameController(b,read('live_profile'),read('pose_profiles'),read('target_jump'),options);
  c.setInput({horizontal:1});
  let reached=false,referenceSupportSeen=false,s;
  for(let k=0;k<maxSteps;k++){
   s=c.step();
   if(s.last?.referenceSupport?.source!=='floor')referenceSupportSeen=true;
   assert(!s.failed,{level,k,state:s});
   if(s.truth[0]>=targetX){reached=true;break;}
  }
  assert(referenceSupportSeen,level+' must feed terrain support to the reference');
  assert(reached,level+' must traverse the admitted smooth terrain');
 }finally{b.dispose();}
}
console.log('Vertical-step safe stop plus ramp/uneven traversal PASS');
