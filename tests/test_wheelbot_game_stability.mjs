import assert from 'node:assert/strict';
import fs from 'node:fs';
import {createGameWorld} from '../src/wheelbot_game_world.mjs';
import {createGameController} from '../src/wheelbot_game_control.mjs';
const read=n=>JSON.parse(fs.readFileSync('assets/wheelbot/'+n+'.json'));
const xml=fs.readFileSync('assets/wheelbot/live_model.xml','utf8');

for(const [level,targetX,maxSteps] of [['obstacles',1.05,1800],['ramp',1.85,2600]]){
 const b=await createGameWorld(xml,level);
 try{
  const c=createGameController(b,read('live_profile'),read('pose_profiles'),read('target_jump'),{seed:7});
  c.setInput({horizontal:1});
  let reached=false,referenceSupportSeen=false;
  for(let k=0;k<maxSteps;k++){
   const s=c.step();
   if(s.last?.referenceSupport?.source!=='floor')referenceSupportSeen=true;
   if(s.failed)break;
   if(s.truth[0]>=targetX){reached=true;break;}
  }
  assert(referenceSupportSeen,level+' must feed terrain support to the reference');
  assert(reached&&!c.snapshot().failed,level+' should traverse the first validated terrain transition without falling');
 }finally{b.dispose();}
}
console.log('Reference-wheel terrain support clears first obstacle and ramp transition PASS');
