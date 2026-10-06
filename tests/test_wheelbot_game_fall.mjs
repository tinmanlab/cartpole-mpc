import fs from 'node:fs';import assert from 'node:assert/strict';
import{createGameWorld}from'../src/wheelbot_game_world.mjs';import{createGameController}from'../src/wheelbot_game_control.mjs';
const read=p=>JSON.parse(fs.readFileSync('assets/wheelbot/'+p+'.json'));
const b=await createGameWorld(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'),'flat');
let disturb=true;
const forced={...b,stepWrench:(state,u,w,flight)=>b.stepWrench(state,u,disturb?[4,0,0]:[0,0,0],flight)};
try{const c=createGameController(forced,read('live_profile'),read('pose_profiles'),read('target_jump'));let s;
 for(let k=0;k<300;k++){s=c.step();if(s.failed)break;}assert(s.failed,'Bounded 4 N physical pulse must exercise the failure transition');
 disturb=false;
 const steps=s.steps,physicalSteps=b.diagnostics().steps,prior=s.truth;
 c.cancelInput();c.beginCharge();c.releaseCharge();s=c.step();assert.equal(s.steps,steps+1,'Falling must continue physics, not freeze on a control failure');assert.equal(b.diagnostics().steps,physicalSteps+5);assert(s.failed);assert.deepEqual(s.last.u,[0,0,0]);assert.equal(s.observerValid,false);assert.notDeepEqual(s.truth,prior);
 console.log('Fallen control disengages motors and observation while actual gravity/contact dynamics continue PASS');
}finally{b.dispose();}
