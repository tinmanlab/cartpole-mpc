import fs from 'node:fs';import assert from 'node:assert/strict';
import{createGameWorld}from'../src/wheelbot_game_world.mjs';import{createGameController}from'../src/wheelbot_game_control.mjs';
const read=p=>JSON.parse(fs.readFileSync('assets/wheelbot/'+p+'.json'));
const b=await createGameWorld(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'),'obstacles');
try{const c=createGameController(b,read('live_profile'),read('pose_profiles'),read('target_jump'));c.setInput({horizontal:.4});let s;
 for(let k=0;k<2000;k++){s=c.step();if(s.failed)break;}assert(s.failed,'The fixed obstacle scenario must exercise the failure transition');
 const steps=s.steps,physicalSteps=b.diagnostics().steps,prior=s.truth;
 c.cancelInput();c.beginCharge();c.releaseCharge();s=c.step();assert.equal(s.steps,steps+1,'Falling must continue physics, not freeze on a control failure');assert.equal(b.diagnostics().steps,physicalSteps+5);assert(s.failed);assert.deepEqual(s.last.u,[0,0,0]);assert.equal(s.observerValid,false);assert.notDeepEqual(s.truth,prior);
 console.log('Fallen control disengages motors and observation while actual gravity/contact dynamics continue PASS');
}finally{b.dispose();}
