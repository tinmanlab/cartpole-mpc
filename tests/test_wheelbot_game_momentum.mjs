// Actual no-stop launch and airborne COM movement, distinguished from landing recoil.
import fs from 'node:fs';import assert from 'node:assert/strict';
import{createGameWorld}from'../src/wheelbot_game_world.mjs';import{createGameController}from'../src/wheelbot_game_control.mjs';
const read=n=>JSON.parse(fs.readFileSync('assets/wheelbot/'+n+'.json'));
const b=await createGameWorld(fs.readFileSync('assets/wheelbot/live_model.xml','utf8'),'flat'),rows=[];
try{for(const direction of [-1,1])for(const duration of [10,100]){
 const c=createGameController(b,read('live_profile'),read('pose_profiles'),read('target_jump'));
 for(let k=0;k<200;k++)c.step();c.setInput({horizontal:direction});for(let k=0;k<200;k++)c.step();c.beginCharge();for(let k=0;k<duration;k++)c.step();
 const before=c.snapshot(),released=c.releaseCharge();assert.equal(released.phase,'jump');assert.deepEqual(released.truth,before.truth);assert.deepEqual(released.estimate,before.estimate);
 let prior=b.jumpTelemetry(before.truth).com[0],wasAir=false,seenAir=false,preMin=Infinity,flightMin=Infinity,flightSamples=0,minBase=Infinity,phaseStart=null,s;
 for(let k=0;k<400;k++){
  s=c.step();assert(!s.failed);const com=b.jumpTelemetry(s.truth).com[0],vx=(com-prior)/.01,air=s.last.physical.flightSamples.every(z=>z.air);
  if(!seenAir)preMin=Math.min(preMin,direction*vx);
  if(air&&wasAir){flightMin=Math.min(flightMin,direction*vx);flightSamples++;}
  if(air){seenAir=true;if(phaseStart===null)phaseStart=(k+1)*.01;}
  assert.deepEqual(s.last.physical.externalWrench,[0,0,0]);assert(s.last.u.every((v,j)=>Math.abs(v)<=b.limits[j]));
  minBase=Math.min(minBase,direction*s.truth[6]);prior=com;wasAir=air;
 }
 assert(preMin>.1,'No stop before physical takeoff');assert(flightSamples>=5&&flightMin>.1,'Moving flight must retain forward total-COM progression');
 assert(s.jump.success&&direction*s.truth[6]>.15&&direction*(s.truth[0]-before.truth[0])>.6);
 rows.push({direction,heldSeconds:duration*.01,minimumLaunchCOMSpeed:preMin,minimumFlightCOMSpeed:flightMin,flightSamples,takeoffSeconds:phaseStart,minimumSignedBaseSpeedIncludingLanding:minBase,finalVx:s.truth[6],translation:s.truth[0]-before.truth[0],passed:true});
}const result={passed:true,rows,scope:'No state reset, positive launch/flight momentum and resumed speed. Brief base recoil during landing is measured, not hidden or interpreted as a stopped jump.'};fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/game-momentum.json',JSON.stringify(result,null,2));console.log(JSON.stringify(result));}finally{b.dispose();}
