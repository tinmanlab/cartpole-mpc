import assert from 'node:assert/strict';
import fs from 'node:fs';
import {worldWheelSupport,gameWorldDefinition,createGameWorld} from '../src/wheelbot_game_world.mjs';

const r=.05;
const flat=worldWheelSupport(gameWorldDefinition('flat'),0,r);
assert.equal(flat.height,0);assert.equal(flat.slope,0);

const ramp=gameWorldDefinition('ramp'),x=1.1,m=.052;
const expected=(x-.7)*m+r*(Math.sqrt(1+m*m)-1);
assert(Math.abs(worldWheelSupport(ramp,x,r).height-expected)<1e-10);

const obstacles=gameWorldDefinition('obstacles'),lowEdge=.85-.18/2;
const lowNear=worldWheelSupport(obstacles,lowEdge-.02,r);
assert(lowNear.height>0&&lowNear.height<.012,'Wheel must meet the 1.2 cm corner before its centre crosses the top face');
assert(Math.abs(worldWheelSupport(obstacles,.85,r).height-.012)<1e-12);

const tallEdge=3.4-.28/2,tallNear=worldWheelSupport(obstacles,tallEdge-.02,r);
assert.equal(tallNear.height,0,'An 8 cm vertical step must not be promoted to traversable support');
assert.equal(tallNear.blockedBy?.source,'tall_block');

assert.throws(()=>worldWheelSupport(ramp,NaN,r));
assert.throws(()=>worldWheelSupport(ramp,0,-1));

const xml=fs.readFileSync('assets/wheelbot/live_model.xml','utf8');
const base=JSON.parse(fs.readFileSync('assets/wheelbot/live_profile.json'));
async function contactAt(level,wheelX,expectedTerrain,{expectSupport=true}={}){
 const b=await createGameWorld(xml,level);
 try{
  const state=[...base.qref,0,0,0,0,0,0],g0=b.geometry(state),support=b.surface(wheelX,g0.wheelRadius);
  state[0]+=wheelX-g0.wheel[0];
  state[1]+=support.height-.0002;
  const details=b.contactDetails(state,[0,0,0]);
  const terrain='terrain_'+expectedTerrain;
  assert(details.pairs.some(p=>p.geom1===terrain||p.geom2===terrain),`${level}: expected actual MuJoCo contact with ${terrain}`);
  if(expectSupport)assert.equal(support.source,expectedTerrain,`${level}: support source must match the contacting geom`);
  return {level,wheelX,support,maximumPenetrationM:details.maximumPenetrationM,pairs:details.pairs.map(p=>[p.geom1,p.geom2])};
 }finally{b.dispose();}
}

const evidence=[];
evidence.push(await contactAt('ramp',1.1,'up_ramp'));
evidence.push(await contactAt('uneven',.90,worldWheelSupport(gameWorldDefinition('uneven'),.90,r).source));

const mediumEdge=1.65-.22/2;
evidence.push(await contactAt('obstacles',lowEdge-.02,'low_step'));
evidence.push(await contactAt('obstacles',mediumEdge-.02,'medium_step'));
evidence.push(await contactAt('obstacles',tallEdge-.02,'tall_block',{expectSupport:false}));

assert(evidence.slice(0,4).every(e=>e.support.finiteRadius&&e.support.knownMap));
assert(evidence.at(-1).support.blockedBy?.source==='tall_block');

fs.mkdirSync('test-results',{recursive:true});
fs.writeFileSync('test-results/wheelbot-wheel-support.json',JSON.stringify({passed:true,evidence},null,2));
console.log('Finite-radius support matches actual MuJoCo contact; tall step remains blocked PASS');
