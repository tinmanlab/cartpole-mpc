import fs from 'node:fs';import assert from 'node:assert/strict';
import{createGameWorld,gameWorldDefinition,gameWorldXML,worldSurface,GAME_LEVELS}from'../src/wheelbot_game_world.mjs';
const xml=fs.readFileSync('assets/wheelbot/live_model.xml','utf8'),base=JSON.parse(fs.readFileSync('assets/wheelbot/live_profile.json'));
const rows=[];
for(const {id}of GAME_LEVELS){
 const b=await createGameWorld(xml,id);
 try{
  assert.equal(b.robotAssetSha256,base.assetSha256);assert.equal(b.diagnostics().version,'3.15.0');
  assert.deepEqual(b.limits,[16,16,1.7]);assert.equal(worldSurface(b.worldManifest,0).height,0);
  const geoms=b.sceneGeometry([...base.qref,0,0,0,0,0,0]);
  const terrain=geoms.filter(g=>g.name.startsWith('terrain_'));assert.equal(terrain.length,b.worldManifest.geoms.length);assert(terrain.every(g=>g.collisionEnabled));
  assert.equal(geoms.find(g=>g.name==='torso_visual').type,6);
  const contacts=[];
  for(const g of b.worldManifest.geoms.slice(0,4)){
   const x=[...base.qref,0,0,0,0,0,0],wheel0=b.geometry(x).wheel;const targetX=g.pos[0],map=worldSurface(b.worldManifest,targetX);
   x[0]+=targetX-wheel0[0];x[1]+=map.height-.0005;
   const detail=b.contactDetails(x,[0,0,0]);
   assert(detail.pairs.some(c=>[c.geom1,c.geom2].includes('terrain_'+g.name)),id+': actual collision missing for '+g.name);
   assert(detail.pairs.every(c=>Number.isFinite(c.normalForceN)&&c.normalForceN>=-1e-9));contacts.push({name:g.name,height:map.height,actualPairs:detail.pairs.map(c=>[c.geom1,c.geom2])});
  }
  rows.push({id,actualSceneSha256:b.assetSha256,robotSha256:b.robotAssetSha256,terrainGeoms:terrain.length,contactChecks:contacts});
 }finally{b.dispose();}
}
assert.throws(()=>gameWorldDefinition('unknown'));assert.throws(()=>gameWorldXML('', 'flat'));assert.throws(()=>worldSurface(gameWorldDefinition('flat'),NaN));
const result={passed:true,levels:rows,scope:'Canonical robot invariants, actual obstacle/ramp/uneven collisions and known-map agreement. Course traversal is a separate closed-loop test; diagnostic contact placement is not a controller teleport.'};
fs.mkdirSync('test-results',{recursive:true});fs.writeFileSync('test-results/game-world.json',JSON.stringify(result,null,2));console.log(JSON.stringify(result));
