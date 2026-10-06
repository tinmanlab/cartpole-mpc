// Known, static test courses. All robot parameters and dynamic coordinates stay
// identical to the canonical box; only explicitly declared world geoms are added.
import {createWheelbotContactBackend} from './wheelbot_contact_backend.mjs';
const box=(name,x,width,height)=>({name,type:'box',pos:[x,0,height/2],size:[width/2,.35,height/2],pitch:0});
function ramp(name,x1,z1,x2,z2){const angle=Math.atan2(z2-z1,x2-x1),half=.018;return {name,type:'box',pos:[(x1+x2)/2+half*Math.sin(angle),0,(z1+z2)/2-half*Math.cos(angle)],size:[Math.hypot(x2-x1,z2-z1)/2,.35,half],pitch:-angle};}
const courses={
 flat:{label:'평지',geoms:[]},
 obstacles:{label:'장애물',geoms:[box('low_step',.85,.18,.012),box('medium_step',1.65,.22,.03),{name:'round_bump',type:'sphere',pos:[2.5,0,-.025],size:[.055],pitch:0},box('tall_block',3.4,.28,.08)]},
 ramp:{label:'경사면',geoms:[ramp('up_ramp',.7,0,1.7,.052),box('plateau',2.05,.7,.052),ramp('down_ramp',2.4,.052,3.4,0)]},
 uneven:{label:'요철',geoms:Array.from({length:16},(_,i)=>{const x=.7+i*.16,z=j=>.012*(1-Math.cos(j*Math.PI/4))/2;return ramp('ridge_'+i,x,z(i),x+.16,z(i+1));})},
};
const clone=v=>structuredClone(v);
const freeze=v=>{if(v&&typeof v==='object'){Object.values(v).forEach(freeze);Object.freeze(v);}return v;};
export const GAME_LEVELS=freeze(Object.entries(courses).map(([id,v])=>({id,label:v.label})));
const encode=v=>v.map(x=>Number(x).toPrecision(15)).join(' ');
export function gameWorldDefinition(id){if(!Object.hasOwn(courses,id))throw Error('Unknown test course');return freeze({id,label:courses[id].label,xLimits:[-3.8,4.6],robotSpawnX:0,geoms:clone(courses[id].geoms)});}
export function gameWorldXML(canonicalXML,id){
 if(typeof canonicalXML!=='string'||canonicalXML.split('</worldbody>').length!==2||!canonicalXML.includes('name="torso"'))throw Error('Invalid canonical robot XML');
 const world=gameWorldDefinition(id);
 const geoms=world.geoms.map(g=>`    <geom name="terrain_${g.name}" type="${g.type}" pos="${encode(g.pos)}" size="${encode(g.size)}" euler="0 ${g.pitch} 0" contype="1" conaffinity="1" friction="1 0.01 0.001" rgba="0.32 0.42 0.48 1" />`).join('\n');
 return {xml:geoms?canonicalXML.replace('</worldbody>',geoms+'\n  </worldbody>'):canonicalXML,world};
}
const boxVertices=g=>{const c=Math.cos(g.pitch),s=Math.sin(g.pitch),hx=g.size[0],hz=g.size[2];return [[-hx,-hz],[hx,-hz],[hx,hz],[-hx,hz]].map(([a,b])=>[g.pos[0]+c*a+s*b,g.pos[2]-s*a+c*b]);};
function pointSurface(world,x,exclude=null){
 if(!Number.isFinite(x))throw Error('Nonfinite map coordinate');
 let height=0,slope=0,source='floor';
 for(const g of world.geoms){
  if(g.name===exclude)continue;
  if(g.type==='sphere'){
   const dx=x-g.pos[0],r=g.size[0];if(Math.abs(dx)>=r)continue;const dz=Math.sqrt(r*r-dx*dx),z=g.pos[2]+dz;
   if(z>height){height=z;slope=-dx/Math.max(dz,1e-8);source=g.name;}
  }else{
   const vertices=boxVertices(g);
   for(let j=0;j<4;j++){const a=vertices[j],b=vertices[(j+1)%4];if(Math.abs(b[0]-a[0])<1e-10||x<Math.min(a[0],b[0])-1e-10||x>Math.max(a[0],b[0])+1e-10)continue;const f=(x-a[0])/(b[0]-a[0]),z=a[1]+f*(b[1]-a[1]);if(z>height){height=z;slope=(b[1]-a[1])/(b[0]-a[0]);source=g.name;}}
  }
 }
 return {height,slope,source,knownMap:true};
}
// Upper surface of the very same box/sphere geoms, used as point-sampled
// known-map context. Vertical faces are obstacles, not support surfaces.
export function worldSurface(world,x){return pointSurface(world,x);}

// Configuration-space support for a finite-radius wheel. The returned height is
// an equivalent terrain lift: wheel-center z = wheelRadius + height. It is
// derived from the exact declared geoms, not from a second terrain model.
//
// Up-facing box faces are offset by the wheel radius. Their upper vertices get
// circular transition arcs so a wheel sees a low step before its centre crosses
// the vertical face. A corner whose rise from the adjacent support exceeds one
// wheel radius is deliberately NOT promoted into a traversable support arc;
// that leaves the vertical obstacle for MuJoCo to reject physically.
export function worldWheelSupport(world,x,wheelRadius){
 if(!world||!Array.isArray(world.geoms)||!Number.isFinite(x)||!Number.isFinite(wheelRadius)||wheelRadius<=0)throw Error('Invalid finite-radius support query');
 let best={height:0,slope:0,source:'floor',feature:'floor',knownMap:true,finiteRadius:true,drivable:true,blockedBy:null};
 let blocked=null;
 const choose=c=>{if(c.height>best.height+1e-12)best={...c,knownMap:true,finiteRadius:true,drivable:true,blockedBy:null};};
 for(const g of world.geoms){
  if(g.type==='sphere'){
   const radius=g.size[0]+wheelRadius,dx=x-g.pos[0];if(Math.abs(dx)>radius)continue;
   const dz=Math.sqrt(Math.max(0,radius*radius-dx*dx));
   choose({height:g.pos[2]+dz-wheelRadius,slope:-dx/Math.max(dz,1e-10),source:g.name,feature:'sphere'});
   continue;
  }
  const vertices=boxVertices(g),upper=new Set();
  for(let j=0;j<4;j++){
   const a=vertices[j],b=vertices[(j+1)%4],ex=b[0]-a[0],ez=b[1]-a[1],length=Math.hypot(ex,ez);
   if(length<1e-12)continue;
   const nx=ez/length,nz=-ex/length,surfaceSlope=Math.abs(ex)>1e-12?ez/ex:Infinity;
   // Only upward, reasonably traversable faces become support. Near-vertical
   // side faces remain collision obstacles.
   if(nz<=1e-10||Math.abs(surfaceSlope)>1)continue;
   upper.add(j);upper.add((j+1)%4);
   const aa=[a[0]+wheelRadius*nx,a[1]+wheelRadius*nz],bb=[b[0]+wheelRadius*nx,b[1]+wheelRadius*nz];
   if(x<Math.min(aa[0],bb[0])-1e-12||x>Math.max(aa[0],bb[0])+1e-12||Math.abs(bb[0]-aa[0])<1e-12)continue;
   const f=(x-aa[0])/(bb[0]-aa[0]);
   choose({height:aa[1]+f*(bb[1]-aa[1])-wheelRadius,slope:(bb[1]-aa[1])/(bb[0]-aa[0]),source:g.name,feature:'face'});
  }
  for(const j of upper){
   const v=vertices[j],dx=x-v[0];if(Math.abs(dx)>wheelRadius)continue;
   const dz=Math.sqrt(Math.max(0,wheelRadius*wheelRadius-dx*dx));
   const height=v[1]+dz-wheelRadius;
   // Query the support immediately on the approach side without this geom.
   // This detects a true vertical step while allowing connected ramp/plateau
   // endpoints and low steps to generate their physical wheel-corner arc.
   const direction=dx===0?0:Math.sign(dx),probe=v[0]+direction*1e-7;
   const adjacent=pointSurface(world,probe,g.name).height,rise=v[1]-adjacent;
   if(rise>wheelRadius+1e-9){
    if(!blocked||height>blocked.height)blocked={source:g.name,feature:'vertical-step',rise,height};
    continue;
   }
   choose({height,slope:-dx/Math.max(dz,1e-10),source:g.name,feature:'corner'});
  }
 }
 if(blocked&&blocked.height>best.height+1e-12)best.blockedBy={source:blocked.source,feature:blocked.feature,rise:blocked.rise};
 return best;
}
export async function createGameWorld(canonicalXML,id){
 const {xml,world}=gameWorldXML(canonicalXML,id),canonical=await createWheelbotContactBackend(canonicalXML);
 if(id==='flat')return {...canonical,robotAssetSha256:canonical.assetSha256,worldManifest:world,surface:(x,r=.05)=>worldWheelSupport(world,x,r)};
 let scene;
 try{
  scene=await createWheelbotContactBackend(xml);const original=canonical.modelInfo(),changed=scene.modelInfo();
  for(const key of ['totalMassKg','bodyMassKg','bodyInertiaKgM2','torqueLimitsNm','jointDamping','jointArmature','jointRanges','rootUnactuated'])if(JSON.stringify(original[key])!==JSON.stringify(changed[key]))throw Error('Terrain changed robot invariant: '+key);
  const pose=[0,.425,0,.74546,-1.44422,0,0,0,0,0,0,0],a=canonical.sceneGeometry(pose),b=scene.sceneGeometry(pose);
  for(const g of a){const next=b.find(x=>x.name===g.name);if(JSON.stringify(g)!==JSON.stringify(next))throw Error('Terrain changed robot/floor geometry: '+g.name);}
  const robotAssetSha256=canonical.assetSha256;canonical.dispose();
  return {...scene,robotAssetSha256,worldManifest:world,surface:(x,r=.05)=>worldWheelSupport(world,x,r)};
 }catch(error){canonical.dispose();scene?.dispose();throw error;}
}
