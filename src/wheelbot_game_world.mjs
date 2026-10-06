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
// Upper surface of the very same box/sphere geoms, used as known-map context.
// Vertical faces are obstacles, not a licence to teleport onto a higher surface.
export function worldSurface(world,x){
 if(!Number.isFinite(x))throw Error('Nonfinite map coordinate');let height=0,slope=0,source='floor';
 for(const g of world.geoms){
  if(g.type==='sphere'){
   const dx=x-g.pos[0],r=g.size[0];if(Math.abs(dx)>=r)continue;const dz=Math.sqrt(r*r-dx*dx),z=g.pos[2]+dz;
   if(z>height){height=z;slope=-dx/Math.max(dz,1e-8);source=g.name;}
  }else{
   const c=Math.cos(g.pitch),s=Math.sin(g.pitch),hx=g.size[0],hz=g.size[2],vertices=[[-hx,-hz],[hx,-hz],[hx,hz],[-hx,hz]].map(([a,b])=>[g.pos[0]+c*a+s*b,g.pos[2]-s*a+c*b]);
   for(let j=0;j<4;j++){const a=vertices[j],b=vertices[(j+1)%4];if(Math.abs(b[0]-a[0])<1e-10||x<Math.min(a[0],b[0])-1e-10||x>Math.max(a[0],b[0])+1e-10)continue;const f=(x-a[0])/(b[0]-a[0]),z=a[1]+f*(b[1]-a[1]);if(z>height){height=z;slope=(b[1]-a[1])/(b[0]-a[0]);source=g.name;}}
  }
 }
 return {height,slope,source,knownMap:true};
}
export async function createGameWorld(canonicalXML,id){
 const {xml,world}=gameWorldXML(canonicalXML,id),canonical=await createWheelbotContactBackend(canonicalXML);
 if(id==='flat')return {...canonical,robotAssetSha256:canonical.assetSha256,worldManifest:world,surface:x=>worldSurface(world,x)};
 let scene;
 try{
  scene=await createWheelbotContactBackend(xml);const original=canonical.modelInfo(),changed=scene.modelInfo();
  for(const key of ['totalMassKg','bodyMassKg','bodyInertiaKgM2','torqueLimitsNm','jointDamping','jointArmature','jointRanges','rootUnactuated'])if(JSON.stringify(original[key])!==JSON.stringify(changed[key]))throw Error('Terrain changed robot invariant: '+key);
  const pose=[0,.425,0,.74546,-1.44422,0,0,0,0,0,0,0],a=canonical.sceneGeometry(pose),b=scene.sceneGeometry(pose);
  for(const g of a){const next=b.find(x=>x.name===g.name);if(JSON.stringify(g)!==JSON.stringify(next))throw Error('Terrain changed robot/floor geometry: '+g.name);}
  const robotAssetSha256=canonical.assetSha256;canonical.dispose();
  return {...scene,robotAssetSha256,worldManifest:world,surface:x=>worldSurface(world,x)};
 }catch(error){canonical.dispose();scene?.dispose();throw error;}
}
