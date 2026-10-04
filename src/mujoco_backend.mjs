// Official MuJoCo 3.7.0 WASM. This adapter maps the lab's four-state interface;
// it contains no equations of motion, rigid-body solver or fallback integrator.
import loadMujoco from '../vendor/mujoco/mujoco.js';
const DT=.02,wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));
const arr=(a,start,n)=>Array.from(a.slice(start,start+n));
export async function createMujocoBackend(){
  const url=new URL('../assets/cartpole.xml',import.meta.url);
  const xml=typeof window==='undefined'?await (await import('node:fs/promises')).readFile(url,'utf8'):await (async()=>{const r=await fetch(url);if(!r.ok)throw Error('MJCF HTTP '+r.status);return r.text();})();
  const digest=await crypto.subtle.digest('SHA-256',new TextEncoder().encode(xml));
  const assetSha256=Array.from(new Uint8Array(digest),v=>v.toString(16).padStart(2,'0')).join('');
  const mj=await loadMujoco();
  const version=mj.mj_versionString();if(version!=='3.7.0')throw Error('Unexpected MuJoCo runtime: '+version);
  mj.FS.writeFile('/cartpole.xml',xml);
  const cache=new Map();let stepCalls=0;
  function entry(spec,params={},substeps=4){
    if(spec.actuator && spec.actuator!=='ideal')throw Error('This MJCF supports an ideal force actuator only; no silent BAM equivalence');
    const p={mc:params.mc??spec.mc,mp:params.mp??spec.mp,l:params.l??spec.l,g:spec.gravity,friction:params.friction??0};
    if(![p.mc,p.mp,p.l,p.g].every(v=>Number.isFinite(v)&&v>0)||!Number.isFinite(p.friction)||p.friction<0||!Number.isInteger(substeps)||substeps<1)throw Error('Invalid MuJoCo parameters');
    const key=JSON.stringify([p,substeps]);if(cache.has(key))return cache.get(key);
    const model=mj.MjModel.mj_loadXML('/cartpole.xml'),data=new mj.MjData(model);
    const id=(type,name)=>{const i=mj.mj_name2id(model,type,name);if(i<0)throw Error('Missing MJCF element '+name);return i;};
    const cart=id(1,'cart'),pole=id(1,'pole'),cartGeom=id(5,'cart_visual'),poleGeom=id(5,'pole_visual'),rail=id(5,'rail'),pivot=id(6,'pivot'),tip=id(6,'pole_tip');
    if(model.nq!==2||model.nv!==2||model.nu!==1)throw Error('Unexpected MJCF dimensions');
    // Change explicit physical parameters and the corresponding visual geometry
    // together; recompute MuJoCo's derived constants after model changes.
    model.body_mass[cart]=p.mc;model.body_mass[pole]=p.mp;
    model.body_ipos[pole*3+2]=p.l;
    model.body_inertia[pole*3]=p.mp*p.l*p.l/3;model.body_inertia[pole*3+1]=p.mp*p.l*p.l/3;
    model.geom_pos[poleGeom*3+2]=p.l;model.geom_size[poleGeom*3+1]=p.l;
    model.site_pos[tip*3+2]=2*p.l;
    model.dof_damping[0]=p.friction;model.opt.gravity[2]=-p.g;model.opt.timestep=DT/substeps;
    mj.mj_setConst(model,data);
    const e={model,data,p,cart,pole,cartGeom,poleGeom,rail,pivot,tip};
    if(cache.size>=32){const oldest=cache.keys().next().value,old=cache.get(oldest);old.data.delete();old.model.delete();cache.delete(oldest);}
    cache.set(key,e);return e;
  }
  function setState(e,state){
    if(state.length!==4||!state.every(Number.isFinite))throw Error('Non-finite or invalid simulation state');
    mj.mj_resetData(e.model,e.data);e.data.qpos[0]=state[0];e.data.qpos[1]=state[2];e.data.qvel[0]=state[1];e.data.qvel[1]=state[3];
  }
  function advance(state,u,external,spec,params=null,substeps=4){
    if(!Number.isFinite(u)||!Number.isFinite(external))throw Error('Non-finite force rejected before MuJoCo');
    const e=entry(spec,params??{},substeps);setState(e,state);
    e.data.ctrl[0]=u;e.data.qfrc_applied[0]=external;
    for(let k=0;k<substeps;k++){mj.mj_step(e.model,e.data);stepCalls++;}
    const out=[e.data.qpos[0],e.data.qvel[0],wrap(e.data.qpos[1]),e.data.qvel[1]];
    if(!out.every(Number.isFinite))throw Error('MuJoCo returned non-finite state');return out;
  }
  function geometry(state,spec,params=null){
    const e=entry(spec,params??{});setState(e,state);mj.mj_forward(e.model,e.data);
    return {cart:arr(e.data.geom_xpos,3*e.cartGeom,3),cartHalfSize:arr(e.model.geom_size,3*e.cartGeom,3),
      pivot:arr(e.data.site_xpos,3*e.pivot,3),tip:arr(e.data.site_xpos,3*e.tip,3),rodRadius:e.model.geom_size[3*e.poleGeom],
      rail:arr(e.data.geom_xpos,3*e.rail,3),railHalfSize:arr(e.model.geom_size,3*e.rail,3),
      mass:[e.model.body_mass[e.cart],e.model.body_mass[e.pole]],poleCom:arr(e.model.body_ipos,3*e.pole,3),poleInertia:arr(e.model.body_inertia,3*e.pole,3)};
  }
  function acceleration(state,u,spec,params=null){const e=entry(spec,params??{});setState(e,state);e.data.ctrl[0]=u;mj.mj_forward(e.model,e.data);return arr(e.data.qacc,0,2);}
  return {name:'mujoco-wasm',version,asset:'assets/cartpole.xml',assetSha256,
    transition:(state,u,spec,params,n=4)=>advance(state,u,0,spec,params,n),advance,geometry,acceleration,
    diagnostics:()=>({backend:'mujoco-wasm',version,asset:'assets/cartpole.xml',assetSha256,stepCalls,cachedModels:cache.size,integrator:'Euler',physicsDt:.005,controlDt:DT}),
    dispose:()=>{for(const e of cache.values()){e.data.delete();e.model.delete();}cache.clear();}};
}
