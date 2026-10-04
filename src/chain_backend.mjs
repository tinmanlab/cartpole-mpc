// Arbitrary-dimension serial-chain adapter to the SAME pinned official MuJoCo WASM.
// No dynamics equations, optimizer, additional engine, motor torque or hidden actuator.
import loadMujoco from '../vendor/mujoco/mujoco.js';
export async function createChainBackend(xml,poles){
 if(!Number.isInteger(poles)||poles<1||poles>32)throw Error('poles outside resource bound 1..32');
 const mj=await loadMujoco();if(mj.mj_versionString()!=='3.7.0')throw Error('Unverified MuJoCo version');
 const digest=await crypto.subtle.digest('SHA-256',new TextEncoder().encode(xml));
 const assetSha256=Array.from(new Uint8Array(digest),v=>v.toString(16).padStart(2,'0')).join('');
 mj.FS.writeFile('/chain.xml',xml);const m=mj.MjModel.mj_loadXML('/chain.xml'),d=new mj.MjData(m);
 const dof=poles+1,nx=2*dof;let disposed=false,steps=0;
 if(m.nq!==dof||m.nv!==dof||m.nu!==1||m.na!==0){d.delete();m.delete();throw Error('Unsupported chain dimension or actuator state');}
 if(Math.abs(m.opt.timestep-.005)>1e-14){d.delete();m.delete();throw Error('Expected 5ms asset timestep');}
 const wrap=v=>Math.atan2(Math.sin(v),Math.cos(v));
 const id=(type,name)=>{const i=mj.mj_name2id(m,type,name);if(i<0)throw Error('Missing canonical chain element '+name);return i;};
 const cart=id(5,'cart_visual'),sites=Array.from({length:poles},(_,i)=>[id(6,'pivot'+(i?'_'+(i+1):'')),id(6,'pole_tip'+(i?'_'+(i+1):''))]);
 function state(x){if(disposed)throw Error('Disposed chain backend');if(!Array.isArray(x)||x.length!==nx||!x.every(Number.isFinite))throw Error('Invalid state dimension or values');mj.mj_resetData(m,d);d.qpos.set(x.slice(0,dof));d.qvel.set(x.slice(dof));}
 function read(){const out=[...d.qpos,...d.qvel];for(let i=1;i<dof;i++)out[i]=wrap(out[i]);if(!out.every(Number.isFinite))throw Error('Nonfinite MuJoCo result');return out;}
 function step(x,u,external=0){if(!Number.isFinite(u)||!Number.isFinite(external))throw Error('Invalid force');state(x);d.ctrl[0]=u;d.qfrc_applied[0]=external;for(let k=0;k<4;k++){mj.mj_step(m,d);steps++;}return read();}
 function geometry(x){state(x);mj.mj_forward(m,d);return {cart:Array.from(d.geom_xpos.slice(3*cart,3*cart+3)),cartHalfSize:Array.from(m.geom_size.slice(3*cart,3*cart+3)),links:sites.map(([a,b])=>({pivot:Array.from(d.site_xpos.slice(3*a,3*a+3)),tip:Array.from(d.site_xpos.slice(3*b,3*b+3))}))};}
 function linearize(x=Array(nx).fill(0),u=0,eps=1e-5){
  const A=Array.from({length:nx},()=>Array(nx).fill(0)),B=Array.from({length:nx},()=>[0]);
  for(let j=0;j<nx;j++){const xp=x.slice(),xm=x.slice();xp[j]+=eps;xm[j]-=eps;const plus=step(xp,u),minus=step(xm,u);for(let i=0;i<nx;i++)A[i][j]=(i>0&&i<dof?wrap(plus[i]-minus[i]):plus[i]-minus[i])/(2*eps);}
  const plus=step(x,u+eps),minus=step(x,u-eps);for(let i=0;i<nx;i++)B[i][0]=(i>0&&i<dof?wrap(plus[i]-minus[i]):plus[i]-minus[i])/(2*eps);
  return {A,B};
 }
 return {poles,dof,nx,nu:1,assetSha256,step,geometry,linearize,
  diagnostics:()=>({backend:'mujoco-wasm',version:'3.7.0',assetSha256,nq:m.nq,nv:m.nv,nu:m.nu,physicsDt:.005,controlDt:.02,steps}),
  dispose(){if(!disposed){d.delete();m.delete();disposed=true;}}};
}
