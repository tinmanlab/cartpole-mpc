// Narrow six-coordinate contact-plant adapter to the existing pinned runtime.
import {MUJOCO_VERSION} from '../vendor/mujoco/version.mjs';
import loadMujoco from '../vendor/mujoco/mujoco.js';
export const STATE_NAMES=['x','z','pitch','hip','knee','wheel','vx','vz','pitch_rate','hip_rate','knee_rate','wheel_rate'];
export const CONTROLLED_INDICES=[0,1,2,3,4,6,7,8,9,10,11];
const vector=(x,n)=>Array.isArray(x)&&x.length===n&&x.every(Number.isFinite);
export async function createWheelbotBackend(xml){
 const mj=await loadMujoco();
 if(mj.mj_versionString()!==MUJOCO_VERSION)throw Error('Unverified MuJoCo version');
 const assetSha256=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',new TextEncoder().encode(xml))),v=>v.toString(16).padStart(2,'0')).join('');
 mj.FS.writeFile('/wheelbot.xml',xml);const m=mj.MjModel.mj_loadXML('/wheelbot.xml');
 function dataBackend(ownsModel=false){
 const d=new mj.MjData(m);
 let disposed=false,steps=0;
 const reject=message=>{d.delete();m.delete();throw Error(message);};
 if(m.nq!==6||m.nv!==6||m.nu!==3||m.na!==0||m.neq!==0||m.ntendon!==0)reject('Unsupported wheelbot model dimensions/support');
 if(Math.abs(m.opt.timestep-.002)>1e-14)reject('Expected 2ms physics timestep');
 const id=(type,name)=>{const n=mj.mj_name2id(m,type,name);if(n<0)reject('Missing wheelbot element '+name);return n;};
 for(const [i,name] of STATE_NAMES.slice(0,6).entries()){
  if(id(3,name)!==i||m.jnt_qposadr[i]!==i||m.jnt_dofadr[i]!==i||m.jnt_type[i]!== (i<2?2:3))reject('Joint mapping mismatch');
  const axis=Array.from(m.jnt_axis.slice(3*i,3*i+3)),want=i===0?[1,0,0]:i===1?[0,0,1]:[0,1,0];
  if(axis.some((v,j)=>Math.abs(v-want[j])>1e-12))reject('Nonplanar joint axis');
 }
 const limits=[];
 for(const [i,name] of ['hip_motor','knee_motor','wheel_motor'].entries()){
  if(id(19,name)!==i||m.actuator_trnid[2*i]!==3+i||m.actuator_gear[6*i]!==1||m.actuator_trntype[i]!==0||m.actuator_dyntype[i]!==0||m.actuator_biastype[i]!==0)reject('Motor transmission mapping mismatch');
  const lo=m.actuator_ctrlrange[2*i],hi=m.actuator_ctrlrange[2*i+1];
  if(!m.actuator_ctrllimited[i]||lo!==-hi||hi<=0)reject('Invalid torque limits');limits.push(hi);
 }
 const torso=id(5,'torso_visual'),wheel=id(5,'wheel_visual');
 const sites=['hip_site','knee_site','wheel_site'].map(n=>id(6,n));
 const state=x=>{if(disposed)throw Error('Disposed backend');if(!vector(x,12))throw Error('Invalid full state');mj.mj_resetData(m,d);d.qpos.set(x.slice(0,6));d.qvel.set(x.slice(6));};
 const forward=x=>{state(x);mj.mj_forward(m,d);};
 const read=()=>{const x=[...d.qpos,...d.qvel];if(!x.every(Number.isFinite))throw Error('Nonfinite physical state');return x;};
 function step(x,u,externalX=0){
  if(!vector(u,3)||!Number.isFinite(externalX))throw Error('Invalid motor torques/external force');state(x);
  d.ctrl.set(u.map((v,i)=>Math.max(-limits[i],Math.min(limits[i],v))));
  // Explicit generalized root-x disturbance: world-horizontal force at the
  // torso/hip origin, NOT its COM. Wheel commands remain joint motor torques.
  d.qfrc_applied[0]=externalX;
  for(let i=0;i<5;i++){mj.mj_step(m,d);steps++;}return read();
 }
 function geometry(x){state(x);mj.mj_kinematics(m,d);return {torso:{position:Array.from(d.geom_xpos.slice(3*torso,3*torso+3)),rotation:Array.from(d.geom_xmat.slice(9*torso,9*torso+9)),halfSize:Array.from(m.geom_size.slice(3*torso,3*torso+3))},hip:Array.from(d.site_xpos.slice(3*sites[0],3*sites[0]+3)),knee:Array.from(d.site_xpos.slice(3*sites[1],3*sites[1]+3)),wheel:Array.from(d.site_xpos.slice(3*sites[2],3*sites[2]+3)),wheelRotation:Array.from(d.geom_xmat.slice(9*wheel,9*wheel+9)),wheelRadius:m.geom_size[3*wheel]};}
 function contact(x){
  const eps=1e-7,xp=x.slice(),xm=x.slice();for(let i=0;i<6;i++){xp[i]+=eps*x[6+i];xm[i]-=eps*x[6+i];}
  const vx=(geometry(xp).wheel[0]-geometry(xm).wheel[0])/(2*eps);state(x);mj.mj_fwdPosition(m,d);
  let wheelContacts=0;for(let i=0;i<d.ncon;i++){const c=d.contact.get(i);if(c.geom1===wheel||c.geom2===wheel)wheelContacts++;}
  return {count:d.ncon,wheelContacts,slip:vx-m.geom_size[3*wheel]*(x[8]+x[9]+x[10]+x[11]),wheelCenterVx:vx};
 }
 function jumpTelemetry(x){
  state(x);mj.mj_fwdPosition(m,d);
  const com=Array.from(d.subtree_com.slice(3,6));
  let wheelContacts=0;for(let i=0;i<d.ncon;i++){const c=d.contact.get(i);if(c.geom1===wheel||c.geom2===wheel)wheelContacts++;}
  const bodyGeoms=[torso,id(5,'upper_link_visual'),id(5,'lower_link_visual')];
  const floorClearance=bodyGeoms.map(g=>{
   const s=m.geom_size.slice(3*g,3*g+3),R=d.geom_xmat.slice(9*g,9*g+9);
   const extent=g===torso?Math.abs(R[6])*s[0]+Math.abs(R[7])*s[1]+Math.abs(R[8])*s[2]:Math.abs(R[8])*s[1]+s[0]*Math.sqrt(Math.max(0,1-R[8]*R[8]));
   return d.geom_xpos[3*g+2]-extent;
  });
  const pairs=[[torso,bodyGeoms[2]],[torso,wheel],[bodyGeoms[1],wheel]];
  const distances=pairs.map(([a,b])=>mj.mj_geomDistance(m,d,a,b,1,null));
  if(!com.every(Number.isFinite)||!distances.every(Number.isFinite))throw Error('Invalid compiled jump geometry');
  return {com,wheelContacts,wheelClearanceM:d.geom_xpos[3*wheel+2]-m.geom_size[3*wheel],minimumBodyFloorClearanceM:Math.min(...floorClearance),minimumNonadjacentDistanceM:Math.min(...distances)};
 }
 return {fork:()=>dataBackend(),nx:12,nu:3,assetSha256,limits,step,geometry,contact,jumpTelemetry,
  diagnostics:()=>({backend:'mujoco-wasm',version:mj.mj_versionString(),assetSha256,nq:6,nv:6,nu:3,physicsDt:.002,controlDt:.01,steps}),
  dispose(){if(!disposed){d.delete();if(ownsModel)m.delete();disposed=true;}}};
 }
 return dataBackend(true);
}
