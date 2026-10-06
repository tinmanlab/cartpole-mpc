// Diagnostic view on the SAME XML and official runtime, while the existing
// backend integrates ordinary modes. Fast trials step only this diagnostic MjData;
// the core MjData is used for read-only geometry, never a second integrator.
import loadMujoco from '../vendor/mujoco/mujoco.js';
import {createWheelbotBackend} from './wheelbot_backend.mjs';
export async function createWheelbotContactBackend(xml){
 const core=await createWheelbotBackend(xml);
 let mj,m,d;
 try{mj=await loadMujoco();mj.FS.writeFile('/contact-diagnostics.xml',xml);m=mj.MjModel.mj_loadXML('/contact-diagnostics.xml');}catch(e){core.dispose();if(d)d.delete();if(m)m.delete();throw e;}
 function dataBackend(core,ownsModel=false){
 const d=new mj.MjData(m);
 const forceBuffer=new mj.DoubleBuffer(6);
 const limits=core.limits,assetSha256=core.assetSha256;
 const vector=(a,n)=>Array.isArray(a)&&a.length===n&&a.every(Number.isFinite);
 const state=x=>{if(!vector(x,12))throw Error('Invalid contact diagnostic state');mj.mj_resetData(m,d);d.qpos.set(x.slice(0,6));d.qvel.set(x.slice(6));};
 const forward=x=>{state(x);mj.mj_forward(m,d);};
 function contactDetails(x,u=[0,0,0]){
  if(!vector(u,3))throw Error('Invalid contact diagnostic torque');
  state(x);d.ctrl.set(u.map((v,i)=>Math.max(-limits[i],Math.min(limits[i],v))));mj.mj_forward(m,d);
  const pairs=[];
  for(let i=0;i<d.ncon;i++){
   const c=d.contact.get(i);mj.mj_contactForce(m,d,i,forceBuffer);const local=Array.from(forceBuffer.GetView());
   const R=Array.from(c.frame);const world=[0,1,2].map(j=>R[j]*local[0]+R[3+j]*local[1]+R[6+j]*local[2]);
   pairs.push({geom1:mj.mj_id2name(m,5,c.geom1),geom2:mj.mj_id2name(m,5,c.geom2),distanceM:c.dist,normalForceN:local[0],forceOnGeom2WorldN:world,positionM:Array.from(c.pos),active:c.efc_address>=0});
  }
  return {count:d.ncon,pairs,maximumPenetrationM:Math.max(0,...pairs.map(p=>-p.distanceM)),source:'MuJoCo contact solver at current state and explicit diagnostic torque; not a sensor measurement'};
 }
 function sceneGeometry(x){
  state(x);mj.mj_kinematics(m,d);const geoms=[];
  for(let i=0;i<m.ngeom;i++)geoms.push({name:mj.mj_id2name(m,5,i),type:m.geom_type[i],position:Array.from(d.geom_xpos.slice(3*i,3*i+3)),rotation:Array.from(d.geom_xmat.slice(9*i,9*i+9)),size:Array.from(m.geom_size.slice(3*i,3*i+3)),rgba:Array.from(m.geom_rgba.slice(4*i,4*i+4)),collisionEnabled:!!(m.geom_contype[i]||m.geom_conaffinity[i])});
  return geoms;
 }
 function modelInfo(){
  return {assetSha256,totalMassKg:Array.from(m.body_mass).reduce((a,b)=>a+b,0),bodyMassKg:Array.from(m.body_mass),bodyInertiaKgM2:Array.from(m.body_inertia),torqueLimitsNm:limits.slice(),jointDamping:Array.from(m.dof_damping),jointArmature:Array.from(m.dof_armature),jointRanges:Array.from(m.jnt_range),rootUnactuated:true};
 }
 let disposed=false,fastSteps=0,lastActionStep=null;
 function physicsStep(x,u){
  if(disposed||!vector(u,3)||u.some((v,i)=>Math.abs(v)>limits[i]))throw Error('Invalid fast motor command');
  state(x);d.ctrl.set(u);mj.mj_step(m,d);fastSteps++;
  const truth=[...d.qpos,...d.qvel];
  if(!truth.every(Number.isFinite))throw Error('Nonfinite fast state');
  const penetration=Math.max(0,...Array.from({length:d.ncon},(_,i)=>-d.contact.get(i).dist));
  return {truth,penetration,externalForcesZero:[...d.qfrc_applied,...d.xfrc_applied].every(v=>v===0)};
 }
 // Explicit world force at the torso origin and pure world-y moment. For
 // these planar root coordinates virtual work gives [Fx,Fz,Ty,0,0,0].
 function stepWrench(x,u,wrench=[0,0,0],flight=false){
  if(disposed||!vector(u,3)||u.some((v,i)=>Math.abs(v)>limits[i])||!vector(wrench,3)||wrench.some((v,i)=>Math.abs(v)>[4,4,.1][i]))throw Error('Invalid action torque/wrench');
  state(x);d.ctrl.set(u);d.qfrc_applied.set([...wrench,0,0,0]);
  let maximumPenetrationM=0,jointLimitExcursionRad=0,minimumNonadjacentDistanceM=1,wheelContactLost=false,maximumWheelClearanceM=0,maximumAbsPitch=0,maximumAirborneWheelRate=0,minimumHeight=Infinity;const flightSamples=[];
  for(let i=0;i<5;i++){mj.mj_step(m,d);fastSteps++;
   mujocoForwardForChecks();
   maximumPenetrationM=Math.max(maximumPenetrationM,...Array.from({length:d.ncon},(_,j)=>-d.contact.get(j).dist));
   jointLimitExcursionRad=Math.max(jointLimitExcursionRad,m.jnt_range[6]-d.qpos[3],d.qpos[3]-m.jnt_range[7],m.jnt_range[8]-d.qpos[4],d.qpos[4]-m.jnt_range[9]);
  }
  function mujocoForwardForChecks(){
   const geometry=core.jumpTelemetry([...d.qpos,...d.qvel]);
   if(flight){const c=core.contact([...d.qpos,...d.qvel]);flightSamples.push({air:c.wheelContacts===0&&geometry.wheelClearanceM>.005,contacts:c.wheelContacts,slip:c.slip});maximumAbsPitch=Math.max(maximumAbsPitch,Math.abs(d.qpos[2]));minimumHeight=Math.min(minimumHeight,d.qpos[1]);if(geometry.wheelContacts===0&&geometry.wheelClearanceM>.005)maximumAirborneWheelRate=Math.max(maximumAirborneWheelRate,Math.abs(d.qvel[2]+d.qvel[3]+d.qvel[4]+d.qvel[5]));}
   maximumWheelClearanceM=Math.max(maximumWheelClearanceM,geometry.wheelClearanceM);
   wheelContactLost ||= geometry.wheelContacts===0;
   minimumNonadjacentDistanceM=Math.min(minimumNonadjacentDistanceM,geometry.minimumNonadjacentDistanceM);
  }
  lastActionStep={flightSamples,maximumAbsPitch,maximumAirborneWheelRate,minimumHeight,maximumPenetrationM,jointLimitExcursionRad,minimumNonadjacentDistanceM,wheelContactLost,maximumWheelClearanceM,externalWrench:wrench.slice(),actualTorqueNm:Array.from(d.actuator_force)};
  const truth=[...d.qpos,...d.qvel];if(!truth.every(Number.isFinite))throw Error('Nonfinite action state');
  return truth;
 }
 function actionTelemetry(x,u=[0,0,0]){
  const contacts=contactDetails(x,u);
  const wheel=mj.mj_name2id(m,5,'wheel_visual');
  return {...contacts,wheelClearanceM:d.geom_xpos[3*wheel+2]-m.geom_size[3*wheel],jointLimitExcursionRad:Math.max(0,m.jnt_range[6]-x[3],x[3]-m.jnt_range[7],m.jnt_range[8]-x[4],x[4]-m.jnt_range[9]),actualTorqueNm:Array.from(d.actuator_force),rootUnactuated:true};
 }
 function trackingGeometry(x,sign){
  forward(x);const body=mj.mj_name2id(m,1,'torso'),g=mj.mj_name2id(m,5,'torso_visual');
  const edge=d.xpos[3*body+2]+d.xmat[9*body+6]*sign*m.geom_size[3*g]+d.xmat[9*body+8]*(m.geom_pos[3*g+2]-m.geom_size[3*g+2]);
  const excursion=Math.max(0,m.jnt_range[6]-x[3],x[3]-m.jnt_range[7],m.jnt_range[8]-x[4],x[4]-m.jnt_range[9]);
  return {edge,excursion,penetration:Math.max(0,...Array.from({length:d.ncon},(_,i)=>-d.contact.get(i).dist))};
 }

 return {...core,fork:()=>dataBackend(core.fork()),stepWrench,lastActionStep:()=>lastActionStep?structuredClone(lastActionStep):null,actionTelemetry,physicsStep,trackingGeometry,diagnostics:()=>({...core.diagnostics(),steps:core.diagnostics().steps+fastSteps,fastPhysicsSteps:fastSteps}),contactDetails,sceneGeometry,modelInfo,dispose(){if(!disposed){core.dispose();forceBuffer.delete();d.delete();if(ownsModel)m.delete();disposed=true;}}};
 }
 return dataBackend(core,true);
}
