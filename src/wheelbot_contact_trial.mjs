// Full-contact physical trials deliberately do not stop at the old local-LQR
// pitch/height envelope. Falling is an observed state, not a hidden reset.
export function createContactTrial(backend,metadata,{scenario='standing',torques=[0,0,0],steps=500}={}){
 if(!['standing','fall-left','fall-right','obstacle','torque'].includes(scenario))throw Error('Unsupported contact scenario');
 if(!Array.isArray(torques)||torques.length!==3||!torques.every(Number.isFinite)||!Number.isInteger(steps)||steps<1||steps>2000)throw Error('Invalid contact trial request');
 const c=metadata.configuration;let truth=metadata.initialState.slice();
 if(scenario.startsWith('fall')){truth[0]=-.4;truth[1]=Math.max(c.upper.lengthM+c.lower.lengthM+c.wheel.radiusM+.12,c.base.sizeM[2]+.25);truth[2]=scenario==='fall-left'?-1.1:1.1;}
 if(scenario==='obstacle'){if(!c.obstacle.enabled)throw Error('Obstacle is disabled');truth[0]=c.obstacle.centerM[0]-.22;truth[1]=c.obstacle.centerM[2]+c.obstacle.sizeM[2]/2+.06;truth[2]=1.25;truth[6]=.2;}
 let count=0,last=null,stopped=false;const history=[];
 const snapshot=()=>({truth:truth.slice(),truth12:truth.slice(),estimate:null,steps:count,failed:stopped,done:stopped||count>=steps,last,mode:'contact-diagnostic',goal:0,scenario});
 return {snapshot,history,step(externalX=0){
  if(externalX!==0)throw Error('No hidden external force in contact/actuator diagnostic');
  if(snapshot().done)throw Error('Contact trial completed; explicit reset required');
  const requested=scenario==='torque'?torques:[0,0,0];
  const u=requested.map((v,i)=>Math.max(-backend.limits[i],Math.min(backend.limits[i],v)));
  truth=backend.step(truth,u,0);count++;
  const contacts=backend.contactDetails(truth,u);
  // These are bounded numeric escape guards, not a successful-get-up definition.
  stopped=!truth.every(Number.isFinite)||Math.abs(truth[0])>10||Math.abs(truth[1])>5;
  last={u,requested:requested.slice(),saturated:requested.some((v,i)=>v!==u[i]),externalX:0,contact:backend.contact(truth),allContacts:contacts};
  const state=snapshot();history.push(state);return state;
 }};
}
