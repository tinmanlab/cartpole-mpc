import {validateWheelbotProfile,observerUpdate} from './wheelbot_control.mjs';
import {createGameJump} from './wheelbot_game_jump.mjs';
import {createStationaryJump} from './wheelbot_stationary_jump.mjs';
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const approach=(v,t,d)=>v+clamp(t-v,-d,d);
export function createGameController(backend,base,atlas,jumpBundle,{seed=7,world=null,stationaryJumpBundle=null}={}){
 const identity={...backend,assetSha256:backend.robotAssetSha256??backend.assetSha256};
 validateWheelbotProfile(identity,base);
 if(!Number.isInteger(seed)||seed<0||seed>0xffffffff||atlas?.schema!=='wheelbot-pose-atlas/v1'||atlas.assetSha256!==identity.assetSha256||atlas.profiles?.length!==25||typeof backend.stepWrench!=='function'||JSON.stringify(backend.limits)!=='[16,16,1.7]')throw Error('Invalid game controller contract');
 atlas.profiles.forEach(p=>validateWheelbotProfile(identity,p));
 let planner=null,stationaryPlanner=null,jumpUnavailableReason='',stationaryJumpUnavailableReason='';
 try{planner=createGameJump(backend,base,jumpBundle);}catch(error){jumpUnavailableReason='Moving jump unavailable · '+(error instanceof Error?error.message:String(error));}
 try{if(stationaryJumpBundle)stationaryPlanner=createStationaryJump(backend,base,stationaryJumpBundle);}catch(error){stationaryJumpUnavailableReason='Stationary jump unavailable · '+(error instanceof Error?error.message:String(error));}
 const zs=[...new Set(atlas.profiles.map(p=>p.target.z))].sort((a,b)=>a-b),ps=[...new Set(atlas.profiles.map(p=>p.target.pitch))].sort((a,b)=>a-b);
 if(zs.length!==5||ps.length!==5||new Set(atlas.profiles.map(p=>`${p.target.z},${p.target.pitch}`)).size!==25)throw Error('Invalid atlas grid');
 const surface=(x,r=.05)=>{const s=backend.surface?.(x,r)??world?.surface?.(x,r)??{height:0,slope:0,source:'floor'};if(!Number.isFinite(s.height)||!Number.isFinite(s.slope))throw Error('Invalid known surface');return s;};
 const verticalStepAhead=(wheelX,direction,r)=>{const manifest=backend.worldManifest??world;if(!direction||manifest?.id!=='obstacles'||!Array.isArray(manifest.geoms))return null;let best=null;for(const g of manifest.geoms){if(g.type!=='box'||Math.abs(g.pitch??0)>1e-8)continue;const edge=direction>0?g.pos[0]-g.size[0]:g.pos[0]+g.size[0],contactCenter=edge-direction*r,distance=direction*(contactCenter-wheelX);if(distance<0||distance>.8)continue;const q={distance,source:g.name,rise:2*g.size[2]};if(!best||q.distance<best.distance)best=q;}return best;};
 function profile(z,pitch,x,height){
  const bracket=(a,v)=>{let i=0;while(i<a.length-2&&v>a[i+1])i++;return [i,clamp((v-a[i])/(a[i+1]-a[i]),0,1)];};
  const [zi,zt]=bracket(zs,z),[pi,pt]=bracket(ps,pitch),corners=[[zi,pi,(1-zt)*(1-pt)],[zi+1,pi,zt*(1-pt)],[zi,pi+1,(1-zt)*pt],[zi+1,pi+1,zt*pt]].map(([i,j,w])=>[atlas.profiles.find(p=>p.target.z===zs[i]&&p.target.pitch===ps[j]),w]);
  const blend=(name)=>{const sum=(v,path)=>Array.isArray(v)?v.map((a,i)=>sum(a,[...path,i])):corners.reduce((s,[p,w])=>s+w*path.reduce((a,i)=>a[i],p[name]),0);return sum(corners[0][0][name],[]);};
  const p={...base};for(const name of ['qref','uref','A','B','K','L'])p[name]=blend(name);p.qref[0]=x;p.qref[1]+=height;return p;
 }
 let rng=seed>>>0;
 const uniform=()=>{rng=(Math.imul(1664525,rng)+1013904223)>>>0;return (rng+.5)/4294967296;};
 const measure=x=>base.measurementIndices.map((j,i)=>x[j]+base.measurementSigma[i]*Math.sqrt(-2*Math.log(uniform()))*Math.cos(2*Math.PI*uniform()));
 let target={x:0,z:.3925+surface(0).height,pitch:0,vx:0,vz:0},height=.3925,pr=profile(height,0,0,surface(0).height),truth=[...pr.qref,0,0,0,0,0,0],estimate=[...measure(truth).slice(0,5),0,0,0,0,0,0];
 let input={horizontal:0,vertical:0,tilt:0,boost:0},speedLimit=.2,steps=0,last=null,failed=false,status=(planner||stationaryPlanner)?'Ready · calibrated charged jump':(jumpUnavailableReason||stationaryJumpUnavailableReason),jump=null,index=0,outcome=null,air=0;
 let charge={active:false,seconds:0,fraction:0,requestedHeight:(stationaryPlanner?.levels[0]??planner?.levels[0]??0),acceptedHeight:null,mode:'stationary'},measurementHistory=[];
 const measuredRates=()=>{if(measurementHistory.length<5)return null;const n=measurementHistory.length,mid=(n-1)/2,den=.01*Array.from({length:n},(_,i)=>(i-mid)**2).reduce((a,b)=>a+b,0);return Array.from({length:5},(_,j)=>measurementHistory.reduce((sum,y,i)=>sum+(i-mid)*y[j],0)/den);};
 const snapshot=()=>structuredClone({truth,estimate,steps,last,failed,observerValid:!failed,phase:failed?'fallen':jump?'jump':charge.active?'charging':'ground',target,speedLimit,charge,jumpAvailable:!!(planner||stationaryPlanner),stationaryJumpAvailable:!!stationaryPlanner,jumpUnavailableReason:jumpUnavailableReason||stationaryJumpUnavailableReason,jump:jump?{profileId:jump.profileId,vx:jump.vx,index,...outcome}:outcome,status,controlDt:.01});
 function step(){
  if(failed){
   // Loss of control never pauses gravity or contact dynamics.
   truth=backend.stepWrench(truth,[0,0,0],[0,0,0],true);steps++;
   last={u:[0,0,0],contact:backend.contact(truth),physical:backend.lastActionStep(),measurement:null,reference:last?.reference??null};
   return snapshot();
  }
  if(charge.active){charge.seconds=Math.min(1,charge.seconds+.01);charge.fraction=charge.seconds;const cp=charge.mode==='stationary'?stationaryPlanner:planner;if(cp)charge.requestedHeight=charge.mode==='stationary'?cp.levels[Math.min(cp.levels.length-1,Math.round(charge.fraction*(cp.levels.length-1)))]:cp.levels[0]+charge.fraction*(cp.levels.at(-1)-cp.levels[0]);}
  let action,referenceSupport=null,terrainObserver=false;
  if(jump)action=jump.command(index,estimate);
  else{
   const geometry=backend.geometry([estimate[0],estimate[1],estimate[2],estimate[3],estimate[4],0,...estimate.slice(5)]),wheel=geometry.wheel,s=surface(wheel[0],geometry.wheelRadius);
   speedLimit=approach(speedLimit,input.boost?.4:.2,input.boost?.0025:.005);
   const direction=Math.sign(input.horizontal||target.vx),preview=direction?Array.from({length:12},(_,i)=>{const distance=(i+1)*geometry.wheelRadius,q=surface(wheel[0]+direction*distance,geometry.wheelRadius);return {distance,q};}):[];
   const firstTerrain=preview.find(v=>v.q.blockedBy||Math.abs(v.q.slope)>.015||Math.abs(v.q.height-s.height)>.0007),verticalStep=verticalStepAhead(wheel[0],direction,geometry.wheelRadius),terrainAhead=!!(firstTerrain||verticalStep);
   let terrainCap=speedLimit;
   if(verticalStep){terrainCap=speedLimit*clamp((verticalStep.distance-.14)/.55,0,1);}
   else if(firstTerrain){const nearCap=Math.abs(firstTerrain.q.slope)>.04?.12:.10,f=clamp((firstTerrain.distance-.06)/.5,0,1);terrainCap=nearCap+(speedLimit-nearCap)*f;}
   const desiredV=input.horizontal*Math.min(speedLimit,terrainCap);
   target.vx=approach(target.vx,desiredV,terrainAhead?.002:.005);
   if(terrainAhead){const sensedX=measurementHistory.at(-1)?.[0]??estimate[0];target.x=approach(target.x,sensedX,.001);}
   if(verticalStep&&verticalStep.distance<.2&&Math.abs(target.vx)<.02&&input.horizontal!==0)status='Vertical step ahead · rolling transition not admitted';
   else if(status.startsWith('Vertical step ahead'))status='Ready · calibrated charged jump';
   if(Math.abs(target.x+target.vx*.01)>3.95)target.vx=approach(target.vx,0,.01);
   target.x=clamp(target.x+target.vx*.01,-4,4);
   const desired=charge.active?clamp((.3925-height)*3,-.06,.06):input.vertical*.06;
   target.vz=approach(target.vz,desired,.003);height=clamp(height+target.vz*.01,.36,.49);target.pitch=clamp(target.pitch+input.tilt*.004,-.17,.17);let next=profile(height,target.pitch,target.x,s.height);const rg0=backend.geometry([...next.qref,0,0,0,0,0,0]);referenceSupport=surface(rg0.wheel[0],rg0.wheelRadius);terrainObserver=s.source!=='floor'||referenceSupport.source!=='floor'||!!(firstTerrain&&firstTerrain.distance<=geometry.wheelRadius*1.5);next=profile(height,target.pitch,target.x,referenceSupport.height);target.z=height+referenceSupport.height;
   const ref=[...next.qref.slice(0,5),0,0,0,0,0,0];
   for(let j=0;j<5;j++)ref[5+j]=(next.qref[j]-pr.qref[j])/.01;ref[5]=target.vx;
   const wheelX=p=>backend.geometry([...p.qref,0,0,0,0,0,0]).wheel[0];
   ref[10]=(wheelX(next)-wheelX(pr))/.01/.05-ref[7]-ref[8]-ref[9];
   const feed=next.uref.slice();for(let j=0;j<3;j++)feed[j]+=.015*ref[8+j];
   const u=feed.map((v,i)=>clamp(v-next.K[i].reduce((s,g,j)=>s+g*(estimate[j]-ref[j]),0),-backend.limits[i],backend.limits[i]));
   if((terrainAhead||terrainObserver)&&!verticalStep)u[2]=clamp(u[2]-.04*(estimate[10]-ref[10]),-backend.limits[2],backend.limits[2]);
   action={u,state:ref};pr=next;
  }
  truth=backend.stepWrench(truth,action.u,[0,0,0],true);const measurement=measure(truth),physical=backend.lastActionStep();
  measurementHistory.push(measurement.slice());if(measurementHistory.length>7)measurementHistory.shift();
  estimate=jump?jump.observe(index,estimate,action.u,measurement):observerUpdate(pr,estimate,action.u,measurement);
  if(!jump&&terrainObserver){const rates=measuredRates();if(rates){for(let j=0;j<5;j++){estimate[j]=.8*estimate[j]+.2*measurement[j];estimate[5+j]=.6*estimate[5+j]+.4*rates[j];}estimate[10]=.6*estimate[10]+.4*measurement[5];}}
  failed=!estimate.every(Number.isFinite)||Math.abs(truth[0])>4.1||Math.abs(truth[2]-target.pitch)>.6||physical.maximumPenetrationM>.01||physical.jointLimitExcursionRad>.02||physical.maximumAirborneWheelRate>25;
  if(jump){
   outcome.maxClearance=Math.max(outcome.maxClearance,physical.maximumWheelClearanceM-jump.height);
   for(const s of physical.flightSamples){air=s.air?air+.002:0;outcome.flightS=Math.max(outcome.flightS,air);if(s.air)outcome.flight=true;else if(outcome.flight&&s.contacts)outcome.landed=true;}
   index++;target.x=action.state[0];target.z=action.state[1];target.vx=jump.vx;
   if(index>=200&&outcome.landed){const r=jump.reference(index).state;outcome.terminalRelativeRate=Math.max(...truth.slice(6).map((v,j)=>Math.abs(v-r[5+j])));outcome.success=false;outcome.recoverySteps=0;target={x:r[0],z:r[1],pitch:r[2],vx:jump.vx,vz:0};height=r[1]-jump.height;pr=profile(height,target.pitch,target.x,jump.height);jump=null;status='Landed · velocity intent resumes';}
   else if(index>=600){jump=null;failed=true;status='Jump did not land';}
  }
  if(!jump&&outcome?.landed){outcome.recoverySteps++;outcome.terminalRelativeRate=Math.max(...truth.slice(6).map((v,j)=>Math.abs(v-action.state[5+j])));outcome.success=!failed&&outcome.flightS>=.03&&outcome.recoverySteps>=100&&outcome.terminalRelativeRate<=.3;}
  if(failed){status='Physical envelope exceeded · motors off';charge.active=false;input={horizontal:0,vertical:0,tilt:0,boost:0};jump=null;}
  steps++;last={u:action.u,contact:backend.contact(truth),physical,measurement,reference:action.state,support:jump?null:surface(backend.geometry(truth).wheel[0],backend.geometry(truth).wheelRadius),referenceSupport:jump?null:referenceSupport};return snapshot();
 }
 return {snapshot,step,setInput(values){if(!values||Object.keys(values).some(k=>!['horizontal','vertical','tilt','boost'].includes(k))||Object.values(values).some(v=>!Number.isFinite(v)||Math.abs(v)>1))throw Error('Inputs must be finite in [-1,1]');input={horizontal:0,vertical:0,tilt:0,boost:0,...values};},beginCharge(){if(failed||jump||charge.active)return snapshot();const stationary=Math.abs(target.vx)<.03&&input.horizontal===0,cp=stationary?stationaryPlanner:planner;if(!cp){status=stationary?(stationaryJumpUnavailableReason||'Stationary jump unavailable'):jumpUnavailableReason;return snapshot();}charge={active:true,seconds:0,fraction:0,requestedHeight:cp.levels[0],acceptedHeight:null,mode:stationary?'stationary':'moving'};return snapshot();},releaseCharge(){if(!charge.active)return snapshot();charge.active=false;const cp=charge.mode==='stationary'?stationaryPlanner:planner;if(!cp){status=charge.mode==='stationary'?stationaryJumpUnavailableReason:jumpUnavailableReason;return snapshot();}try{const geometry=backend.geometry(truth);jump=cp.start(estimate,charge.fraction,surface(geometry.wheel[0],geometry.wheelRadius));index=0;air=0;outcome={flight:false,landed:false,flightS:0,maxClearance:0,success:false};charge.acceptedHeight=jump.acceptedHeight;status=jump.stationary?'Stationary charged jump · launch x retained':'Charged moving jump · momentum preserved';}catch(e){status=e instanceof Error?e.message:String(e);}return snapshot();},cancelInput(){input={horizontal:0,vertical:0,tilt:0,boost:0};charge.active=false;return snapshot();}};
}
