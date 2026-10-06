import {validateWheelbotProfile,observerUpdate} from './wheelbot_control.mjs';
import {createGameJump} from './wheelbot_game_jump.mjs';
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const approach=(v,t,d)=>v+clamp(t-v,-d,d);
export function createGameController(backend,base,atlas,jumpBundle,{seed=7,world=null}={}){
 const identity={...backend,assetSha256:backend.robotAssetSha256??backend.assetSha256};
 validateWheelbotProfile(identity,base);
 if(!Number.isInteger(seed)||seed<0||seed>0xffffffff||atlas?.schema!=='wheelbot-pose-atlas/v1'||atlas.assetSha256!==identity.assetSha256||atlas.profiles?.length!==25||typeof backend.stepWrench!=='function'||JSON.stringify(backend.limits)!=='[16,16,1.7]')throw Error('Invalid game controller contract');
 atlas.profiles.forEach(p=>validateWheelbotProfile(identity,p));
 const planner=createGameJump(backend,base,jumpBundle),zs=[...new Set(atlas.profiles.map(p=>p.target.z))].sort((a,b)=>a-b),ps=[...new Set(atlas.profiles.map(p=>p.target.pitch))].sort((a,b)=>a-b);
 if(zs.length!==5||ps.length!==5||new Set(atlas.profiles.map(p=>`${p.target.z},${p.target.pitch}`)).size!==25)throw Error('Invalid atlas grid');
 const surface=x=>{const s=backend.surface?.(x)??world?.surface?.(x)??{height:0,slope:0};if(!Number.isFinite(s.height)||!Number.isFinite(s.slope))throw Error('Invalid known surface');return s;};
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
 let input={horizontal:0,vertical:0,tilt:0},steps=0,last=null,failed=false,status='Ready · calibrated charge height',jump=null,index=0,outcome=null,air=0;
 let charge={active:false,seconds:0,fraction:0,requestedHeight:planner.levels[0],acceptedHeight:null};
 const snapshot=()=>structuredClone({truth,estimate,steps,last,failed,observerValid:!failed,phase:failed?'fallen':jump?'jump':charge.active?'charging':'ground',target,charge,jump:jump?{profileId:jump.profileId,vx:jump.vx,index,...outcome}:outcome,status,controlDt:.01});
 function step(){
  if(failed){
   // Loss of control never pauses gravity or contact dynamics.
   truth=backend.stepWrench(truth,[0,0,0],[0,0,0],true);steps++;
   last={u:[0,0,0],contact:backend.contact(truth),physical:backend.lastActionStep(),measurement:null,reference:last?.reference??null};
   return snapshot();
  }
  if(charge.active){charge.seconds=Math.min(1,charge.seconds+.01);charge.fraction=charge.seconds;charge.requestedHeight=planner.levels[0]+charge.fraction*(planner.levels[1]-planner.levels[0]);}
  let action;
  if(jump)action=jump.command(index,estimate);
  else{
   const wheel=backend.geometry([estimate[0],estimate[1],estimate[2],estimate[3],estimate[4],0,...estimate.slice(5)]).wheel,s=surface(wheel[0]);
   const speed=Math.abs(s.slope)>.1?.08:.2;
   target.vx=approach(target.vx,input.horizontal*speed,.005);
   if(Math.abs(target.x+target.vx*.01)>3.95)target.vx=approach(target.vx,0,.01);
   target.x=clamp(target.x+target.vx*.01,-4,4);
   const desired=charge.active?clamp((.3925-height)*3,-.06,.06):input.vertical*.06;
   target.vz=approach(target.vz,desired,.003);height=clamp(height+target.vz*.01,.36,.49);target.pitch=clamp(target.pitch+input.tilt*.004,-.17,.17);target.z=height+s.height;
   const next=profile(height,target.pitch,target.x,s.height),ref=[...next.qref.slice(0,5),0,0,0,0,0,0];
   for(let j=0;j<5;j++)ref[5+j]=(next.qref[j]-pr.qref[j])/.01;
   const wheelX=p=>backend.geometry([...p.qref,0,0,0,0,0,0]).wheel[0];
   ref[10]=(wheelX(next)-wheelX(pr))/.01/.05-ref[7]-ref[8]-ref[9];
   const feed=next.uref.slice();for(let j=0;j<3;j++)feed[j]+=.015*ref[8+j];
   const u=feed.map((v,i)=>clamp(v-next.K[i].reduce((s,g,j)=>s+g*(estimate[j]-ref[j]),0),-backend.limits[i],backend.limits[i]));
   action={u,state:ref};pr=next;
  }
  truth=backend.stepWrench(truth,action.u,[0,0,0],true);const measurement=measure(truth),physical=backend.lastActionStep();
  estimate=jump?jump.observe(index,estimate,action.u,measurement):observerUpdate(pr,estimate,action.u,measurement);
  failed=!estimate.every(Number.isFinite)||Math.abs(truth[0])>4.1||Math.abs(truth[2]-target.pitch)>.6||physical.maximumPenetrationM>.01||physical.jointLimitExcursionRad>.02||physical.maximumAirborneWheelRate>25;
  if(jump){
   outcome.maxClearance=Math.max(outcome.maxClearance,physical.maximumWheelClearanceM-jump.height);
   for(const s of physical.flightSamples){air=s.air?air+.002:0;outcome.flightS=Math.max(outcome.flightS,air);if(s.air)outcome.flight=true;else if(outcome.flight&&s.contacts)outcome.landed=true;}
   index++;target.x=action.state[0];target.z=action.state[1];target.vx=jump.vx;
   if(index>=200&&outcome.landed){const r=jump.reference(index).state;outcome.terminalRelativeRate=Math.max(...truth.slice(6).map((v,j)=>Math.abs(v-r[5+j])));outcome.success=false;outcome.recoverySteps=0;target={x:r[0],z:r[1],pitch:r[2],vx:jump.vx,vz:0};height=r[1]-jump.height;pr=profile(height,target.pitch,target.x,jump.height);jump=null;status='Landed · velocity intent resumes';}
   else if(index>=600){jump=null;failed=true;status='Jump did not land';}
  }
  if(!jump&&outcome?.landed){outcome.recoverySteps++;outcome.terminalRelativeRate=Math.max(...truth.slice(6).map((v,j)=>Math.abs(v-action.state[5+j])));outcome.success=!failed&&outcome.flightS>=.03&&outcome.recoverySteps>=100&&outcome.terminalRelativeRate<=.3;}
  if(failed){status='Physical envelope exceeded · motors off';charge.active=false;input={horizontal:0,vertical:0,tilt:0};jump=null;}
  steps++;last={u:action.u,contact:backend.contact(truth),physical,measurement,reference:action.state};return snapshot();
 }
 return {snapshot,step,setInput(values){if(!values||Object.keys(values).some(k=>!['horizontal','vertical','tilt'].includes(k))||Object.values(values).some(v=>!Number.isFinite(v)||Math.abs(v)>1))throw Error('Inputs must be finite in [-1,1]');input={horizontal:0,vertical:0,tilt:0,...values};},beginCharge(){if(!failed&&!jump&&!charge.active)charge={active:true,seconds:0,fraction:0,requestedHeight:planner.levels[0],acceptedHeight:null};return snapshot();},releaseCharge(){if(!charge.active)return snapshot();charge.active=false;try{jump=planner.start(estimate,charge.fraction,surface(estimate[0]));index=0;air=0;outcome={flight:false,landed:false,flightS:0,maxClearance:0,success:false};charge.acceptedHeight=jump.acceptedHeight;status='Charged moving jump · momentum preserved';}catch(e){status=e.message;}return snapshot();},cancelInput(){input={horizontal:0,vertical:0,tilt:0};charge.active=false;return snapshot();}};
}
