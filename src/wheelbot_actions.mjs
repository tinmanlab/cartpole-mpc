import {validateWheelbotProfile,observerUpdate} from './wheelbot_control.mjs';
import {CONTROLLED_INDICES as indices} from './wheelbot_backend.mjs';
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const keys=['x','z','pitch'];
// Same final position/rate bounds as ordinary posture admission; flight must be physical.
export function jumpCompleted(record){
 if(record===null||typeof record!=='object'||Array.isArray(record))return false;
 const {flight,landed,maxClearance,truth,reference}=record;
 // Array.from deliberately makes missing sparse entries visible to validation.
 const finiteVector=(value,length)=>Array.isArray(value)&&value.length===length&&Array.from(value).every(Number.isFinite);
 if(flight!==true||landed!==true||!Number.isFinite(maxClearance)||maxClearance<.02||!finiteVector(truth,12)||!finiteVector(reference,11))return false;
 return truth.slice(0,3).every((value,j)=>Math.abs(value-reference[j])<=[.02,.005,.03][j])&&truth.slice(6).every(value=>Math.abs(value)<=.3);
}
export function simplifyPath(points,tolerance=.003){
 if(points.length<3)return {points:points.slice(),maxDeviationM:0};
 const distance=(p,a,b)=>{const dx=b.x-a.x,dz=b.z-a.z,t=clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/(dx*dx+dz*dz||1),0,1);return Math.hypot(p.x-a.x-t*dx,p.z-a.z-t*dz);};
 const keep=new Set([0,points.length-1]);let maxDeviationM=0;
 function split(a,b){let max=0,index=a;for(let i=a+1;i<b;i++){const d=distance(points[i],points[a],points[b]);if(d>max){max=d;index=i;}}if(max>tolerance){keep.add(index);split(a,index);split(index,b);}else maxDeviationM=Math.max(maxDeviationM,max);}
 split(0,points.length-1);return {points:[...keep].sort((a,b)=>a-b).map(i=>points[i]),maxDeviationM};
}
export function createWheelbotActions(backend,baseProfile,poseBundle,{seed=17,initial=null,jumpPlanner=null}={}){
 if(!Number.isInteger(seed)||seed<0||seed>0xffffffff)throw Error('Seed must be an unsigned 32-bit integer');
 validateWheelbotProfile(backend,baseProfile);
 const bundle=structuredClone(poseBundle),base=structuredClone(baseProfile);
 if(bundle?.schema!=='wheelbot-pose-atlas/v1'||bundle.assetSha256!==backend.assetSha256||!Array.isArray(bundle.profiles)||bundle.profiles.length!==25||typeof backend.stepWrench!=='function')throw Error('Invalid posture atlas/backend');
 for(const k of keys)if(!Array.isArray(bundle.ranges?.[k])||bundle.ranges[k].length!==2||!bundle.ranges[k].every(Number.isFinite)||bundle.ranges[k][0]>=bundle.ranges[k][1])throw Error('Invalid atlas ranges');
 for(const name of ['velocity','acceleration'])if(!Array.isArray(bundle.referenceLimits?.[name])||bundle.referenceLimits[name].length!==3||!bundle.referenceLimits[name].every(v=>Number.isFinite(v)&&v>0))throw Error('Invalid reference limits');
 bundle.profiles.forEach(p=>validateWheelbotProfile(backend,p));
 const zs=[...new Set(bundle.profiles.map(p=>p.target.z))].sort((a,b)=>a-b),ps=[...new Set(bundle.profiles.map(p=>p.target.pitch))].sort((a,b)=>a-b);
 if(zs.length!==5||ps.length!==5||!zs.every(Number.isFinite)||!ps.every(Number.isFinite)||new Set(bundle.profiles.map(p=>[p.target.z,p.target.pitch].join(','))).size!==25)throw Error('Invalid atlas grid');
 const bracket=(a,v)=>{let i=0;while(i<a.length-2&&v>a[i+1])i++;return [i,clamp((v-a[i])/(a[i+1]-a[i]),0,1)];};
 let cachedProfile=null,cachedWheel=null;
 function wheelX(p){
  const q=p.qref;if(cachedWheel&&q.slice(1,5).every((v,i)=>v===cachedWheel.q[i]))return q[0]+cachedWheel.offset;
  const offset=backend.geometry([0,...q.slice(1),0,0,0,0,0,0]).wheel[0];cachedWheel={q:q.slice(1,5),offset};return q[0]+offset;
 }
 function profile(z,pitch,x){
  if(cachedProfile?.z===z&&cachedProfile.pitch===pitch)return {...cachedProfile.p,qref:[x,...cachedProfile.p.qref.slice(1)]};
  const [zi,zt]=bracket(zs,z),[pi,pt]=bracket(ps,pitch);
  const corners=[[zi,pi,(1-zt)*(1-pt)],[zi+1,pi,zt*(1-pt)],[zi,pi+1,(1-zt)*pt],[zi+1,pi+1,zt*pt]].map(([i,j,w])=>[bundle.profiles.find(p=>p.target.z===zs[i]&&p.target.pitch===ps[j]),w]);
  const blend=(name)=>{const go=(v,path)=>Array.isArray(v)?v.map((a,i)=>go(a,[...path,i])):corners.reduce((s,[p,w])=>s+w*path.reduce((a,i)=>a[i],p[name]),0);return go(corners[0][0][name],[]);};
  const p={...base};for(const name of ['qref','uref','A','B','K','L'])p[name]=blend(name);p.qref[0]=x;cachedProfile={z,pitch,p};return p;
 }
 if(initial){
  const valid=(a,n)=>Array.isArray(a)&&a.length===n&&a.every(Number.isFinite);
  if(!valid(initial.truth,12)||!valid(initial.estimate,11)||(initial.referenceVelocity&&!valid(initial.referenceVelocity,3))||!initial.appliedTarget||!keys.every(k=>Number.isFinite(initial.appliedTarget[k])&&initial.appliedTarget[k]>=bundle.ranges[k][0]&&initial.appliedTarget[k]<=bundle.ranges[k][1])||Math.abs(initial.truth[0])>2||initial.truth[1]>.7||initial.truth.slice(6).some(v=>Math.abs(v)>20)||Math.abs(initial.truth[2])>.5||initial.truth[1]<.26||Math.abs(initial.truth[3])>1.28||Math.abs(initial.truth[4])>2.53||initial.queue?.some(p=>!keys.every(k=>Number.isFinite(p[k])&&p[k]>=bundle.ranges[k][0]&&p[k]<=bundle.ranges[k][1])))throw Error('Invalid or fallen initial state/reference');
 }
 let rng=seed>>>0;
 const uniform=()=>{rng=(Math.imul(1664525,rng)+1013904223)>>>0;return (rng+.5)/4294967296;};
 const noise=()=>Math.sqrt(-2*Math.log(uniform()))*Math.cos(2*Math.PI*uniform());
 const measure=x=>base.measurementIndices.map((j,i)=>x[j]+base.measurementSigma[i]*noise());
 let truth=initial?.truth.slice()??[...base.qref,0,0,0,0,0,0];
 let estimate=initial?.estimate.slice()??[...measure(truth).slice(0,5),0,0,0,0,0,0];
 let appliedTarget=initial?.appliedTarget??{x:0,z:base.qref[1],pitch:base.qref[2]},target={...(initial?.target??appliedTarget)};
 let referenceVelocity=initial?.referenceVelocity??[0,0,0];
 let steps=0,last=null,failed=false,queue=initial?.queue??[],cursor=0,serial=0,status='Ready',path=null,pulse=null,jump=null,jumpIndex=0,jumpWasAir=false,jumpLanded=false,jumpClearance=0,jumpOutcome=null;
 const snapshot=()=>structuredClone({actions:true,truth,estimate,steps,last,failed,done:false,target,appliedTarget,goal:target.x,phase:failed?'fallen':jump?'jump':cursor<queue.length?'moving':'standing',actionStatus:status,jumpOutcome,path,disturbance:pulse,capabilities:{jump:!!jumpPlanner,recover:false},controlDt:.01});
 function schedule(points,scale){
  const nodes=[{...appliedTarget},...points.filter((p,i)=>i||keys.some(k=>Math.abs(p[k]-appliedTarget[k])>1e-8))],frames=[];
  const spans=nodes.slice(1).map((p,i)=>Math.max(.15,Math.abs(p.x-nodes[i].x)/.24,Math.abs(p.z-nodes[i].z)/.06,Math.abs(p.pitch-nodes[i].pitch)/.24)*scale);
  const rates=nodes.map((p,i)=>i===0?referenceVelocity.slice():i===nodes.length-1?[0,0,0]:keys.map(k=>{const a=(p[k]-nodes[i-1][k])/spans[i-1],b=(nodes[i+1][k]-p[k])/spans[i];return a*b>0?Math.sign(a)*Math.min(Math.abs(a),Math.abs(b)):0;}));
  for(let i=0;i<spans.length;i++){
   const from=nodes[i],to=nodes[i+1],span=spans[i],n=Math.ceil(span/.01);
   for(let k=1;k<=n;k++){const t=k/n,s=10*t**3-15*t**4+6*t**5;frames.push(Object.fromEntries(keys.map((key,j)=>[key,from[key]+(to[key]-from[key])*s+span*rates[i][j]*(t-6*t**3+8*t**4-3*t**5)+span*rates[i+1][j]*(-4*t**3+7*t**4-3*t**5)])));}
  }
  return frames;
 }
 function step(){
  if(jump&&!failed){
   const started=performance.now(),action=jump.command(jumpIndex,estimate),controlMs=performance.now()-started,plantStart=performance.now();
   truth=backend.stepWrench(truth,action.u,[0,0,0],true);const plantMs=performance.now()-plantStart,measurement=measure(truth);
   estimate=jump.observe(jumpIndex,estimate,action.u,measurement);const physical=backend.lastActionStep();
   jumpClearance=Math.max(jumpClearance,physical.maximumWheelClearanceM);
   for(const sample of physical.flightSamples){if(sample.air)jumpWasAir=true;else if(jumpWasAir&&!jumpLanded&&sample.contacts){jumpLanded=true;failed ||= !Number.isFinite(sample.slip)||Math.abs(sample.slip)>jump.landingSpeedBound;}}
   failed ||= physical.maximumPenetrationM>.01||physical.jointLimitExcursionRad>.02||physical.maximumAbsPitch>.6||physical.maximumAirborneWheelRate>25||physical.minimumHeight<.15||physical.minimumNonadjacentDistanceM<0;
   last={u:action.u,requested:action.u,externalWrench:[0,0,0],measurement,controlMs,plantMs,physical,reference:action.state};steps++;jumpIndex++;
   if(failed){jumpOutcome={success:false,flight:jumpWasAir,landed:jumpLanded,maxClearance:jumpClearance};jump=null;status='점프 물리 한계 초과 · 모터 해제';}
   else if(jumpIndex===600){const r=jump.reference(599).state;const success=jumpCompleted({flight:jumpWasAir,landed:jumpLanded,maxClearance:jumpClearance,truth,reference:r});jumpOutcome={success,flight:jumpWasAir,landed:jumpLanded,maxClearance:jumpClearance};appliedTarget={x:r[0],z:r[1],pitch:r[2]};target={...appliedTarget};referenceVelocity=[0,0,0];jump=null;status=success?'점프 완료 · 도착 자세 균형 유지':'점프 목표 미달 · 균형 제어 계속';}
   return snapshot();
  }
  const started=performance.now(),old=profile(appliedTarget.z,appliedTarget.pitch,appliedTarget.x);
  if(cursor<queue.length)appliedTarget={...queue[cursor++]};
  const p=profile(appliedTarget.z,appliedTarget.pitch,appliedTarget.x);
  const ref=indices.map(j=>j<6?p.qref[j]:0);
  referenceVelocity=keys.map((k,j)=>(p.qref[j]-old.qref[j])/.01);
  // All scheduled coordinate rates, including the joints induced by atlas FK.
  for(let j=0;j<5;j++)ref[5+j]=(p.qref[j]-old.qref[j])/.01;
  const oldWheel=wheelX(old);
  const newWheel=wheelX(p);
  ref[10]=(newWheel-oldWheel)/.01/.05-ref[7]-ref[8]-ref[9];
  const requested=failed?[0,0,0]:p.uref.map((v,i)=>v-p.K[i].reduce((sum,g,j)=>sum+g*(estimate[j]-ref[j]),0));
  const u=requested.map((v,i)=>clamp(v,-backend.limits[i],backend.limits[i])),controlMs=performance.now()-started,plantStart=performance.now();
  const wrench=[0,0,0];
  if(pulse){const envelope=pulse.kind==='gust'?Math.sin(Math.PI*(pulse.total-pulse.remaining+.5)/pulse.total):1;wrench[pulse.kind==='twist'?2:0]=pulse.direction*pulse.amplitude*envelope;}
  truth=backend.stepWrench(truth,u,wrench);const plantMs=performance.now()-plantStart;
  const observerStart=performance.now(),measurement=measure(truth);if(!failed)estimate=observerUpdate(p,estimate,u,measurement);
  const physical=backend.lastActionStep();
  failed ||= Math.abs(truth[2]-appliedTarget.pitch)>.5||truth[1]<.26||physical.maximumPenetrationM>.005||physical.jointLimitExcursionRad>.02||physical.minimumNonadjacentDistanceM<0;
  if(failed){status='Outside ground control domain; motors disengaged';queue=[];}
  if(pulse&&--pulse.remaining===0)pulse=null;
  last={u,requested,externalWrench:wrench,measurement,controlMs:controlMs+performance.now()-observerStart,plantMs,loopMs:performance.now()-started,physical,reference:ref};steps++;return snapshot();
 }
 function brakeFuture(){
  const origin={...appliedTarget},v=referenceVelocity.slice();queue=[];cursor=0;
  if(v.some(n=>Math.abs(n)>1e-8))for(let k=1;k<=20;k++){const t=k/20;queue.push(Object.fromEntries(keys.map((key,j)=>[key,clamp(origin[key]+v[j]*.2*(t-t*t+t*t*t/3),...bundle.ranges[key])])));}
  target={...(queue.at(-1)??appliedTarget)};
 }
 function requestPath(requested,options){return planPath(requested,options);}
 async function planPath(requested,{mode='base',action='follow',pitch=target.pitch,jumpUnavailableReason=null,yieldTask=()=>new Promise(resolve=>setTimeout(resolve,0))}={},token=++serial,retry=0,started=performance.now()){
  if(!['base','wheel'].includes(mode)||!['follow','jump'].includes(action)||!Number.isFinite(pitch)||!Array.isArray(requested)||!requested.length||requested.length>64||requested.some(p=>!p||!Number.isFinite(p.x)||!Number.isFinite(p.z)))throw Error('Use 1–64 finite 2D points');
  const original=structuredClone(requested),previous=path;
  if(jump){status='Unavailable: 현재 점프의 착지까지 기다리세요';return snapshot();}
  if(action==='jump'){
   try{if(mode!=='wheel')throw Error('몸통 꼭짓점 점프는 지원하지 않습니다');if(!jumpPlanner)throw Error(jumpUnavailableReason??'검증된 점프 묶음 없음');if(failed||pulse||cursor<queue.length)throw Error('정지 균형 상태에서만 점프 가능');
    const plan=jumpPlanner.start(estimate,{...original.at(-1),mode});jump=plan;jumpIndex=0;jumpWasAir=false;jumpLanded=false;jumpClearance=0;jumpOutcome=null;queue=[];cursor=0;
    status='검증된 작은 점프 · 수락한 꼭짓점으로 투영 · 6.00초';path={requested:original,accepted:plan.accepted.wheelPath.map(([x,z])=>({x,z})),apex:plan.accepted.apex,landing:plan.accepted.landing,mode,available:true,durationS:6,reason:status};
   }catch(e){status='Unavailable: '+e.message;path={...previous,requested:original,reason:status,available:false};}return snapshot();
  }
  if(failed)throw Error('Ground controller unavailable after fall');
  if(steps<20&&!initial){status='Unavailable: estimator ready admission requires 0.20 s of unchanged noisy measurements; balance continues';path={...previous,requested:original,available:false,reason:status};return snapshot();}
  if(pulse){status='Unavailable: wait for active disturbance expiry; last path retained';path={...previous,requested:original,available:false,reason:status};return snapshot();}
  if(retry===0)brakeFuture();
  const prediction=backend.fork();
  try{
  const compressed=simplifyPath(original),points=[],reasons=[],attempts=[];
  for(const point of compressed.points){
   let z=mode==='wheel'?appliedTarget.z:point.z;
   const q=profile(clamp(z,...bundle.ranges.z),clamp(pitch,...bundle.ranges.pitch),0).qref;
   const wheel=prediction.geometry([...q,0,0,0,0,0,0]).wheel;
   const next={x:clamp(point.x-(mode==='wheel'?wheel[0]:0),...bundle.ranges.x),z:clamp(z,...bundle.ranges.z),pitch:clamp(pitch,...bundle.ranges.pitch)};
   if(next.x!==point.x||next.z!==point.z||mode==='wheel')reasons.push('Local atlas/FK projection; wheel centre constrained to existing floor');
   points.push(next);
  }
  const start=estimate.slice(),referenceStart={...appliedTarget};let accepted=null,planningMs=0,reason='No checked trajectory satisfied nonlinear preview';
  // ponytail: three local retimings only; expand the candidate family after measured failures.
  for(const scale of [1,2,4]){
   if(performance.now()-started>5000){reason="Planning wall budget 5 s exceeded";break;}
   const frames=schedule(points,scale);if(frames.some(p=>keys.some(k=>p[k]<bundle.ranges[k][0]||p[k]>bundle.ranges[k][1]))){reason='Continuous reference leaves atlas domain';continue;}if(frames.length>1200){reason='Path exceeds 12 s bounded preview; use fewer/closer points';continue;}
   const x=Array(12).fill(0);indices.forEach((j,i)=>x[j]=start[i]);
   const preview=createWheelbotActions(prediction,base,bundle,{seed,initial:{truth:x,estimate:start,appliedTarget:{...appliedTarget},referenceVelocity:referenceVelocity.slice(),queue:frames,target:points.at(-1)}});
   let safe=true,s,airSeconds=0,chunkStart=performance.now(),tailPass=true;const metrics={scale,steps:0,maximumPenetrationM:0,maxTrackingError:[0,0,0],peakTorqueNm:[0,0,0],reason:null};
   for(let k=0;k<frames.length+250;k++){
    const t=performance.now();s=preview.step();planningMs+=performance.now()-t;
    metrics.steps=k+1;metrics.peakTorqueNm=s.last.u.map((v,j)=>Math.max(Math.abs(v),metrics.peakTorqueNm[j]));metrics.maximumPenetrationM=Math.max(metrics.maximumPenetrationM,s.last.physical.maximumPenetrationM);
    const errors=keys.map((key,j)=>Math.abs(s.truth[j]-s.appliedTarget[key]));metrics.maxTrackingError=errors.map((e,j)=>Math.max(e,metrics.maxTrackingError[j]));
    airSeconds=s.last.physical.wheelContactLost?airSeconds+.01:0;
    if(s.failed||s.last.physical.maximumWheelClearanceM>.005||airSeconds>.05||errors.some((e,j)=>e>[.12,.035,.15][j])){safe=false;metrics.reason=s.failed?'physical envelope':airSeconds>.05?'contact absent > 50 ms':s.last.physical.maximumWheelClearanceM>.005?'wheel clearance > 5 mm':'path tracking envelope';break;}
    if(k>=frames.length+200)tailPass&&=errors.every((e,j)=>e<=[.02,.005,.03][j])&&s.truth.slice(6).every(v=>Math.abs(v)<=.3);
    if(performance.now()-started>5000){safe=false;metrics.reason="Planning wall budget 5 s exceeded";break;}
    if(performance.now()-chunkStart>=6){await yieldTask();chunkStart=performance.now();if(token!==serial)return snapshot();}
   }
   const end=points.at(-1);
   metrics.reason??=tailPass?'checked':'tail error/rate';attempts.push(metrics);reason=metrics.reason;
   if(safe&&tailPass&&Math.abs(s.truth[0]-end.x)<=.02&&Math.abs(s.truth[1]-end.z)<=.005&&Math.abs(s.truth[2]-end.pitch)<=.03&&s.truth.slice(6).every(v=>Math.abs(v)<=.3)){accepted=frames;break;}
  }
  if(token!==serial)return snapshot();
  if(keys.some(k=>Math.abs(appliedTarget[k]-referenceStart[k])>1e-9)||estimate.some((v,j)=>Math.abs(v-start[j])>(j<5?.01:.15))){if(retry===0&&performance.now()-started<5000){status='상태 갱신 · 자동으로 한 번 더 확인합니다';return await planPath(original,{mode,action,pitch,yieldTask},token,1,started);}accepted=null;reason='State changed after bounded automatic retry; braking/balance retained';}
  if(!accepted){status='Unavailable: '+reason+(attempts.length?'; checked candidates: '+attempts.map(a=>a.reason).join(', '):'');path={...previous,requested:original,available:false,reason:status,planningMs,planningWallMs:performance.now()-started,planningBudgetMs:5000,retries:retry,attempts,compression:{input:original.length,output:compressed.points.length,maxDeviationM:compressed.maxDeviationM}};return snapshot();}
  queue=accepted;cursor=0;target={...points.at(-1)};
  const visible=points.map(p=>{if(mode==='base')return {x:p.x,z:p.z};const q=profile(p.z,p.pitch,p.x).qref,w=prediction.geometry([...q,0,0,0,0,0,0]).wheel;return{x:w[0],z:w[2]};});
  status=(reasons.length?'목표 조정 · ':'')+'경로 확인 · '+(queue.length*.01).toFixed(2)+'초 + 안정화 최대 2.50초 · 계획 '+Math.round(performance.now()-started)+' ms';
  path={requested:original,accepted:visible,mode,available:true,reason:status,planningMs,planningWallMs:performance.now()-started,planningBudgetMs:5000,retries:retry,attempts,compression:{input:original.length,output:compressed.points.length,maxDeviationM:compressed.maxDeviationM},settlingS:2.5,durationS:queue.length*.01};return snapshot();
  }finally{prediction.dispose();}
 }
 function jumpAdmission(){
  if(!jumpPlanner)return {ready:false,reason:'검증된 점프 묶음 없음'};
  if(failed||pulse||jump||cursor<queue.length)return {ready:false,reason:'정지 균형 상태에서만 점프 가능'};
  try{jumpPlanner.start(estimate,{x:estimate[0]+.03,z:.09,mode:'wheel'});return {ready:true,reason:'바퀴 중심을 선택하고 작은 점프 목표를 클릭하세요'};}
  catch(e){return {ready:false,reason:e.message};}
 }
 return {snapshot,step,requestPath,jumpAdmission,setJumpPlanner(planner){jumpPlanner=planner;},disturb({kind,direction,strength}={}){
  if(failed||pulse||jump)throw Error('Disturbance unavailable while fallen or another disturbance is active; reset is not get-up');
  if(!['push','gust','twist'].includes(kind)||![-1,1].includes(direction)||!['light','medium','strong'].includes(strength))throw Error('Invalid disturbance');
  serial++;const level=['light','medium','strong'].indexOf(strength),duration=kind==='gust'?[.2,.25,.2][level]:[.1,.2,.2][level];
  pulse={kind,direction,strength,total:Math.round(duration/.01),remaining:Math.round(duration/.01),amplitude:(kind==='twist'?[.02,.05,.1]:[.5,1.5,4])[level],units:kind==='twist'?'N m':'N'};status=kind+' applied';return snapshot();
 },cancel(){if(jump){status='현재 점프의 착지와 균형 복귀를 마칩니다';return snapshot();}serial++;queue=[];cursor=0;target={...appliedTarget};status='Future path cancelled; balance retained';return snapshot();},setTarget(request){if(!request||Object.keys(request).some(k=>!keys.includes(k))||Object.values(request).some(v=>!Number.isFinite(v)))throw Error('Finite base target required');return requestPath([{x:request.x??target.x,z:request.z??target.z}],{pitch:request.pitch??target.pitch});}};
}
