// Fixed-timestep physics is independent of canvas and low-rate text updates.
const bounded=(a,v,n=300)=>{a.push(v);if(a.length>n)a.shift();};
const percentile=(a,p)=>a.length?a.slice().sort((x,y)=>x-y)[Math.min(a.length-1,Math.floor(a.length*p))]:0;
export function pipelineLabels({liveActive=false,configuredModel=false,poseOnly=false,tracking=false,feedback=false,mode,scenario}={}){
 const plant=liveActive||configuredModel||tracking?'MuJoCo full contact':'MuJoCo wheel-contact benchmark';
 if(poseOnly)return ['Pose preview · no sensor sampling','No observer','No control',plant];
 if(tracking)return ['Exact simulator state','No observer',feedback?'Scheduled feedback':'Planned torques · no feedback',plant];
 if(mode==='contact-diagnostic')return ['Simulator state display','No observer',scenario==='torque'?'Manual torques · no feedback':'Passive · no feedback',plant];
 const control={lqr_kf:'LQR feedback',mpc_kf:'Local MPC feedback',tvlqr_kf:'Scheduled TVLQR feedback',passive:'Passive · no feedback'}[mode];
 return control?[liveActive?'Six noisy channels: five poses + wheel encoder rate':'Five noisy position channels','KF state estimate',control,plant]:['Simulator state display','No observer','No control',plant];
}
export function createRuntimeMeter(now=()=>performance.now()){
 const control=[],render=[],frames=[];let anchorWall=0,pausedWall=0,anchorSim=0,latestSim=0,lastFrame=0,active=false;
 return {start(sim=0){anchorWall=now();pausedWall=0;anchorSim=latestSim=sim;lastFrame=0;active=true;},pause(){if(active)pausedWall=(now()-anchorWall)/1000;active=false;},sampleControl(ms,sim){bounded(control,ms);latestSim=sim;},sampleRender(ms){bounded(render,ms);},frame(time){if(active&&lastFrame)bounded(frames,time-lastFrame);lastFrame=time;},reset(){control.length=render.length=frames.length=0;anchorWall=pausedWall=anchorSim=latestSim=lastFrame=0;active=false;},snapshot(){const wall=active?(now()-anchorWall)/1000:pausedWall;return{samples:control.length,controlMedianMs:percentile(control,.5),controlP95Ms:percentile(control,.95),controlMaxMs:Math.max(0,...control),renderP95Ms:percentile(render,.95),frameP95Ms:percentile(frames,.95),fps:frames.length?1000/(frames.reduce((a,b)=>a+b,0)/frames.length):0,simulatedSeconds:latestSim-anchorSim,wallSeconds:wall,realTimeFactor:active&&wall>.1?(latestSim-anchorSim)/wall:null,active};}};
}
export function createLiveView(){
 const canvas=document.getElementById('view'),ctx=canvas.getContext('2d'),traces=[];let lastTrace=-1;
 const resize=()=>{const r=canvas.getBoundingClientRect(),ratio=Math.min(devicePixelRatio||1,1.5);const w=Math.max(1,Math.round(r.width*ratio)),h=Math.max(1,Math.round(r.height*ratio));if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;}};
 new ResizeObserver(resize).observe(canvas);resize();
 function drawScene(geoms,x,{live=false,comLocal=.02,showCOM=true,goal=0}={}){
  resize();const W=canvas.width,H=canvas.height,ground=H-43,scale=Math.min(H*.98,W*.8),camera=live?0:(geoms.find(g=>g.name==='torso_visual')?.position[0]??0);
  const px=v=>W/2+(v-camera)*scale,py=v=>ground-v*scale;
  ctx.clearRect(0,0,W,H);ctx.fillStyle='#fbfdff';ctx.fillRect(0,0,W,H);
  ctx.font=`${Math.max(10,H/36)}px system-ui`;ctx.lineWidth=1;ctx.strokeStyle='#dfe8ef';ctx.fillStyle='#7690a0';
  for(let i=-10;i<=10;i++){const value=i*.1,screen=px(value);if(screen<0||screen>W)continue;ctx.beginPath();ctx.moveTo(screen,ground);ctx.lineTo(screen,ground+6);ctx.stroke();if(i%2===0)ctx.fillText(value.toFixed(1)+' m',screen-12,ground+22);}
  ctx.strokeStyle='#7892a4';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(0,ground);ctx.lineTo(W,ground);ctx.stroke();
  if(live){ctx.setLineDash([4,4]);ctx.strokeStyle='#daa653';ctx.beginPath();ctx.moveTo(px(goal),ground);ctx.lineTo(px(goal),ground-H*.75);ctx.stroke();ctx.setLineDash([]);}
  const ordered=geoms.filter(g=>g.type!==0).sort((a,b)=>(a.name==='torso_visual'?1:0)-(b.name==='torso_visual'?1:0));
  for(const g of ordered){ctx.save();ctx.translate(px(g.position[0]),py(g.position[2]));ctx.rotate(-Math.atan2(g.rotation[6],g.rotation[0]));ctx.fillStyle=g.name==='torso_visual'?'#e8f0f5':g.name==='wheel_visual'?'#344b60':g.name.includes('link')?'#4385b7':'#b3946c';ctx.strokeStyle=g.name==='torso_visual'?'#54778f':'#285778';ctx.lineWidth=2;
   if(g.name==='wheel_visual'){const radius=g.size[0]*scale;ctx.beginPath();ctx.arc(0,0,radius,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.strokeStyle='#d2e2ed';ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(radius,0);ctx.stroke();}
   else if(g.type===4){ctx.beginPath();ctx.ellipse(0,0,g.size[0]*scale,g.size[2]*scale,0,0,Math.PI*2);ctx.fill();ctx.stroke();}
   else if(g.type===6){ctx.fillRect(-g.size[0]*scale,-g.size[2]*scale,2*g.size[0]*scale,2*g.size[2]*scale);ctx.strokeRect(-g.size[0]*scale,-g.size[2]*scale,2*g.size[0]*scale,2*g.size[2]*scale);}
   else if(g.type===5){ctx.lineWidth=Math.max(3,2*g.size[0]*scale);ctx.lineCap='round';ctx.beginPath();ctx.moveTo(0,-g.size[1]*scale);ctx.lineTo(0,g.size[1]*scale);ctx.stroke();}
   ctx.restore();
  }
  const torso=geoms.find(g=>g.name==='torso_visual');
  // The root is the hip axis. Compute knee from the actual upper-link transform.
  const upper=geoms.find(g=>g.name==='upper_link_visual');const hip=[x[0],x[1]],knee=upper?[upper.position[0]-upper.rotation[2]*upper.size[1],upper.position[2]-upper.rotation[8]*upper.size[1]]:hip;
  for(const [point,radius]of[[hip,9],[knee,7]]){ctx.fillStyle='#c7514d';ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.beginPath();ctx.arc(px(point[0]),py(point[1]),radius*H/400,0,Math.PI*2);ctx.fill();ctx.stroke();}
  if(showCOM&&torso){const com=[x[0]+Math.sin(x[2])*comLocal,x[1]+Math.cos(x[2])*comLocal];ctx.strokeStyle='#d39431';ctx.lineWidth=2;ctx.beginPath();ctx.arc(px(com[0]),py(com[1]),5*H/400,0,Math.PI*2);ctx.moveTo(px(com[0])-8,py(com[1]));ctx.lineTo(px(com[0])+8,py(com[1]));ctx.moveTo(px(com[0]),py(com[1])-8);ctx.lineTo(px(com[0]),py(com[1])+8);ctx.stroke();ctx.fillStyle='#8c651f';ctx.fillText('base COM',px(com[0])+12,py(com[1])-10);}
 }
 function sample(s,dt){const t=s.steps*dt;if(lastTrace>=0&&t<lastTrace){traces.length=0;}lastTrace=t;if(traces.length&&t-traces.at(-1).t<.04)return;bounded(traces,{t,x:s.truth[0],goal:s.goal??0,pitch:s.truth[2]*180/Math.PI,estimate:s.estimate?.[2]*180/Math.PI},240);}
 function plot(id,keys){const c=document.getElementById(id),g=c.getContext('2d'),W=c.width,H=c.height;g.clearRect(0,0,W,H);g.fillStyle='#fbfdff';g.fillRect(0,0,W,H);if(traces.length<2)return;const values=traces.flatMap(z=>keys.map(k=>z[k]).filter(Number.isFinite));let lo=Math.min(...values),hi=Math.max(...values);const margin=Math.max((hi-lo)*.15,id==='position-chart'?.004:.1);lo-=margin;hi+=margin;const t0=traces[0].t,t1=Math.max(t0+.1,traces.at(-1).t);g.font='10px system-ui';g.fillStyle='#8296a5';g.fillText(hi.toFixed(2),3,11);g.fillText(lo.toFixed(2),3,H-4);for(let j=0;j<keys.length;j++){g.strokeStyle=j?'#ce963d':'#237ab3';g.setLineDash(j?[4,3]:[]);g.lineWidth=1.6;g.beginPath();let first=true;for(const z of traces){if(!Number.isFinite(z[keys[j]]))continue;const x=34+(W-42)*(z.t-t0)/(t1-t0),y=H-12-(H-25)*(z[keys[j]]-lo)/(hi-lo);if(first){g.moveTo(x,y);first=false;}else g.lineTo(x,y);}g.stroke();}g.setLineDash([]);}
 return{drawScene,sample,drawCharts(){plot('position-chart',['x','goal']);plot('pitch-chart',['pitch','estimate']);},clear(){traces.length=0;lastTrace=-1;}};
}
